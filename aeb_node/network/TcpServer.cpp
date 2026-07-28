/**
 * @file TcpServer.cpp
 * @brief Listening socket, event loop and component wiring for the dev server.
 *
 * The heavy lifting lives in the collaborators: @ref aeb::net::FrameQueue owns
 * the producer hand-off, @ref aeb::net::ClientSession owns one connection, and
 * @ref aeb::net::WakePipe makes a blocking @c poll interruptible. This file is
 * responsible only for the listening socket, the event loop that drives those
 * collaborators, and the public counters.
 */

#include "network/TcpServer.hpp"

#include <poll.h>

#include <atomic>
#include <cerrno>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include "network/ClientSession.hpp"
#include "network/FileDescriptor.hpp"
#include "network/FrameQueue.hpp"
#include "network/ListenSocket.hpp"
#include "network/WakePipe.hpp"

namespace aeb {

/**
 * @brief Private state of a @ref TcpServer instance.
 *
 * Locking discipline: @c state_mutex guards the counters and the error string
 * and is never held across a syscall. The listening socket, the wake pipe and
 * the session are touched only by the I/O thread once @ref TcpServer::start has
 * returned, so they need no lock of their own. The queue provides its own.
 */
struct TcpServer::Impl {
    /**
     * @brief Construct with a validated configuration.
     * @param cfg Configuration snapshot.
     */
    explicit Impl(TcpServerConfig cfg) : config{std::move(cfg)}, queue{config.queue_capacity} {}

    /** @brief Immutable configuration captured at construction. */
    TcpServerConfig config;

    /** @brief Producer/consumer hand-off, bounded and drop-oldest. */
    net::FrameQueue queue;

    /** @brief I/O thread; joinable exactly while running. */
    std::thread worker;

    /** @brief Set to request I/O thread shutdown. */
    std::atomic<bool> stop_requested{false};

    /** @brief True between a successful start and the I/O thread exiting. */
    std::atomic<bool> running{false};

    /** @brief Informational mirror of session validity, readable by any thread. */
    std::atomic<bool> client_connected{false};

    /** @brief Listening socket, owned by this object. */
    net::FileDescriptor listen_fd;

    /** @brief Wakeup channel for producers and for shutdown. */
    net::WakePipe wake;

    /** @brief The current client session; empty when no viewer is attached. */
    net::ClientSession session;

    /** @brief Guards @ref stats and @ref last_error. */
    mutable std::mutex state_mutex;

    /** @brief Runtime counters. */
    TcpServerStats stats;

    /** @brief Description of the most recent failure. */
    std::string last_error;

    /**
     * @brief Record a failure description, ignoring empty messages.
     * @param message Description of the failure.
     */
    void setError(std::string message)
    {
        if (message.empty()) {
            return;
        }
        const std::lock_guard<std::mutex> lock(state_mutex);
        last_error = std::move(message);
    }

    /**
     * @brief Accept a pending connection, replacing any existing session.
     *
     * Replacing rather than rejecting lets a viewer that crashed and left a
     * half-open socket behind reconnect immediately, instead of waiting for TCP
     * keepalive to expire.
     */
    void acceptClient()
    {
        std::string error;
        net::ClientSession accepted = net::ClientSession::accept(listen_fd.get(), config, error);
        setError(std::move(error));
        if (!accepted.valid()) {
            return;
        }

        if (session.valid()) {
            endSession(false);
        }
        session = std::move(accepted);
        client_connected.store(true, std::memory_order_release);

        // A new session begins at a packet boundary: drop frames that predate
        // the connection so the viewer never renders stale geometry.
        queue.clear();

        const std::lock_guard<std::mutex> lock(state_mutex);
        ++stats.client_connects;
        stats.client_connected = true;
    }

    /**
     * @brief Terminate the current session and update the counters.
     * @param error_related @c true if the session ended due to a socket error.
     */
    void endSession(bool error_related) noexcept
    {
        if (!session.valid()) {
            return;
        }
        session.close();
        client_connected.store(false, std::memory_order_release);

        const std::lock_guard<std::mutex> lock(state_mutex);
        ++stats.client_disconnects;
        if (error_related) {
            ++stats.send_errors;
        }
        stats.client_connected = false;
    }

    /**
     * @brief Transmit whatever the socket will currently accept.
     */
    void serviceOutput()
    {
        net::SessionProgress progress;
        std::string error;
        const net::SessionResult result = session.pump(queue, progress, error);

        if (progress.bytes_sent != 0U || progress.frames_sent != 0U) {
            const std::lock_guard<std::mutex> lock(state_mutex);
            stats.bytes_sent += progress.bytes_sent;
            stats.frames_sent += progress.frames_sent;
        }

        if (result == net::SessionResult::Failed) {
            setError(std::move(error));
            endSession(true);
        }
    }

    /**
     * @brief Handle the events @c poll reported for the client socket.
     * @param revents Reported event mask.
     */
    void serviceClientEvents(short revents)
    {
        if ((revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
            endSession((revents & POLLERR) != 0);
            return;
        }
        if ((revents & POLLIN) == 0) {
            return;
        }
        const net::SessionResult result = session.drainInput();
        if (result == net::SessionResult::Closed) {
            endSession(false);
        } else if (result == net::SessionResult::Failed) {
            endSession(true);
        }
    }

    /**
     * @brief I/O thread body: one @c poll loop over the wake pipe, the
     *        listening socket and the client socket.
     */
    void run()
    {
        while (!stop_requested.load(std::memory_order_acquire)) {
            pollfd fds[3];
            nfds_t count = 0U;

            const nfds_t wake_index = count;
            fds[count++] = pollfd{wake.readFd(), POLLIN, 0};

            const nfds_t listen_index = count;
            fds[count++] = pollfd{listen_fd.get(), POLLIN, 0};

            const nfds_t client_index = count;
            if (session.valid()) {
                fds[count++] = pollfd{session.fd(), session.pollEvents(!queue.empty()), 0};
            }

            if (::poll(fds, count, -1 /*block until an event*/) < 0) {
                if (errno == EINTR) {
                    continue;
                }
                setError(net::describeErrno("poll()", errno));
                break;
            }

            if ((fds[wake_index].revents & POLLIN) != 0) {
                wake.drain();
            }
            if (session.valid() && client_index < count) {
                serviceClientEvents(fds[client_index].revents);
            }
            if ((fds[listen_index].revents & POLLIN) != 0) {
                acceptClient();
            }

            if (session.valid()) {
                serviceOutput();
            } else {
                queue.clear();  // No viewer: never let the queue hold memory.
            }
        }

        endSession(false);
        queue.clear();
        running.store(false, std::memory_order_release);
    }
};

TcpServer::TcpServer(TcpServerConfig config)
    : impl_{std::make_unique<Impl>(std::move(config))}
{
}

TcpServer::~TcpServer()
{
    stop();
}

bool TcpServer::start()
{
    if (impl_->running.load(std::memory_order_acquire)) {
        impl_->setError("start() called while already running");
        return false;
    }
    impl_->stop_requested.store(false, std::memory_order_release);

    if (!impl_->wake.open()) {
        impl_->setError(net::describeErrno("pipe()", errno));
        impl_->wake.close();
        return false;
    }

    std::string error;
    impl_->listen_fd = net::makeListenSocket(impl_->config, error);
    if (!impl_->listen_fd.valid()) {
        impl_->setError(std::move(error));
        impl_->wake.close();
        return false;
    }

    impl_->running.store(true, std::memory_order_release);
    try {
        impl_->worker = std::thread([this] { impl_->run(); });
    } catch (const std::system_error& e) {
        impl_->running.store(false, std::memory_order_release);
        impl_->listen_fd.reset();
        impl_->wake.close();
        impl_->setError(std::string{"failed to spawn I/O thread: "} + e.what());
        return false;
    }
    return true;
}

void TcpServer::stop() noexcept
{
    impl_->stop_requested.store(true, std::memory_order_release);
    impl_->wake.signal();

    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }

    impl_->listen_fd.reset();
    impl_->wake.close();
    impl_->client_connected.store(false, std::memory_order_release);
    impl_->running.store(false, std::memory_order_release);
}

void TcpServer::publish(const ScanFrame& frame)
{
    {
        const std::lock_guard<std::mutex> lock(impl_->state_mutex);
        ++impl_->stats.frames_published;
    }

    // Discard early when nobody is listening, so an absent viewer costs the
    // acquisition thread two atomic loads and nothing else.
    if (!impl_->client_connected.load(std::memory_order_acquire) ||
        !impl_->running.load(std::memory_order_acquire)) {
        const std::lock_guard<std::mutex> lock(impl_->state_mutex);
        ++impl_->stats.frames_dropped;
        return;
    }

    const std::size_t dropped = impl_->queue.push(frame);
    if (dropped != 0U) {
        const std::lock_guard<std::mutex> lock(impl_->state_mutex);
        impl_->stats.frames_dropped += dropped;
    }

    impl_->wake.signal();
}

bool TcpServer::isRunning() const noexcept
{
    return impl_->running.load(std::memory_order_acquire);
}

bool TcpServer::hasClient() const noexcept
{
    return impl_->client_connected.load(std::memory_order_acquire);
}

TcpServerStats TcpServer::stats() const noexcept
{
    const std::lock_guard<std::mutex> lock(impl_->state_mutex);
    return impl_->stats;
}

std::string TcpServer::lastError() const
{
    const std::lock_guard<std::mutex> lock(impl_->state_mutex);
    return impl_->last_error;
}

}  // namespace aeb
