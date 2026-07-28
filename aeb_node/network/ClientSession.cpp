/**
 * @file ClientSession.cpp
 * @brief Implementation of the single-client outbound streaming session.
 */

#include "network/ClientSession.hpp"

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstring>
#include <utility>

#include "common/Protocol.hpp"

namespace aeb::net {
namespace {

/** @brief Flag suppressing SIGPIPE on write, where the platform provides it. */
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

/** @brief Initial reservation for the serialisation buffer, in bytes. */
constexpr std::size_t kSendBufferReserve = 64U * 1024U;

/**
 * @brief Format @c errno as "context: message".
 * @param context Description of the failed operation.
 * @param err     Captured @c errno value.
 * @return Diagnostic string.
 */
[[nodiscard]] std::string describeErrno(const char* context, int err)
{
    return std::string{context} + ": " + std::strerror(err);
}

/**
 * @brief Apply the configured socket options to an accepted connection.
 *
 * Option failures are tolerated: they affect latency or buffering, never
 * correctness, and refusing a viewer over them would be disproportionate.
 *
 * @param fd     Accepted socket.
 * @param config Tuning to apply.
 */
void applySocketOptions(int fd, const TcpServerConfig& config) noexcept
{
    if (config.tcp_nodelay) {
        const int enable = 1;
        (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
    }
    if (config.send_buffer_bytes > 0) {
        (void)::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &config.send_buffer_bytes,
                           sizeof(config.send_buffer_bytes));
    }
#ifdef SO_NOSIGPIPE
    {
        const int enable = 1;
        (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enable, sizeof(enable));
    }
#endif
}

}  // namespace

ClientSession::ClientSession(FileDescriptor socket) noexcept : socket_{std::move(socket)}
{
    send_buffer_.reserve(kSendBufferReserve);
}

ClientSession ClientSession::accept(int listen_fd, const TcpServerConfig& config,
                                    std::string& error)
{
    for (;;) {
        FileDescriptor accepted{::accept(listen_fd, nullptr, nullptr)};

        if (!accepted.valid()) {
            if (errno == EINTR) {
                continue;
            }
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                error = describeErrno("accept()", errno);
            }
            return ClientSession{};
        }

        if (!setNonBlocking(accepted.get())) {
            error = describeErrno("fcntl(O_NONBLOCK) on client socket", errno);
            continue;  // Socket closed by the FileDescriptor destructor.
        }

        applySocketOptions(accepted.get(), config);
        return ClientSession{std::move(accepted)};
    }
}

short ClientSession::pollEvents(bool has_pending_frames) const noexcept
{
    short events = POLLIN;
    if (has_pending_frames || send_offset_ < send_buffer_.size()) {
        events = static_cast<short>(events | POLLOUT);
    }
    return events;
}

SessionResult ClientSession::pump(FrameQueue& queue, SessionProgress& progress,
                                  std::string& error)
{
    if (!socket_.valid()) {
        return SessionResult::Failed;
    }

    for (;;) {
        if (send_offset_ >= send_buffer_.size()) {
            if (!queue.pop(staging_)) {
                return SessionResult::Progressed;  // Nothing left to send.
            }
            proto::ProtocolEncoder::encode(staging_, send_buffer_);
            send_offset_ = 0U;
        }

        const ssize_t written = ::send(socket_.get(), send_buffer_.data() + send_offset_,
                                       send_buffer_.size() - send_offset_, kSendFlags);
        if (written > 0) {
            send_offset_ += static_cast<std::size_t>(written);
            progress.bytes_sent += static_cast<std::uint64_t>(written);
            if (send_offset_ >= send_buffer_.size()) {
                ++progress.frames_sent;
            }
            continue;
        }

        if (written < 0 && errno == EINTR) {
            continue;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return SessionResult::Blocked;  // Resume on the next POLLOUT.
        }

        error = describeErrno("send()", errno);
        return SessionResult::Failed;
    }
}

SessionResult ClientSession::drainInput()
{
    if (!socket_.valid()) {
        return SessionResult::Failed;
    }

    std::uint8_t scratch[256];
    for (;;) {
        const ssize_t n = ::recv(socket_.get(), scratch, sizeof(scratch), 0);
        if (n > 0) {
            continue;  // Unexpected but harmless; the protocol is one-way.
        }
        if (n == 0) {
            return SessionResult::Closed;
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return SessionResult::Progressed;
        }
        return SessionResult::Failed;
    }
}

void ClientSession::close() noexcept
{
    socket_.reset();
    send_buffer_.clear();
    send_offset_ = 0U;
}

}  // namespace aeb::net
