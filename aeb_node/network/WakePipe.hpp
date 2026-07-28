/**
 * @file WakePipe.hpp
 * @brief Self-pipe used to interrupt a blocking @c poll from another thread.
 *
 * The I/O thread blocks in @c poll with an infinite timeout: no busy-waiting,
 * no arbitrary timeout tuning. This class provides the readable descriptor that
 * makes that safe, letting a producer or a shutdown request wake the loop
 * immediately.
 *
 * Internal header of the networking layer.
 */

#ifndef AEB_NETWORK_WAKEPIPE_HPP
#define AEB_NETWORK_WAKEPIPE_HPP

#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>

#include "network/FileDescriptor.hpp"

namespace aeb::net {

/**
 * @brief A non-blocking pipe pair used purely as a wakeup signal.
 *
 * @ref signal is safe to call from any thread and at any rate: it coalesces, so
 * at most one byte is ever in flight and a fast producer can never fill the
 * pipe buffer and block.
 */
class WakePipe {
public:
    /** @brief Construct closed. Call @ref open before use. */
    WakePipe() noexcept = default;

    WakePipe(const WakePipe&) = delete;
    WakePipe& operator=(const WakePipe&) = delete;
    WakePipe(WakePipe&&) = delete;
    WakePipe& operator=(WakePipe&&) = delete;

    /**
     * @brief Create the pipe and set both ends non-blocking.
     * @return @c true on success; on failure nothing is left open and @c errno
     *         describes the cause.
     */
    [[nodiscard]] bool open()
    {
        int fds[2] = {-1, -1};
        if (::pipe(fds) != 0) {
            return false;
        }
        FileDescriptor read_end{fds[0]};
        FileDescriptor write_end{fds[1]};

        if (!setNonBlocking(read_end.get()) || !setNonBlocking(write_end.get())) {
            return false;
        }

        read_end_ = std::move(read_end);
        write_end_ = std::move(write_end);
        return true;
    }

    /** @brief Close both ends. Safe to call when already closed. */
    void close() noexcept
    {
        read_end_.reset();
        write_end_.reset();
        pending_.store(false, std::memory_order_release);
    }

    /** @brief Descriptor to register with @c poll for @c POLLIN. */
    [[nodiscard]] int readFd() const noexcept { return read_end_.get(); }

    /** @brief Whether the pipe is open. */
    [[nodiscard]] bool valid() const noexcept { return read_end_.valid(); }

    /**
     * @brief Wake the waiting thread.
     *
     * Idempotent while a wakeup is already pending, so calling it once per
     * acquired frame costs a single atomic exchange in the common case.
     */
    void signal() noexcept
    {
        if (!write_end_.valid()) {
            return;
        }
        if (pending_.exchange(true, std::memory_order_acq_rel)) {
            return;
        }
        const std::uint8_t byte = 1U;
        ssize_t written = 0;
        do {
            written = ::write(write_end_.get(), &byte, sizeof(byte));
        } while (written < 0 && errno == EINTR);

        if (written <= 0) {
            // The wakeup was not delivered; allow the next caller to retry
            // rather than latching the coalescing flag forever.
            pending_.store(false, std::memory_order_release);
        }
    }

    /**
     * @brief Consume every pending wakeup byte.
     *
     * Called by the I/O thread after @c poll reports @c POLLIN on @ref readFd.
     */
    void drain() noexcept
    {
        std::uint8_t scratch[64];
        for (;;) {
            const ssize_t n = ::read(read_end_.get(), scratch, sizeof(scratch));
            if (n > 0 || (n < 0 && errno == EINTR)) {
                continue;
            }
            break;
        }
        pending_.store(false, std::memory_order_release);
    }

private:
    /** @brief Readable end, polled by the I/O thread. */
    FileDescriptor read_end_;

    /** @brief Writable end, used by producers and by shutdown. */
    FileDescriptor write_end_;

    /** @brief True while an undelivered wakeup byte is in flight. */
    std::atomic<bool> pending_{false};
};

}  // namespace aeb::net

#endif  // AEB_NETWORK_WAKEPIPE_HPP
