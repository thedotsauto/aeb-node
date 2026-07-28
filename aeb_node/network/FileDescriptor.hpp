/**
 * @file FileDescriptor.hpp
 * @brief Move-only RAII ownership of a POSIX file descriptor.
 *
 * Every socket and pipe in the networking layer is held by one of these, so no
 * error path can leak a descriptor and the owner of each descriptor is always
 * unambiguous. This is an internal header of the networking layer; it is not
 * part of the public interface of @ref aeb::TcpServer.
 */

#ifndef AEB_NETWORK_FILEDESCRIPTOR_HPP
#define AEB_NETWORK_FILEDESCRIPTOR_HPP

#include <fcntl.h>
#include <unistd.h>

namespace aeb::net {

/**
 * @brief Sole owner of a POSIX file descriptor.
 *
 * Copying is forbidden because two owners would double-close. Moving transfers
 * ownership and leaves the source empty.
 */
class FileDescriptor {
public:
    /** @brief Construct an empty owner. */
    FileDescriptor() noexcept = default;

    /**
     * @brief Take ownership of @p fd.
     * @param fd Descriptor to own, or -1 for none.
     */
    explicit FileDescriptor(int fd) noexcept : fd_{fd} {}

    /** @brief Closes the owned descriptor, if any. */
    ~FileDescriptor() { reset(); }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    /**
     * @brief Transfer ownership from @p other.
     * @param other Source, left empty.
     */
    FileDescriptor(FileDescriptor&& other) noexcept : fd_{other.fd_} { other.fd_ = -1; }

    /**
     * @brief Transfer ownership from @p other, closing any current descriptor.
     * @param other Source, left empty.
     * @return Reference to this object.
     */
    FileDescriptor& operator=(FileDescriptor&& other) noexcept
    {
        if (this != &other) {
            reset();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    /** @brief The owned descriptor, or -1 when empty. */
    [[nodiscard]] int get() const noexcept { return fd_; }

    /** @brief Whether a descriptor is owned. */
    [[nodiscard]] bool valid() const noexcept { return fd_ >= 0; }

    /**
     * @brief Close the owned descriptor and become empty.
     *
     * @c EINTR is not retried: on Linux the descriptor is already released when
     * @c close returns, and retrying risks closing a descriptor that another
     * thread has since been given.
     */
    void reset() noexcept
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    /**
     * @brief Relinquish ownership without closing.
     * @return The descriptor, or -1.
     */
    [[nodiscard]] int release() noexcept
    {
        const int fd = fd_;
        fd_ = -1;
        return fd;
    }

private:
    /** @brief Owned descriptor, -1 when empty. */
    int fd_{-1};
};

/**
 * @brief Put a descriptor into non-blocking mode.
 *
 * Mandatory for every descriptor in this layer: a blocking @c accept or
 * @c send would stall the I/O thread and, in turn, delay shutdown.
 *
 * @param fd Descriptor to modify.
 * @return @c true on success; on failure @c errno describes the cause.
 */
[[nodiscard]] inline bool setNonBlocking(int fd) noexcept
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

}  // namespace aeb::net

#endif  // AEB_NETWORK_FILEDESCRIPTOR_HPP
