/**
 * @file ClientSession.hpp
 * @brief One connected visualisation client: socket, encoder and send pump.
 *
 * Separating the session from @ref aeb::TcpServer keeps two very different
 * responsibilities apart: the server owns the listening socket and the event
 * loop, the session owns exactly one client connection and everything needed to
 * push bytes into it.
 *
 * Internal header of the networking layer.
 */

#ifndef AEB_NETWORK_CLIENTSESSION_HPP
#define AEB_NETWORK_CLIENTSESSION_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "common/Types.hpp"
#include "network/FileDescriptor.hpp"
#include "network/FrameQueue.hpp"
#include "network/TcpServer.hpp"

namespace aeb::net {

/**
 * @brief Result of a session I/O operation.
 */
enum class SessionResult {
    Progressed,  ///< Work was done and the session remains healthy.
    Blocked,     ///< The socket accepted no more data; wait for @c POLLOUT.
    Closed,      ///< The peer closed the connection in an orderly manner.
    Failed       ///< A socket error occurred; the session must be discarded.
};

/**
 * @brief Bytes and frames transferred by a single pump call.
 */
struct SessionProgress {
    /** @brief Payload bytes written to the socket. */
    std::uint64_t bytes_sent{0U};

    /** @brief Frames whose final byte was written. */
    std::uint64_t frames_sent{0U};
};

/**
 * @brief Owns one accepted client socket and its outbound byte stream.
 *
 * Move-only, so exactly one owner exists at a time. Destruction closes the
 * socket. All operations are non-blocking: a stalled viewer produces
 * @ref SessionResult::Blocked, never a stalled thread.
 *
 * Not thread-safe; it is used exclusively by the server's I/O thread.
 */
class ClientSession {
public:
    /** @brief Construct an empty session holding no socket. */
    ClientSession() = default;

    /**
     * @brief Adopt an accepted, already configured socket.
     * @param socket Connected socket, transferred into the session.
     */
    explicit ClientSession(FileDescriptor socket) noexcept;

    ClientSession(const ClientSession&) = delete;
    ClientSession& operator=(const ClientSession&) = delete;
    ClientSession(ClientSession&&) noexcept = default;
    ClientSession& operator=(ClientSession&&) noexcept = default;

    /**
     * @brief Accept a pending connection and apply the socket options.
     *
     * @param listen_fd   Listening socket in non-blocking mode.
     * @param config      Socket tuning to apply to the accepted connection.
     * @param[out] error  Receives a description if accepting failed for a
     *                    reason other than "no pending connection".
     * @return A valid session, or an empty one if nothing was accepted.
     */
    [[nodiscard]] static ClientSession accept(int listen_fd, const TcpServerConfig& config,
                                              std::string& error);

    /** @brief Whether a socket is held. */
    [[nodiscard]] bool valid() const noexcept { return socket_.valid(); }

    /** @brief The socket descriptor, or -1 when empty. */
    [[nodiscard]] int fd() const noexcept { return socket_.get(); }

    /**
     * @brief Events to register with @c poll for this session.
     * @param has_pending_frames Whether the outbound queue is non-empty.
     * @return @c POLLIN, plus @c POLLOUT when there is anything to write.
     */
    [[nodiscard]] short pollEvents(bool has_pending_frames) const noexcept;

    /**
     * @brief Encode and transmit queued frames until the socket blocks.
     *
     * Honours partial writes: a frame split across several @c send calls is
     * resumed exactly where it stopped.
     *
     * @param queue          Source of frames to transmit.
     * @param[out] progress  Accumulates the bytes and frames transferred.
     * @param[out] error     Receives a description on @ref SessionResult::Failed.
     * @return Outcome of the pump.
     */
    [[nodiscard]] SessionResult pump(FrameQueue& queue, SessionProgress& progress,
                                     std::string& error);

    /**
     * @brief Read and discard inbound data to detect end of stream.
     *
     * The protocol is unidirectional; inbound bytes exist only so that a
     * disconnect is noticed promptly rather than on the next failed write.
     *
     * @return @ref SessionResult::Closed on orderly shutdown by the peer.
     */
    [[nodiscard]] SessionResult drainInput();

    /** @brief Close the socket and discard any partially sent frame. */
    void close() noexcept;

private:
    /** @brief The connected socket. */
    FileDescriptor socket_;

    /** @brief Serialised bytes of the frame currently being transmitted. */
    std::vector<std::uint8_t> send_buffer_;

    /** @brief Bytes of @ref send_buffer_ already written. */
    std::size_t send_offset_{0U};

    /** @brief Scratch frame reused while dequeuing, to avoid reallocation. */
    ScanFrame staging_;
};

}  // namespace aeb::net

#endif  // AEB_NETWORK_CLIENTSESSION_HPP
