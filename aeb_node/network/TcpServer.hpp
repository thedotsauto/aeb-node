/**
 * @file TcpServer.hpp
 * @brief Development-only TCP streaming server for @ref aeb::ScanFrame.
 *
 * @ref aeb::TcpServer publishes scan frames to a single remote visualisation
 * client (the Mac viewer) using the binary protocol defined in
 * @c common/Protocol.hpp.
 *
 * @section safety Safety boundary
 *
 * This component is **development and visualisation infrastructure only**. The
 * obstacle detection and emergency braking logic added in Phase 3 must never
 * include this header, must never call it, and must never observe its state.
 * The dependency arrow points one way:
 *
 * @code
 * Lidar -> ScanFrame -> [ perception / braking ]   (safety path)
 *                    \-> TcpServer                 (development path)
 * @endcode
 *
 * Consequently the server is designed so that *no* client behaviour can affect
 * the producer:
 *
 *  - @ref aeb::TcpServer::publish never blocks, never performs I/O and never
 *    fails in a way the caller must handle;
 *  - the outbound queue is bounded, and overflow drops the **oldest** frames,
 *    because for a live visualiser stale data is worthless;
 *  - all socket work happens on the server's own thread;
 *  - a disconnect, a half-open socket or a stalled client is absorbed silently
 *    and acquisition continues at full rate.
 *
 * @section threading Threading model
 *
 * One internal I/O thread owns the listening socket and the client socket and
 * runs a @c poll loop. @ref aeb::TcpServer::publish may be called from any
 * thread, typically the lidar acquisition thread.
 */

#ifndef AEB_NETWORK_TCPSERVER_HPP
#define AEB_NETWORK_TCPSERVER_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "common/Types.hpp"

namespace aeb {

/**
 * @brief Static configuration for a @ref TcpServer instance.
 */
struct TcpServerConfig {
    /**
     * @brief Local address to bind.
     *
     * @c "0.0.0.0" listens on every interface. Restrict this on a vehicle
     * build, or better, do not build the networking layer at all.
     */
    std::string bind_address{"0.0.0.0"};

    /** @brief TCP port to listen on. */
    std::uint16_t port{7000U};

    /**
     * @brief Maximum number of frames buffered for transmission.
     *
     * Bounds worst-case memory: a C1 revolution is roughly 10 KB encoded, so
     * the default caps the outbound buffer at a few hundred kilobytes. When the
     * queue is full the oldest frame is discarded.
     */
    std::size_t queue_capacity{16U};

    /**
     * @brief Disable Nagle's algorithm on the client socket.
     *
     * Frames are sent as whole packets at ~10 Hz; coalescing only adds latency
     * to the visualisation.
     */
    bool tcp_nodelay{true};

    /**
     * @brief Kernel socket send buffer size in bytes, or 0 to leave the default.
     *
     * A modest buffer is preferred: a large one merely hides a slow client and
     * lets it display data that is seconds old.
     */
    int send_buffer_bytes{262144};
};

/**
 * @brief Runtime counters for a @ref TcpServer instance.
 */
struct TcpServerStats {
    /** @brief Frames handed to @ref TcpServer::publish. */
    std::uint64_t frames_published{0U};

    /** @brief Frames discarded because the outbound queue was full. */
    std::uint64_t frames_dropped{0U};

    /** @brief Frames fully written to a client socket. */
    std::uint64_t frames_sent{0U};

    /** @brief Total payload bytes written. */
    std::uint64_t bytes_sent{0U};

    /** @brief Clients accepted since construction. */
    std::uint64_t client_connects{0U};

    /** @brief Client sessions terminated, for any reason. */
    std::uint64_t client_disconnects{0U};

    /** @brief Socket write failures that ended a session. */
    std::uint64_t send_errors{0U};

    /** @brief Whether a client is connected right now. */
    bool client_connected{false};
};

/**
 * @brief Single-client TCP publisher for scan frames.
 *
 * Owns the listening socket, the accepted client socket, the outbound queue and
 * the I/O thread. Lifetime is RAII: every descriptor is closed by @ref stop or
 * by the destructor, including on all error paths.
 *
 * Non-copyable and non-movable, because the I/O thread captures @c this.
 *
 * @code
 * aeb::TcpServerConfig cfg;
 * cfg.port = 7000;
 *
 * aeb::TcpServer server{cfg};
 * if (!server.start()) {
 *     std::cerr << server.lastError() << '\n';
 *     return 1;
 * }
 *
 * // Fed from the lidar callback; returns immediately whether or not a
 * // viewer is attached.
 * lidar.start([&server](const aeb::ScanFrame& f) { server.publish(f); });
 * @endcode
 */
class TcpServer {
public:
    /**
     * @brief Construct an idle server. No socket is created.
     * @param config Configuration snapshot, copied into the instance.
     */
    explicit TcpServer(TcpServerConfig config);

    /**
     * @brief Stops the server and closes every descriptor.
     *
     * Blocks until the I/O thread has joined.
     */
    ~TcpServer();

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;
    TcpServer(TcpServer&&) = delete;
    TcpServer& operator=(TcpServer&&) = delete;

    /**
     * @brief Create the listening socket and start the I/O thread.
     *
     * Binding and listening happen synchronously so that a port conflict is
     * reported to the caller immediately.
     *
     * @return @c true if the server is listening; @c false on failure, in which
     *         case no descriptor is left open and @ref lastError explains why.
     */
    [[nodiscard]] bool start();

    /**
     * @brief Stop the server, close all sockets and join the I/O thread.
     *
     * Safe to call when stopped and safe to call repeatedly.
     */
    void stop() noexcept;

    /**
     * @brief Queue a frame for transmission.
     *
     * Non-blocking and total: it always succeeds from the caller's point of
     * view. If no client is attached, or the queue is full, the frame is
     * discarded and only a counter changes. This is what allows the lidar
     * callback to call it directly.
     *
     * @param frame Frame to transmit. Copied into the queue; the caller keeps
     *              ownership of the original.
     *
     * @note Serialisation happens on the I/O thread, not here, to keep the
     *       producer's critical path as short as possible.
     */
    void publish(const ScanFrame& frame);

    /**
     * @brief Whether the I/O thread is running.
     */
    [[nodiscard]] bool isRunning() const noexcept;

    /**
     * @brief Whether a visualisation client is currently attached.
     *
     * Purely informational. No safety-relevant behaviour may branch on this.
     */
    [[nodiscard]] bool hasClient() const noexcept;

    /**
     * @brief Snapshot of the runtime counters.
     */
    [[nodiscard]] TcpServerStats stats() const noexcept;

    /**
     * @brief Description of the most recent failure.
     * @return Empty string if no failure has been recorded.
     */
    [[nodiscard]] std::string lastError() const;

private:
    /**
     * @brief Opaque implementation holding the descriptors, queue and thread.
     *
     * PIMPL keeps @c <sys/socket.h> and @c <poll.h> out of the public header,
     * so no consumer inherits POSIX macros transitively.
     */
    struct Impl;

    /** @brief Sole owner of the implementation. */
    std::unique_ptr<Impl> impl_;
};

}  // namespace aeb

#endif  // AEB_NETWORK_TCPSERVER_HPP
