/**
 * @file CanBus.hpp
 * @brief RAII wrapper around a Linux SocketCAN raw socket.
 *
 * @ref aeb::CanBus opens a SocketCAN interface by name (e.g. @c "can0"), sends
 * frames from the calling thread, and closes the socket on destruction.
 *
 * Linux kernel headers (@c <linux/can.h> etc.) are confined to @c CanBus.cpp
 * so that this header is host-portable for unit testing on a development Mac.
 */

#ifndef AEB_CANBUS_CANBUS_HPP
#define AEB_CANBUS_CANBUS_HPP

#include <string>

#include "canbus/CanMessage.hpp"

namespace aeb {

/**
 * @brief Configuration for @ref CanBus.
 */
struct CanBusConfig {
    /** @brief SocketCAN interface name (e.g. @c "can0"). */
    std::string interface{"can0"};

    /**
     * @brief Selects the UNO Q transport instead of SocketCAN.
     *
     * When @c true, @ref CanBus::open and @ref CanBus::send connect to the
     * App Lab can-bridge application's TCP server (127.0.0.1:39001, see
     * @ref fifo_path) instead of a SocketCAN interface. The generated
     * @ref CanMessage is unchanged; only the transport differs. Default is
     * @c false so existing SocketCAN behaviour is untouched.
     */
    bool use_fifo_transport{false};

    /**
     * @brief UNO Q transport endpoint, shown in start-up logging when
     * @ref use_fifo_transport is @c true.
     *
     * @note The actual connection always targets 127.0.0.1:39001 (see
     *       CanBus.cpp); this field is informational only. Its name and
     *       type are kept from the earlier FIFO-based transport so that
     *       existing call sites which set/read it by name (CommandLine.cpp,
     *       Application.cpp) continue to compile unchanged.
     */
    std::string fifo_path{"127.0.0.1:39001"};
};

/**
 * @brief Non-blocking SocketCAN raw socket, bound to a single interface.
 *
 * Non-copyable and non-movable: it owns the socket file descriptor.
 *
 * @note @ref send is safe to call from the lidar acquisition thread; it
 *       uses @c MSG_DONTWAIT and never blocks.
 */
class CanBus {
public:
    /**
     * @brief Construct with configuration.  No socket is created yet.
     * @param config Interface name; copied into the instance.
     */
    explicit CanBus(CanBusConfig config);

    /**
     * @brief Close the socket if open.
     */
    ~CanBus();

    CanBus(const CanBus&)            = delete;
    CanBus& operator=(const CanBus&) = delete;
    CanBus(CanBus&&)                 = delete;
    CanBus& operator=(CanBus&&)      = delete;

    /**
     * @brief Open the SocketCAN socket and bind to the configured interface.
     * @return @c true on success; @c false on failure, @ref lastError describes the cause.
     */
    [[nodiscard]] bool open();

    /**
     * @brief Close the socket.  Safe to call when not open.
     */
    void close() noexcept;

    /**
     * @brief Send a CAN frame.
     *
     * Non-blocking: uses @c MSG_DONTWAIT.  If the kernel transmit buffer is
     * full, the frame is silently dropped (AEB must never stall the lidar
     * acquisition thread).
     *
     * @param msg Frame to transmit.
     * @return @c true if the frame was accepted by the kernel.
     */
    [[nodiscard]] bool send(const CanMessage& msg) noexcept;

    /** @brief @c true if the socket is open. */
    [[nodiscard]] bool isOpen() const noexcept;

    /** @brief Human-readable description of the last failure. */
    [[nodiscard]] const std::string& lastError() const noexcept;

private:
    CanBusConfig config_;
    int          fd_{-1};
    std::string  last_error_;
};

}  // namespace aeb

#endif  // AEB_CANBUS_CANBUS_HPP
