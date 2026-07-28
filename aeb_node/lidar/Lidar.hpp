/**
 * @file Lidar.hpp
 * @brief RAII acquisition driver for the Slamtec RPLidar C1.
 *
 * @ref aeb::Lidar owns everything required to turn a serial port into a stream
 * of @ref aeb::ScanFrame objects: the Slamtec driver instance, the serial
 * channel and the acquisition thread. Its single responsibility is
 * *acquisition* - it does not filter, detect, record or transmit.
 *
 * @section threading Threading model
 *
 * @ref aeb::Lidar::start spawns exactly one thread. That thread blocks inside
 * the Slamtec SDK waiting for a complete revolution, converts the result into a
 * @ref aeb::ScanFrame and invokes the user callback. All other public methods
 * are safe to call concurrently from any thread.
 *
 * The callback is the *only* extension point, and it is deliberately narrow:
 * consumers (network sender, recorder, obstacle detector) subscribe by wrapping
 * this callback, never by inheriting from @ref aeb::Lidar.
 *
 * @section deps Dependency isolation
 *
 * This header does **not** include any Slamtec SDK header. The driver lives
 * behind a PIMPL so that the perception and braking layers can depend on the
 * lidar abstraction without inheriting the SDK's include path, macros or
 * compile-time flags - and so the SDK can be swapped for a replay source in
 * Phase 2 without recompiling those layers.
 */

#ifndef AEB_LIDAR_LIDAR_HPP
#define AEB_LIDAR_LIDAR_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "common/Types.hpp"

namespace aeb {

/**
 * @brief Static configuration for a @ref Lidar instance.
 *
 * Passed by value at construction and never mutated afterwards, so there is no
 * shared mutable configuration state to synchronise.
 */
struct LidarConfig {
    /**
     * @brief Serial device path of the CP2102 USB-UART adapter.
     *
     * Prefer a stable @c /dev/serial/by-id/... path on the vehicle:
     * @c /dev/ttyUSB0 is assigned by enumeration order and will move if another
     * USB serial device is present.
     */
    std::string serial_port{"/dev/ttyUSB0"};

    /** @brief Serial baud rate. The RPLidar C1 requires 460800. */
    std::uint32_t baudrate{460800U};

    /**
     * @brief Maximum time to wait for one complete revolution, milliseconds.
     *
     * At the C1's nominal 10 Hz a revolution takes ~100 ms. 2000 ms tolerates
     * spin-up and transient stalls while still detecting a dead sensor quickly.
     */
    std::uint32_t scan_timeout_ms{2000U};

    /**
     * @brief Upper bound on measurements retrieved per revolution.
     *
     * Sizes the SDK's node buffer, which is allocated once at start and reused,
     * so acquisition performs no steady-state heap allocation.
     */
    std::size_t max_nodes_per_scan{8192U};

    /**
     * @brief Whether the acquisition thread should recover from I/O faults.
     *
     * When @c true, a scan timeout or driver error triggers a full stop,
     * close, reopen and restart cycle rather than terminating acquisition.
     * A vehicle sensor node must survive a transient USB glitch.
     */
    bool auto_reconnect{true};

    /** @brief Delay between reconnection attempts, milliseconds. */
    std::uint32_t reconnect_delay_ms{500U};
};

/**
 * @brief Runtime health counters for a @ref Lidar instance.
 *
 * Returned by value as a consistent-enough snapshot for diagnostics. These are
 * observability counters, not a safety interlock; Phase 3 will derive its own
 * sensor-liveness signal from frame timestamps.
 */
struct LidarStats {
    /** @brief Complete frames delivered to the callback since construction. */
    std::uint64_t frames_delivered{0U};

    /** @brief Scan timeouts or driver read failures observed. */
    std::uint64_t read_errors{0U};

    /** @brief Successful (re)connections to the sensor. */
    std::uint64_t connects{0U};

    /** @brief Frames discarded because they contained no valid points. */
    std::uint64_t empty_frames{0U};

    /** @brief Monotonic timestamp of the most recent delivered frame. */
    TimestampUs last_frame_us{0U};
};

/**
 * @brief Owning, thread-safe driver for the RPLidar C1.
 *
 * Lifetime is strictly RAII: the serial channel and driver are acquired in
 * @ref start and released in @ref stop or the destructor, in reverse order of
 * acquisition, even on the error paths. There is no way to leak the device.
 *
 * The class is non-copyable and non-movable: it owns a running thread that
 * captures @c this, so relocating the object would dangle.
 *
 * @code
 * aeb::LidarConfig cfg;
 * cfg.serial_port = "/dev/ttyUSB0";
 *
 * aeb::Lidar lidar{cfg};
 * if (!lidar.start([&](const aeb::ScanFrame& f) { pipeline.submit(f); })) {
 *     std::cerr << lidar.lastError() << '\n';
 *     return 1;
 * }
 * // ... lidar.stop() is implicit at scope exit ...
 * @endcode
 */
class Lidar {
public:
    /**
     * @brief Sink invoked once per complete revolution.
     *
     * @param frame Freshly acquired frame. The reference is valid only for the
     *              duration of the call; copy or move it to retain the data.
     *
     * @warning Executed on the acquisition thread. It **must not block**. Any
     *          work longer than a few hundred microseconds (socket writes, disk
     *          I/O, detection) belongs behind a queue, otherwise the next
     *          revolution is missed and the sensor stream stutters. This is why
     *          the networking layer must never be called synchronously here.
     * @warning It must not throw; an escaping exception is caught, counted as a
     *          read error and otherwise ignored, because a rendering bug in a
     *          development viewer must never take down sensing.
     */
    using FrameCallback = std::function<void(const ScanFrame&)>;

    /**
     * @brief Construct an idle driver. No hardware is touched.
     * @param config Configuration snapshot, copied into the instance.
     */
    explicit Lidar(LidarConfig config);

    /**
     * @brief Stops acquisition and releases the device.
     *
     * Blocks until the acquisition thread has joined, so the callback is
     * guaranteed not to be running once the destructor returns.
     */
    ~Lidar();

    Lidar(const Lidar&) = delete;
    Lidar& operator=(const Lidar&) = delete;
    Lidar(Lidar&&) = delete;
    Lidar& operator=(Lidar&&) = delete;

    /**
     * @brief Open the device, spin up the motor and begin acquiring.
     *
     * Connection and the first @c startScan are performed synchronously so the
     * caller learns immediately whether the sensor is present. The acquisition
     * thread is only spawned once that succeeds.
     *
     * @param callback Frame sink. Must be non-empty.
     * @return @c true if acquisition is running; @c false on failure, in which
     *         case all partially acquired resources have been released and
     *         @ref lastError describes the cause.
     *
     * @note Idempotent-safe: calling @ref start on a running instance returns
     *       @c false and leaves the existing session untouched.
     */
    [[nodiscard]] bool start(FrameCallback callback);

    /**
     * @brief Stop acquisition, join the thread and release the device.
     *
     * Safe to call when not running, and safe to call more than once. Blocks
     * until the acquisition thread has terminated.
     */
    void stop() noexcept;

    /**
     * @brief Whether the acquisition thread is currently active.
     */
    [[nodiscard]] bool isRunning() const noexcept;

    /**
     * @brief Snapshot of the health counters.
     */
    [[nodiscard]] LidarStats stats() const noexcept;

    /**
     * @brief Description of the most recent failure.
     * @return Empty string if no failure has been recorded.
     */
    [[nodiscard]] std::string lastError() const;

    /**
     * @brief Monotonic clock reading in microseconds.
     *
     * Exposed so that recording, replay and the perception layer stamp and
     * compare events on exactly the same time base as acquisition.
     *
     * @return Microseconds since an unspecified but fixed epoch. Never
     *         decreases and is immune to wall-clock adjustments.
     */
    [[nodiscard]] static TimestampUs nowMicros() noexcept;

private:
    /**
     * @brief Opaque implementation holding the Slamtec driver, the serial
     *        channel, the thread and the synchronisation primitives.
     */
    struct Impl;

    /** @brief Sole owner of the implementation. */
    std::unique_ptr<Impl> impl_;
};

}  // namespace aeb

#endif  // AEB_LIDAR_LIDAR_HPP
