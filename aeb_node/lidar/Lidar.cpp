/**
 * @file Lidar.cpp
 * @brief Acquisition thread, recovery policy and health accounting.
 *
 * Hardware access is delegated entirely to @ref aeb::hw::RpLidarDevice. What
 * remains here is the supervision policy: run a thread, obtain revolutions,
 * stamp them, hand them to the consumer, and recover from device faults - all
 * expressible without a single line of vendor code.
 */

#include "lidar/Lidar.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "lidar/RpLidarDevice.hpp"

namespace aeb {

/**
 * @brief Private state of a @ref Lidar instance.
 *
 * Locking discipline:
 *
 *  - @c device_mutex guards @ref device and is never held while the user
 *    callback runs, so a slow consumer cannot delay @ref Lidar::stop;
 *  - @c state_mutex guards @ref stats and @ref last_error and is never held
 *    across a blocking call.
 *
 * The two are never acquired together, so no lock ordering rule exists and
 * deadlock is structurally impossible.
 */
struct Lidar::Impl {
    /**
     * @brief Construct with a validated configuration.
     * @param cfg Configuration snapshot.
     */
    explicit Impl(LidarConfig cfg) : config{std::move(cfg)}, device{config}
    {
        frame.points.reserve(config.max_nodes_per_scan);
    }

    /** @brief Immutable configuration captured at construction. */
    LidarConfig config;

    /** @brief The hardware, owned exclusively by this instance. */
    hw::RpLidarDevice device;

    /** @brief Frame sink, assigned by @ref Lidar::start before the thread runs. */
    FrameCallback callback;

    /** @brief Acquisition thread; joinable exactly while running. */
    std::thread worker;

    /** @brief Set to request acquisition shutdown. */
    std::atomic<bool> stop_requested{false};

    /** @brief True between a successful start and the thread exiting. */
    std::atomic<bool> running{false};

    /** @brief Guards @ref device. */
    std::mutex device_mutex;

    /** @brief Guards @ref stats and @ref last_error. */
    mutable std::mutex state_mutex;

    /** @brief Health counters. */
    LidarStats stats;

    /** @brief Description of the most recent failure. */
    std::string last_error;

    /** @brief Signals the reconnect back-off so shutdown is never delayed. */
    std::condition_variable shutdown_cv;

    /** @brief Companion mutex for @ref shutdown_cv. */
    std::mutex shutdown_mutex;

    /** @brief Reusable output frame, keeping acquisition allocation free. */
    ScanFrame frame;

    /** @brief Next sequence number to assign. */
    SequenceNumber next_sequence{0U};

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
     * @brief Open the device and note the successful connection.
     * @return @c true if the device is scanning.
     */
    [[nodiscard]] bool openDevice()
    {
        std::string error;
        bool opened = false;
        {
            const std::lock_guard<std::mutex> lock(device_mutex);
            opened = device.open(error);
        }
        if (!opened) {
            setError(std::move(error));
            return false;
        }
        const std::lock_guard<std::mutex> lock(state_mutex);
        ++stats.connects;
        last_error.clear();
        return true;
    }

    /**
     * @brief Sleep for the reconnect back-off, waking early on shutdown.
     */
    void backOff()
    {
        std::unique_lock<std::mutex> lock(shutdown_mutex);
        (void)shutdown_cv.wait_for(lock, std::chrono::milliseconds(config.reconnect_delay_ms),
                                   [this] {
                                       return stop_requested.load(std::memory_order_acquire);
                                   });
    }

    /**
     * @brief Deliver a completed frame to the consumer.
     *
     * The callback runs outside every lock held by this class. Any exception it
     * throws is absorbed: a fault in a viewer or recorder must never stop
     * sensing on a vehicle.
     */
    void deliverFrame()
    {
        frame.sequence = next_sequence++;
        frame.timestamp_us = Lidar::nowMicros();

        try {
            callback(frame);
        } catch (...) {
            const std::lock_guard<std::mutex> lock(state_mutex);
            ++stats.read_errors;
        }

        const std::lock_guard<std::mutex> lock(state_mutex);
        ++stats.frames_delivered;
        stats.last_frame_us = frame.timestamp_us;
    }

    /**
     * @brief Acquire one revolution and hand it to the consumer.
     * @return @c true if the cycle completed without a device error; @c false
     *         if the caller should tear the connection down and retry.
     */
    [[nodiscard]] bool acquireOnce()
    {
        std::string error;
        bool grabbed = false;
        {
            const std::lock_guard<std::mutex> lock(device_mutex);
            if (!device.isOpen()) {
                return false;
            }
            grabbed = device.grabScan(frame.points, error);
        }

        if (!grabbed) {
            setError(std::move(error));
            return false;
        }
        if (frame.points.empty()) {
            const std::lock_guard<std::mutex> lock(state_mutex);
            ++stats.empty_frames;
            return true;
        }

        deliverFrame();
        return true;
    }

    /**
     * @brief Acquisition thread body.
     *
     * Runs until shutdown is requested, or until a device fault occurs with
     * @c auto_reconnect disabled.
     */
    void run()
    {
        while (!stop_requested.load(std::memory_order_acquire)) {
            if (acquireOnce()) {
                continue;
            }

            {
                const std::lock_guard<std::mutex> lock(state_mutex);
                ++stats.read_errors;
            }
            {
                const std::lock_guard<std::mutex> lock(device_mutex);
                device.close();
            }

            if (!config.auto_reconnect || stop_requested.load(std::memory_order_acquire)) {
                break;
            }
            backOff();
            if (!openDevice()) {
                backOff();  // Sensor still absent; retry on the next iteration.
            }
        }

        {
            const std::lock_guard<std::mutex> lock(device_mutex);
            device.close();
        }
        running.store(false, std::memory_order_release);
    }
};

Lidar::Lidar(LidarConfig config) : impl_{std::make_unique<Impl>(std::move(config))} {}

Lidar::~Lidar()
{
    stop();
}

bool Lidar::start(FrameCallback callback)
{
    if (!callback) {
        impl_->setError("start() requires a non-empty callback");
        return false;
    }
    if (impl_->running.load(std::memory_order_acquire)) {
        impl_->setError("start() called while already running");
        return false;
    }

    impl_->stop_requested.store(false, std::memory_order_release);
    impl_->callback = std::move(callback);

    // Connect synchronously so a missing sensor is reported to the caller
    // immediately rather than failing silently inside the thread.
    if (!impl_->openDevice()) {
        impl_->callback = nullptr;
        return false;
    }

    impl_->running.store(true, std::memory_order_release);
    try {
        impl_->worker = std::thread([this] { impl_->run(); });
    } catch (const std::system_error& e) {
        impl_->running.store(false, std::memory_order_release);
        {
            const std::lock_guard<std::mutex> lock(impl_->device_mutex);
            impl_->device.close();
        }
        impl_->callback = nullptr;
        impl_->setError(std::string{"failed to spawn acquisition thread: "} + e.what());
        return false;
    }
    return true;
}

void Lidar::stop() noexcept
{
    impl_->stop_requested.store(true, std::memory_order_release);
    impl_->shutdown_cv.notify_all();

    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }

    {
        const std::lock_guard<std::mutex> lock(impl_->device_mutex);
        impl_->device.close();
    }
    impl_->running.store(false, std::memory_order_release);
    impl_->callback = nullptr;
}

bool Lidar::isRunning() const noexcept
{
    return impl_->running.load(std::memory_order_acquire);
}

LidarStats Lidar::stats() const noexcept
{
    const std::lock_guard<std::mutex> lock(impl_->state_mutex);
    return impl_->stats;
}

std::string Lidar::lastError() const
{
    const std::lock_guard<std::mutex> lock(impl_->state_mutex);
    return impl_->last_error;
}

TimestampUs Lidar::nowMicros() noexcept
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<TimestampUs>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

}  // namespace aeb
