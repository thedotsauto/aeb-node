/**
 * @file Application.hpp
 * @brief Composition root: component ownership, wiring and supervision.
 *
 * @ref aeb::app::Application is the only place that knows which concrete
 * components exist and how they are connected. Its single responsibility is
 * lifecycle: create, wire, supervise, tear down in reverse order.
 *
 * @section wiring Wiring
 *
 * @code
 * Lidar --(callback, must not block)--> Perception --> CanBus (brake command)
 * @endcode
 *
 * The lidar callback is the single fan-out point. Perception runs first;
 * if any sector contains an obstacle, a CAN brake command is sent before
 * returning. The safety path never waits on, or depends on, any development
 * infrastructure.
 */

#ifndef AEB_APP_APPLICATION_HPP
#define AEB_APP_APPLICATION_HPP

#include <cstdint>
#include <memory>

#include "app/Options.hpp"
#include "app/SignalWaiter.hpp"
#include "canbus/CanBus.hpp"
#include "lidar/Lidar.hpp"
#include "perception/Perception.hpp"

namespace aeb::app {

/**
 * @brief Owns the node's components and runs them until shutdown.
 *
 * Non-copyable and non-movable: it owns components that own threads.
 */
class Application {
public:
    /**
     * @brief Construct with a resolved configuration. No hardware is touched.
     * @param options Runtime configuration.
     */
    explicit Application(Options options);

    /** @brief Stops and releases every component. */
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;
    Application(Application&&) = delete;
    Application& operator=(Application&&) = delete;

    /**
     * @brief Start every component, supervise, then shut down cleanly.
     *
     * Blocks until @c SIGINT or @c SIGTERM arrives, or until acquisition stops
     * unrecoverably.
     *
     * @return Process exit code: 0 on a clean shutdown, 1 on failure.
     */
    [[nodiscard]] int run();

private:
    /**
     * @brief Open the SocketCAN interface for braking output.
     * @return @c true if the interface is open.
     */
    [[nodiscard]] bool startCanBus();

    /**
     * @brief Create the lidar and begin acquisition.
     * @return @c true if acquisition is running.
     */
    [[nodiscard]] bool startLidar();

    /**
     * @brief Block until shutdown is requested, reporting health periodically.
     * @param signals Signal waiter installed by @ref run.
     * @return Process exit code.
     */
    [[nodiscard]] int supervise(const SignalWaiter& signals);

    /** @brief Print one line of acquisition and streaming health. */
    void reportHealth();

    /** @brief Stop and destroy components in reverse construction order. */
    void shutdown() noexcept;

    /** @brief Immutable runtime configuration. */
    Options options_;

    /** @brief SocketCAN braking output. */
    CanBus can_bus_;

    /** @brief Acquisition driver; null until started. */
    std::unique_ptr<Lidar> lidar_;

    /** @brief Obstacle detection; runs on every acquired frame. */
    Perception perception_;

    /**
     * @brief Read-error count at the previous health report.
     *
     * Lets @ref reportHealth distinguish a fault happening *now* from one that
     * happened once at start-up, instead of repeating a stale message forever.
     */
    std::uint64_t reported_read_errors_{0U};
};

}  // namespace aeb::app

#endif  // AEB_APP_APPLICATION_HPP
