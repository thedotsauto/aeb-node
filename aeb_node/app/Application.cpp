/**
 * @file Application.cpp
 * @brief Implementation of component wiring and supervision.
 */

#include "app/Application.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <utility>

namespace aeb::app {
namespace {

/** @brief Exit code reported on a clean shutdown. */
constexpr int kExitSuccess = 0;

/** @brief Exit code reported on a start-up or runtime failure. */
constexpr int kExitFailure = 1;

/** @brief Supervision tick used when health reporting is disabled. */
constexpr std::chrono::milliseconds kIdleTick{1000};

/** @brief CAN arbitration ID for the braking command frame. */
constexpr std::uint32_t kBrakeCanId = 0x080U;

/** @brief Payload: disengage the brake (no obstacle detected). */
constexpr std::uint8_t kBrakeDisengage = 0x01U;

/** @brief Payload: engage the brake (obstacle detected in at least one sector). */
constexpr std::uint8_t kBrakeEngage = 0x02U;

}  // namespace

Application::Application(Options options)
    : options_{std::move(options)}
    , can_bus_{options_.canbus}
    , tcp_server_{options_.tcp_server}
    , perception_{options_.perception}
{}

Application::~Application()
{
    shutdown();
}

int Application::run()
{
    // Installed before any component, so every worker thread inherits the
    // blocked signal mask and shutdown is delivered only here.
    const SignalWaiter signals;
    if (!signals.valid()) {
        std::cerr << "fatal: unable to install signal mask\n";
        return kExitFailure;
    }

    if (!startCanBus()) {
        return kExitFailure;
    }
    if (!startTcpServer()) {
        shutdown();
        return kExitFailure;
    }
    if (!startLidar()) {
        shutdown();
        return kExitFailure;
    }

    const int exit_code = supervise(signals);
    shutdown();

    std::cout << "aeb_node: stopped\n";
    return exit_code;
}

bool Application::startCanBus()
{
    if (options_.no_can) {
        std::cout << "aeb_node: CAN disabled (--no-can), running in viewer-only mode\n";
        return true;
    }

    if (!can_bus_.open()) {
        std::cerr << "fatal: CanBus::open failed: " << can_bus_.lastError() << '\n';
        return false;
    }

    if (options_.canbus.use_fifo_transport) {
        std::cout << "aeb_node: UNO Q CAN transport open (" << options_.canbus.fifo_path << ")\n";
    } else {
        std::cout << "aeb_node: CAN interface " << options_.canbus.interface << " open\n";
    }
    return true;
}

bool Application::startTcpServer()
{
    if (!tcp_server_.start()) {
        std::cerr << "fatal: TcpServer::start failed: " << tcp_server_.lastError() << '\n';
        return false;
    }

    std::cout << "aeb_node: TCP visualisation server listening on port "
              << options_.tcp_server.port << '\n';
    return true;
}

bool Application::startLidar()
{
    lidar_ = std::make_unique<Lidar>(options_.lidar);

    // The single fan-out point, executed on the acquisition thread.
    // Perception runs first; then a non-blocking CAN frame is sent.
    // Nothing here may block: CanBus::send uses MSG_DONTWAIT.
    CanBus* const can = &can_bus_;
    TcpServer* const tcp = &tcp_server_;
    const bool no_can = options_.no_can;
    const bool started = lidar_->start([this, can, tcp, no_can](const ScanFrame& frame) {
        tcp->publish(frame);

        const DetectionResult result = perception_.process(frame);

        const bool obstacle =
            result.left.obstacle_detected        ||
            result.left_center.obstacle_detected  ||
            result.center.obstacle_detected       ||
            result.right_center.obstacle_detected ||
            result.right.obstacle_detected;

        if (!no_can) {
            CanMessage msg;
            msg.id      = kBrakeCanId;
            msg.dlc     = 1U;
            msg.data[0] = obstacle ? kBrakeEngage : kBrakeDisengage;
            can->send(msg);
        }

        std::cout << (obstacle ? "BRAKE  " : "clear  ")
                  << "pts=" << result.total_points
                  << " filt=" << result.quality_filtered_points
                  << '\n';
    });

    if (!started) {
        std::cerr << "fatal: Lidar::start failed: " << lidar_->lastError() << '\n';
        lidar_.reset();
        return false;
    }

    std::cout << "aeb_node: acquiring from " << options_.lidar.serial_port << " @ "
              << options_.lidar.baudrate << " baud\n";
    return true;
}

int Application::supervise(const SignalWaiter& signals)
{
    const auto tick = options_.health_interval_s > 0U
                          ? std::chrono::milliseconds{options_.health_interval_s * 1000U}
                          : kIdleTick;

    for (;;) {
        const int signal_number = signals.waitFor(tick);
        if (signal_number != 0) {
            std::cout << "aeb_node: received signal " << signal_number << ", shutting down\n";
            return kExitSuccess;
        }

        // Acquisition stops by itself only when auto-reconnect is disabled and
        // the device failed. Treat it as fatal rather than idling blind.
        if (!lidar_->isRunning()) {
            std::cerr << "fatal: acquisition stopped: " << lidar_->lastError() << '\n';
            return kExitFailure;
        }

        if (options_.health_interval_s > 0U) {
            reportHealth();
        }
    }
}

void Application::reportHealth()
{
    const LidarStats lidar_stats = lidar_->stats();

    const TcpServerStats tcp_stats = tcp_server_.stats();

    std::cout << "health: frames=" << lidar_stats.frames_delivered
              << " errors=" << lidar_stats.read_errors
              << " empty=" << lidar_stats.empty_frames
              << " connects=" << lidar_stats.connects
              << " can=" << (options_.no_can ? "disabled" : (can_bus_.isOpen() ? "open" : "closed"))
              << " tcp_client=" << (tcp_stats.client_connected ? "connected" : "waiting");

    // Report the driver's own description only for errors that occurred since
    // the previous tick. Repeating a start-up timeout for the life of the
    // process would make a healthy node look permanently faulty.
    const std::uint64_t new_errors = lidar_stats.read_errors - reported_read_errors_;
    reported_read_errors_ = lidar_stats.read_errors;
    if (new_errors != 0U) {
        const std::string error = lidar_->lastError();
        if (!error.empty()) {
            std::cout << "\n  " << new_errors << " new error(s), last: " << error;
        }
    }
    std::cout << std::endl;
}

void Application::shutdown() noexcept
{
    // Lidar first: stop producing frames before releasing any consumer.
    if (lidar_) {
        lidar_->stop();
    }
    lidar_.reset();
    tcp_server_.stop();
    can_bus_.close();
}

}  // namespace aeb::app
