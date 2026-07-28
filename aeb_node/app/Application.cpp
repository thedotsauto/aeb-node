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

}  // namespace

Application::Application(Options options) : options_{std::move(options)} {}

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

    if (!startNetwork()) {
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

bool Application::startNetwork()
{
    if (!options_.enable_network) {
        std::cout << "aeb_node: networking disabled\n";
        return true;
    }

    server_ = std::make_unique<TcpServer>(options_.network);
    if (!server_->start()) {
        std::cerr << "fatal: TcpServer::start failed: " << server_->lastError() << '\n';
        server_.reset();
        return false;
    }

    std::cout << "aeb_node: streaming on " << options_.network.bind_address << ':'
              << options_.network.port << '\n';
    return true;
}

bool Application::startLidar()
{
    lidar_ = std::make_unique<Lidar>(options_.lidar);

    // The single fan-out point, executed on the acquisition thread. Every
    // consumer attached here must be non-blocking; TcpServer::publish
    // guarantees that by contract. A future obstacle detector is invoked
    // *before* the publish, so the safety path never waits on, or depends on,
    // development infrastructure - including when `server` is null.
    TcpServer* const server = server_.get();
    const bool started = lidar_->start([server](const ScanFrame& frame) {
        if (server != nullptr) {
            server->publish(frame);
        }
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

    std::cout << "health: frames=" << lidar_stats.frames_delivered
              << " errors=" << lidar_stats.read_errors
              << " empty=" << lidar_stats.empty_frames
              << " connects=" << lidar_stats.connects;

    if (server_) {
        const TcpServerStats net_stats = server_->stats();
        std::cout << " | client=" << (net_stats.client_connected ? "yes" : "no")
                  << " sent=" << net_stats.frames_sent
                  << " dropped=" << net_stats.frames_dropped
                  << " bytes=" << net_stats.bytes_sent;
    }

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
    // Reverse construction order: stop producing before destroying consumers,
    // so no frame is ever published into a half-destroyed server.
    if (lidar_) {
        lidar_->stop();
    }
    server_.reset();
    lidar_.reset();
}

}  // namespace aeb::app
