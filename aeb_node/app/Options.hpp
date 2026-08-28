/**
 * @file Options.hpp
 * @brief Fully resolved runtime configuration for the AEB node.
 *
 * A plain value type shared by the parser and the application. Keeping it
 * separate from both means the application can be constructed directly from
 * code - in a test, or from a future configuration file loader - without going
 * through @c argv.
 */

#ifndef AEB_APP_OPTIONS_HPP
#define AEB_APP_OPTIONS_HPP

#include "canbus/CanBus.hpp"
#include "lidar/Lidar.hpp"
#include "network/TcpServer.hpp"
#include "perception/PerceptionConfig.hpp"

namespace aeb::app {

/**
 * @brief Everything the node needs to know at start-up.
 */
struct Options {
    /** @brief Lidar acquisition settings. */
    LidarConfig lidar{};

    /** @brief SocketCAN braking output settings. */
    CanBusConfig canbus{};

    /** @brief When true, the CAN bus is not opened and no brake commands are sent. */
    bool no_can{false};

    /** @brief TCP visualisation server settings. */
    TcpServerConfig tcp_server{};

    /** @brief Obstacle detection thresholds. */
    PerceptionConfig perception{};

    /** @brief Interval between health reports in seconds; 0 disables them. */
    unsigned health_interval_s{5U};
};

}  // namespace aeb::app

#endif  // AEB_APP_OPTIONS_HPP
