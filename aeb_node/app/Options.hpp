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

#include "lidar/Lidar.hpp"
#include "network/TcpServer.hpp"

namespace aeb::app {

/**
 * @brief Everything the node needs to know at start-up.
 */
struct Options {
    /** @brief Lidar acquisition settings. */
    LidarConfig lidar{};

    /** @brief Development streaming server settings. */
    TcpServerConfig network{};

    /**
     * @brief Whether to build and run the networking layer at all.
     *
     * When @c false no socket is ever created. This is the intended vehicle
     * configuration, and it continuously proves that acquisition - and, from
     * Phase 3, perception and braking - have no dependency on the network.
     */
    bool enable_network{true};

    /** @brief Interval between health reports in seconds; 0 disables them. */
    unsigned health_interval_s{5U};
};

}  // namespace aeb::app

#endif  // AEB_APP_OPTIONS_HPP
