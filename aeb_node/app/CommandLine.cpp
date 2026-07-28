/**
 * @file CommandLine.cpp
 * @brief Implementation of command line parsing.
 */

#include "app/CommandLine.hpp"

#include <cerrno>
#include <cstdlib>
#include <iostream>

namespace aeb::app {
namespace {

/** @brief Largest accepted value for a numeric option. */
constexpr unsigned long kMaxNumericValue = 0xFFFFFFFFUL;

/** @brief Highest valid TCP port number. */
constexpr std::uint32_t kMaxPort = 65535U;

/**
 * @brief Parse a non-negative decimal integer, rejecting trailing junk.
 *
 * Strict on purpose: silently accepting "8OOO" as 8 would be a configuration
 * error that only surfaces as mysterious runtime behaviour.
 *
 * @param text        Input text.
 * @param[out] out    Receives the value on success.
 * @param[out] error  Receives a description on failure.
 * @param option      Option name, used in the error message.
 * @return @c true on success.
 */
[[nodiscard]] bool parseUnsigned(const std::string& text, std::uint32_t& out, std::string& error,
                                 const std::string& option)
{
    if (text.empty()) {
        error = option + " requires a number";
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0' || parsed > kMaxNumericValue) {
        error = option + ": '" + text + "' is not a valid number";
        return false;
    }
    out = static_cast<std::uint32_t>(parsed);
    return true;
}

/**
 * @brief Apply an option that takes no value.
 * @param arg          Option token.
 * @param[in,out] out  Options being built.
 * @return @c true if @p arg was recognised and applied.
 */
[[nodiscard]] bool applyFlag(const std::string& arg, Options& out)
{
    if (arg == "--no-network") {
        out.enable_network = false;
        return true;
    }
    if (arg == "--no-reconnect") {
        out.lidar.auto_reconnect = false;
        return true;
    }
    return false;
}

/**
 * @brief Apply an option that takes a value.
 * @param arg          Option token.
 * @param value        Its argument.
 * @param[in,out] out  Options being built.
 * @param[out] error   Receives a description on failure.
 * @return @c true if @p arg was recognised and applied successfully.
 */
[[nodiscard]] bool applyValued(const std::string& arg, const std::string& value, Options& out,
                               std::string& error)
{
    if (arg == "--device") {
        out.lidar.serial_port = value;
        return true;
    }
    if (arg == "--baud") {
        return parseUnsigned(value, out.lidar.baudrate, error, arg);
    }
    if (arg == "--scan-timeout-ms") {
        return parseUnsigned(value, out.lidar.scan_timeout_ms, error, arg);
    }
    if (arg == "--bind") {
        out.network.bind_address = value;
        return true;
    }
    if (arg == "--port") {
        std::uint32_t port = 0U;
        if (!parseUnsigned(value, port, error, arg)) {
            return false;
        }
        if (port == 0U || port > kMaxPort) {
            error = "--port must be in the range 1.." + std::to_string(kMaxPort);
            return false;
        }
        out.network.port = static_cast<std::uint16_t>(port);
        return true;
    }
    if (arg == "--queue") {
        std::uint32_t depth = 0U;
        if (!parseUnsigned(value, depth, error, arg)) {
            return false;
        }
        if (depth == 0U) {
            error = "--queue must be a positive integer";
            return false;
        }
        out.network.queue_capacity = depth;
        return true;
    }
    if (arg == "--health-interval") {
        std::uint32_t seconds = 0U;
        if (!parseUnsigned(value, seconds, error, arg)) {
            return false;
        }
        out.health_interval_s = seconds;
        return true;
    }

    error = "unknown option " + arg;
    return false;
}

}  // namespace

std::optional<Options> CommandLine::parse(int argc, char** argv, std::string& error)
{
    Options options{};
    error.clear();

    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return std::nullopt;  // error stays empty: this is not a failure.
        }
        if (applyFlag(arg, options)) {
            continue;
        }
        if (i + 1 >= argc) {
            error = "missing value for option " + arg;
            return std::nullopt;
        }
        if (!applyValued(arg, std::string{argv[++i]}, options, error)) {
            return std::nullopt;
        }
    }

    return options;
}

void CommandLine::printUsage(const char* program)
{
    std::cout << "AEB sensing node\n\n"
              << "Usage: " << program << " [options]\n\n"
              << "Lidar:\n"
              << "  --device PATH           Serial device (default /dev/ttyUSB0)\n"
              << "  --baud RATE             Baud rate (default 460800)\n"
              << "  --scan-timeout-ms MS    Per-revolution timeout (default 2000)\n"
              << "  --no-reconnect          Exit acquisition on device error\n\n"
              << "Development network stream:\n"
              << "  --bind ADDRESS          Listen address (default 0.0.0.0)\n"
              << "  --port PORT             Listen port (default 7000)\n"
              << "  --queue DEPTH           Outbound frame queue depth (default 16)\n"
              << "  --no-network            Disable streaming entirely\n\n"
              << "Diagnostics:\n"
              << "  --health-interval SEC   Health report period, 0 disables (default 5)\n"
              << "  -h, --help              Show this message\n";
}

}  // namespace aeb::app
