/**
 * @file CommandLine.hpp
 * @brief Translation of command line arguments into @ref aeb::app::Options.
 *
 * Single responsibility: turning strings into configuration. It performs no
 * I/O other than writing usage text when asked, touches no global state, and is
 * a pure function of its inputs - so it is directly unit testable by passing a
 * synthetic @c argv.
 */

#ifndef AEB_APP_COMMANDLINE_HPP
#define AEB_APP_COMMANDLINE_HPP

#include <optional>
#include <string>

#include "app/Options.hpp"

namespace aeb::app {

/**
 * @brief Command line parser for the AEB node.
 */
class CommandLine {
public:
    /**
     * @brief Parse command line arguments.
     *
     * @param argc        Argument count as received by @c main.
     * @param argv        Argument vector as received by @c main.
     * @param[out] error  Receives a description if parsing failed. Left empty
     *                    when help was requested, which distinguishes "user
     *                    asked for help" from "user made a mistake".
     * @return The parsed options, or @c std::nullopt if parsing failed or help
     *         was printed.
     */
    [[nodiscard]] static std::optional<Options> parse(int argc, char** argv, std::string& error);

    /**
     * @brief Write usage text to standard output.
     * @param program Program name as invoked.
     */
    static void printUsage(const char* program);
};

}  // namespace aeb::app

#endif  // AEB_APP_COMMANDLINE_HPP
