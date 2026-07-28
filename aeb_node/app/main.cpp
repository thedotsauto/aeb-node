/**
 * @file main.cpp
 * @brief Process entry point for the AEB sensing node.
 *
 * Deliberately trivial. Argument handling lives in @ref aeb::app::CommandLine
 * and the entire lifecycle lives in @ref aeb::app::Application, so this file
 * contains no application logic and never needs to change when components are
 * added.
 */

#include <iostream>
#include <string>

#include "app/Application.hpp"
#include "app/CommandLine.hpp"
#include "app/Options.hpp"

namespace {

/** @brief Exit code reported when the command line could not be parsed. */
constexpr int kExitUsage = 2;

/** @brief Exit code reported when help was requested. */
constexpr int kExitHelp = 0;

}  // namespace

/**
 * @brief Parse arguments and run the node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Process exit code: 0 on success, 1 on failure, 2 on usage error.
 */
int main(int argc, char** argv)
{
    std::string error;
    const auto options = aeb::app::CommandLine::parse(argc, argv, error);

    if (!options.has_value()) {
        // An empty error message means help was printed, not that parsing failed.
        if (error.empty()) {
            return kExitHelp;
        }
        std::cerr << "error: " << error << "\nTry --help\n";
        return kExitUsage;
    }

    aeb::app::Application application{*options};
    return application.run();
}
