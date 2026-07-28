/**
 * @file ListenSocket.hpp
 * @brief Construction of the server's listening socket.
 *
 * Socket creation is a distinct concern from running the event loop, and it is
 * the part most likely to fail in the field (port in use, bad bind address,
 * insufficient privilege). Isolating it keeps every failure path in one small,
 * directly testable place.
 *
 * Internal header of the networking layer.
 */

#ifndef AEB_NETWORK_LISTENSOCKET_HPP
#define AEB_NETWORK_LISTENSOCKET_HPP

#include <string>

#include "network/FileDescriptor.hpp"
#include "network/TcpServer.hpp"

namespace aeb::net {

/**
 * @brief Create a bound, listening, non-blocking TCP socket.
 *
 * On any failure the partially configured descriptor is closed before
 * returning, so the caller can never observe a half-open socket.
 *
 * @param config      Bind address and port to use.
 * @param[out] error  Receives a diagnostic message on failure.
 * @return An owning descriptor, or an empty one on failure.
 */
[[nodiscard]] FileDescriptor makeListenSocket(const TcpServerConfig& config, std::string& error);

/**
 * @brief Format @c errno as "context: message".
 *
 * Shared by the networking layer so that every diagnostic has the same shape.
 *
 * @param context Description of the failed operation.
 * @param err     Captured @c errno value.
 * @return Diagnostic string.
 */
[[nodiscard]] std::string describeErrno(const std::string& context, int err);

}  // namespace aeb::net

#endif  // AEB_NETWORK_LISTENSOCKET_HPP
