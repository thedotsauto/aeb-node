/**
 * @file ListenSocket.cpp
 * @brief Implementation of listening socket construction.
 */

#include "network/ListenSocket.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace aeb::net {
namespace {

/**
 * @brief Backlog for @c listen.
 *
 * The server serves one visualisation client at a time, so a depth of one is
 * sufficient; a second pending connection simply waits or is refused.
 */
constexpr int kListenBacklog = 1;

}  // namespace

std::string describeErrno(const std::string& context, int err)
{
    return context + ": " + std::strerror(err);
}

FileDescriptor makeListenSocket(const TcpServerConfig& config, std::string& error)
{
    FileDescriptor fd{::socket(AF_INET, SOCK_STREAM, 0)};
    if (!fd.valid()) {
        error = describeErrno("socket()", errno);
        return FileDescriptor{};
    }

    // Allows an immediate restart of the node while the previous socket is
    // still in TIME_WAIT, which matters during development iteration.
    const int enable = 1;
    if (::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) != 0) {
        error = describeErrno("setsockopt(SO_REUSEADDR)", errno);
        return FileDescriptor{};
    }

    if (!setNonBlocking(fd.get())) {
        error = describeErrno("fcntl(O_NONBLOCK) on listen socket", errno);
        return FileDescriptor{};
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = ::htons(config.port);
    if (::inet_pton(AF_INET, config.bind_address.c_str(), &address.sin_addr) != 1) {
        error = "invalid bind address: " + config.bind_address;
        return FileDescriptor{};
    }

    if (::bind(fd.get(), reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        error = describeErrno("bind() to " + config.bind_address + ":" +
                                  std::to_string(config.port),
                              errno);
        return FileDescriptor{};
    }

    if (::listen(fd.get(), kListenBacklog) != 0) {
        error = describeErrno("listen()", errno);
        return FileDescriptor{};
    }

    return fd;
}

}  // namespace aeb::net
