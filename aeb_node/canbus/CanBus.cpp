/**
 * @file CanBus.cpp
 * @brief Linux SocketCAN implementation of @ref aeb::CanBus.
 */

#include "canbus/CanBus.hpp"

#include <cerrno>
#include <cstring>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace aeb {
namespace {

/**
 * @brief Format an error string from @c errno.
 * @param context Brief description of the failing operation.
 * @return Human-readable string.
 */
std::string describeErrno(const std::string& context)
{
    return context + ": " + std::strerror(errno);
}

}  // namespace

CanBus::CanBus(CanBusConfig config) : config_{std::move(config)} {}

CanBus::~CanBus()
{
    close();
}

bool CanBus::open()
{
    if (fd_ >= 0) {
        last_error_ = "already open";
        return false;
    }

    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        last_error_ = describeErrno("socket(PF_CAN)");
        fd_ = -1;
        return false;
    }

    struct ifreq ifr{};
    if (config_.interface.size() >= IFNAMSIZ) {
        last_error_ = "interface name too long: " + config_.interface;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1U);

    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        last_error_ = describeErrno("ioctl(SIOCGIFINDEX) on " + config_.interface);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        last_error_ = describeErrno("bind on " + config_.interface);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    last_error_.clear();
    return true;
}

void CanBus::close() noexcept
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool CanBus::send(const CanMessage& msg) noexcept
{
    if (fd_ < 0) {
        return false;
    }

    struct can_frame frame{};
    frame.can_id  = msg.id;
    frame.can_dlc = msg.dlc;
    const std::uint8_t bytes = (msg.dlc <= kCanMaxDlc) ? msg.dlc : kCanMaxDlc;
    std::memcpy(frame.data, msg.data, bytes);

    const ssize_t sent = ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
    return sent == static_cast<ssize_t>(sizeof(frame));
}

bool CanBus::isOpen() const noexcept
{
    return fd_ >= 0;
}

const std::string& CanBus::lastError() const noexcept
{
    return last_error_;
}

}  // namespace aeb
