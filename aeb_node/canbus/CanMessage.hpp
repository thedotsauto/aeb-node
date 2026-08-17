/**
 * @file CanMessage.hpp
 * @brief Portable representation of a single CAN bus frame.
 */

#ifndef AEB_CANBUS_CANMESSAGE_HPP
#define AEB_CANBUS_CANMESSAGE_HPP

#include <cstdint>

namespace aeb {

/** @brief Maximum number of data bytes in a classical CAN frame. */
inline constexpr std::uint8_t kCanMaxDlc = 8U;

/**
 * @brief A single classical CAN frame (11-bit or 29-bit ID, up to 8 bytes).
 *
 * Layout-compatible with @c struct can_frame from @c <linux/can.h>; the
 * actual kernel struct is only referenced inside @ref CanBus.cpp so that
 * these headers remain host-portable.
 */
struct CanMessage {
    /** @brief CAN arbitration ID (11-bit standard or 29-bit extended). */
    std::uint32_t id{0U};

    /** @brief Number of valid bytes in @ref data (0-8). */
    std::uint8_t dlc{0U};

    /** @brief Payload bytes; only the first @ref dlc bytes are meaningful. */
    std::uint8_t data[kCanMaxDlc]{};
};

}  // namespace aeb

#endif  // AEB_CANBUS_CANMESSAGE_HPP
