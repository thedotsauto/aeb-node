/**
 * @file FrameQueue.cpp
 * @brief Implementation of the bounded, drop-oldest frame queue.
 */

#include "network/FrameQueue.hpp"

#include <utility>

namespace aeb::net {
namespace {

/**
 * @brief Extra frame objects kept in the pool beyond the queue capacity.
 *
 * Two spares cover the frames temporarily held by the producer and by the
 * consumer while they are outside the queue.
 */
constexpr std::size_t kPoolSlack = 2U;

}  // namespace

FrameQueue::FrameQueue(std::size_t capacity) : capacity_{capacity == 0U ? 1U : capacity}
{
    pool_.reserve(capacity_ + kPoolSlack);
}

std::size_t FrameQueue::push(const ScanFrame& frame)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    std::size_t evicted = 0U;
    while (queue_.size() >= capacity_) {
        recycleLocked(std::move(queue_.front()));
        queue_.pop_front();
        ++evicted;
    }

    ScanFrame slot = acquireLocked();
    slot.sequence = frame.sequence;
    slot.timestamp_us = frame.timestamp_us;
    // assign() reuses the recycled capacity instead of allocating.
    slot.points.assign(frame.points.begin(), frame.points.end());
    queue_.push_back(std::move(slot));

    return evicted;
}

bool FrameQueue::pop(ScanFrame& out)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
        return false;
    }
    recycleLocked(std::move(out));
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void FrameQueue::clear()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
        recycleLocked(std::move(queue_.front()));
        queue_.pop_front();
    }
}

bool FrameQueue::empty() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

std::size_t FrameQueue::size() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

void FrameQueue::recycleLocked(ScanFrame&& frame)
{
    if (pool_.size() >= capacity_ + kPoolSlack) {
        return;  // Pool is full; let this frame's storage be released.
    }
    frame.points.clear();  // Keeps capacity, drops contents.
    pool_.push_back(std::move(frame));
}

ScanFrame FrameQueue::acquireLocked()
{
    if (pool_.empty()) {
        return ScanFrame{};
    }
    ScanFrame frame = std::move(pool_.back());
    pool_.pop_back();
    return frame;
}

}  // namespace aeb::net
