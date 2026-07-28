/**
 * @file FrameQueue.hpp
 * @brief Bounded, drop-oldest, allocation-recycling frame queue.
 *
 * Decouples the lidar acquisition thread from the network I/O thread. Its
 * single responsibility is hand-off with a hard memory bound; it knows nothing
 * about sockets or the wire protocol.
 */

#ifndef AEB_NETWORK_FRAMEQUEUE_HPP
#define AEB_NETWORK_FRAMEQUEUE_HPP

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include "common/Types.hpp"

namespace aeb::net {

/**
 * @brief Thread-safe FIFO of scan frames with a fixed capacity.
 *
 * Overflow policy is **drop oldest**. Applying back-pressure to the producer is
 * forbidden: the producer is the lidar acquisition thread, and stalling it to
 * accommodate a slow development viewer would degrade the sensing path. For a
 * live visualiser the newest frame is also the only interesting one.
 *
 * Frames popped or dropped are retained in an internal pool with their point
 * vector capacity intact and reused by the next @ref push, so the steady state
 * performs no heap allocation despite copying frames between threads.
 *
 * All methods are safe to call from any thread. The lock is never held across a
 * syscall or an allocation of unbounded size, so @ref push has a short and
 * predictable critical section.
 */
class FrameQueue {
public:
    /**
     * @brief Construct a queue holding at most @p capacity frames.
     * @param capacity Maximum queued frames; values below 1 are clamped to 1.
     */
    explicit FrameQueue(std::size_t capacity);

    FrameQueue(const FrameQueue&) = delete;
    FrameQueue& operator=(const FrameQueue&) = delete;
    FrameQueue(FrameQueue&&) = delete;
    FrameQueue& operator=(FrameQueue&&) = delete;

    /**
     * @brief Copy @p frame into the queue, evicting the oldest if full.
     *
     * Never blocks beyond the internal mutex and never fails.
     *
     * @param frame Frame to enqueue. The caller retains ownership.
     * @return Number of frames evicted to make room; 0 in the nominal case.
     */
    std::size_t push(const ScanFrame& frame);

    /**
     * @brief Remove the oldest frame.
     *
     * @param[out] out Receives the frame. Its previous contents are recycled
     *                 into the pool, so passing the same object repeatedly
     *                 avoids reallocation.
     * @return @c true if a frame was dequeued; @c false if the queue is empty.
     */
    [[nodiscard]] bool pop(ScanFrame& out);

    /**
     * @brief Discard every queued frame, returning them to the pool.
     *
     * Used when a client attaches or detaches: a new session must not receive
     * frames that predate it.
     */
    void clear();

    /** @brief Whether the queue currently holds no frames. */
    [[nodiscard]] bool empty() const;

    /** @brief Number of frames currently queued. */
    [[nodiscard]] std::size_t size() const;

private:
    /**
     * @brief Return a frame object to the pool for reuse.
     * @param frame Frame to recycle; its buffer capacity is retained.
     * @pre @ref mutex_ is held by the caller.
     */
    void recycleLocked(ScanFrame&& frame);

    /**
     * @brief Obtain a frame object from the pool, or a fresh one.
     * @return A frame with cleared contents and, usually, reserved storage.
     * @pre @ref mutex_ is held by the caller.
     */
    [[nodiscard]] ScanFrame acquireLocked();

    /** @brief Guards every member below. */
    mutable std::mutex mutex_;

    /** @brief Queued frames, oldest at the front. */
    std::deque<ScanFrame> queue_;

    /** @brief Recycled frame objects with pre-grown point storage. */
    std::vector<ScanFrame> pool_;

    /** @brief Maximum number of queued frames. */
    std::size_t capacity_;
};

}  // namespace aeb::net

#endif  // AEB_NETWORK_FRAMEQUEUE_HPP
