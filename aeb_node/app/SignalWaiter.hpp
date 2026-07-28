/**
 * @file SignalWaiter.hpp
 * @brief Deterministic termination-signal handling with no global state.
 *
 * The conventional approach - an asynchronous handler setting a file-scope
 * @c volatile @c sig_atomic_t - is rejected here for three reasons:
 *
 *  1. it requires a global mutable variable, which this project forbids;
 *  2. the handler runs on an arbitrary thread at an arbitrary instruction, so
 *     everything it touches must be async-signal-safe;
 *  3. shutdown ordering becomes non-deterministic, which is unacceptable when
 *     components own hardware.
 *
 * Instead the termination signals are *blocked* in every thread and consumed
 * synchronously with @c sigtimedwait, on a thread of our choosing, at a point
 * of our choosing.
 */

#ifndef AEB_APP_SIGNALWAITER_HPP
#define AEB_APP_SIGNALWAITER_HPP

#include <csignal>

#include <chrono>

namespace aeb::app {

/**
 * @brief Blocks and synchronously waits for termination signals.
 *
 * Construction installs the signal mask; destruction restores the previous one.
 *
 * @warning Must be constructed **before** any worker thread, because threads
 *          inherit the signal mask of their creator. If a thread is created
 *          first, a signal may still be delivered to it asynchronously.
 */
class SignalWaiter {
public:
    /**
     * @brief Block @c SIGINT, @c SIGTERM and @c SIGPIPE for this thread and
     *        every thread created after it.
     *
     * @c SIGPIPE is included because a visualisation client disconnecting
     * mid-write must never terminate the node. The networking layer also
     * requests @c MSG_NOSIGNAL; this is defence in depth.
     */
    SignalWaiter() noexcept;

    /** @brief Restore the previously installed signal mask. */
    ~SignalWaiter();

    SignalWaiter(const SignalWaiter&) = delete;
    SignalWaiter& operator=(const SignalWaiter&) = delete;
    SignalWaiter(SignalWaiter&&) = delete;
    SignalWaiter& operator=(SignalWaiter&&) = delete;

    /**
     * @brief Whether the signal mask was installed successfully.
     *
     * If this is @c false the process would terminate abruptly on @c SIGINT,
     * so the caller should treat it as a fatal start-up error.
     */
    [[nodiscard]] bool valid() const noexcept { return installed_; }

    /**
     * @brief Wait up to @p timeout for a termination signal.
     *
     * @c SIGPIPE is absorbed and waiting resumes; it is never a reason to shut
     * down.
     *
     * @param timeout Maximum time to wait.
     * @return The signal number received, or 0 if the timeout expired.
     */
    [[nodiscard]] int waitFor(std::chrono::milliseconds timeout) const noexcept;

private:
    /** @brief The set of signals handled synchronously. */
    sigset_t mask_{};

    /** @brief Mask to restore on destruction. */
    sigset_t previous_{};

    /** @brief Whether the mask was installed. */
    bool installed_{false};
};

}  // namespace aeb::app

#endif  // AEB_APP_SIGNALWAITER_HPP
