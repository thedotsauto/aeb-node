/**
 * @file SignalWaiter.cpp
 * @brief Implementation of synchronous termination-signal handling.
 */

#include "app/SignalWaiter.hpp"

#include <pthread.h>

#include <cerrno>
#include <ctime>

namespace aeb::app {
namespace {

/** @brief Nanoseconds per millisecond. */
constexpr long kNanosPerMilli = 1000000L;

/** @brief Milliseconds per second. */
constexpr long kMillisPerSecond = 1000L;

}  // namespace

SignalWaiter::SignalWaiter() noexcept
{
    sigemptyset(&mask_);
    sigaddset(&mask_, SIGINT);
    sigaddset(&mask_, SIGTERM);
    sigaddset(&mask_, SIGPIPE);
    installed_ = (::pthread_sigmask(SIG_BLOCK, &mask_, &previous_) == 0);
}

SignalWaiter::~SignalWaiter()
{
    if (installed_) {
        (void)::pthread_sigmask(SIG_SETMASK, &previous_, nullptr);
    }
}

int SignalWaiter::waitFor(std::chrono::milliseconds timeout) const noexcept
{
    if (!installed_) {
        return 0;
    }

    const long millis = static_cast<long>(timeout.count() < 0 ? 0 : timeout.count());
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(millis / kMillisPerSecond);
    ts.tv_nsec = (millis % kMillisPerSecond) * kNanosPerMilli;

    for (;;) {
        const int signal_number = ::sigtimedwait(&mask_, nullptr, &ts);
        if (signal_number > 0) {
            if (signal_number == SIGPIPE) {
                continue;  // Absorbed: a dropped client is not a shutdown.
            }
            return signal_number;
        }
        if (errno == EINTR) {
            continue;  // Interrupted by an unblocked signal; keep waiting.
        }
        return 0;  // EAGAIN: the timeout expired.
    }
}

}  // namespace aeb::app
