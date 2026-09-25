// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__SDK_CALLBACK_GATE_HPP_
#define UMRR_ROS2_DRIVER__SDK_CALLBACK_GATE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace smartmicro::drivers::radar
{
// The SDK retains callbacks without exposing high-level unregistration. Callbacks
// own only this shared gate state, and cannot enter the owner after close().
// close() must be called from outside a guarded callback, before owner destruction.
// This protects the owner, not unloading the library containing callback code.
//
// Exceptions never leave a wrapped callback: vendor threads are not prepared for
// them and an escaping exception terminates the process. They are counted and
// reported through the error handler, at most once per second per gate.
class SdkCallbackGate
{
public:
  using ErrorHandler = std::function<void (const std::string & message)>;

private:
  struct State
  {
    std::mutex mutex;
    std::condition_variable idle;
    bool closed{false};
    size_t active{0};

    std::mutex error_mutex;
    ErrorHandler on_error;
    std::chrono::steady_clock::time_point last_report{};
    bool reported{false};
    uint64_t exceptions{0};
    uint64_t suppressed{0};
  };

  class Lease
  {
  public:
    explicit Lease(std::shared_ptr<State> state) : state_(std::move(state))
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      entered_ = !state_->closed;
      if (entered_) {++state_->active;}
    }
    ~Lease()
    {
      if (entered_) {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (--state_->active == 0) {state_->idle.notify_all();}
      }
    }
    explicit operator bool() const {return entered_;}
    Lease(const Lease &) = delete;
    Lease & operator=(const Lease &) = delete;

  private:
    std::shared_ptr<State> state_;
    bool entered_{false};
  };

  static void report(const std::shared_ptr<State> & state, const char * what) noexcept
  {
    try {
      ErrorHandler handler;
      std::string message;
      {
        std::lock_guard<std::mutex> lock(state->error_mutex);
        ++state->exceptions;
        const auto now = std::chrono::steady_clock::now();
        if (state->reported && now - state->last_report < std::chrono::seconds(1)) {
          ++state->suppressed;
          return;
        }
        message = std::string("Exception in SDK callback: ") + what;
        if (state->suppressed) {
          message += " (" + std::to_string(state->suppressed) + " similar suppressed)";
        }
        state->suppressed = 0;
        state->reported = true;
        state->last_report = now;
        handler = state->on_error;
      }
      if (handler) {
        handler(message);
      } else {
        std::fprintf(stderr, "[smartmicro] %s\n", message.c_str());
      }
    } catch (...) {
      // Reporting must never throw into a vendor thread.
    }
  }

  static void close(const std::shared_ptr<State> & state)
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    state->closed = true;
    state->idle.wait(lock, [&] {return state->active == 0;});
  }

public:
  ~SdkCallbackGate() {close();}
  SdkCallbackGate() = default;
  SdkCallbackGate(const SdkCallbackGate &) = delete;
  SdkCallbackGate & operator=(const SdkCallbackGate &) = delete;

  template<typename F>
  auto wrap(F callback) const
  {
    return [state = state_, callback = std::move(callback)](auto && ... args) {
             Lease lease(state);
             if (!lease) {return;}
             try {
               callback(std::forward<decltype(args)>(args)...);
             } catch (const std::exception & error) {
               report(state, error.what());
             } catch (...) {
               report(state, "unknown exception");
             }
           };
  }

  // The handler runs on the SDK thread that raised the exception; keep it cheap.
  void set_error_handler(ErrorHandler handler)
  {
    std::lock_guard<std::mutex> lock(state_->error_mutex);
    state_->on_error = std::move(handler);
  }

  uint64_t exception_count() const
  {
    std::lock_guard<std::mutex> lock(state_->error_mutex);
    return state_->exceptions;
  }

  void close() {close(state_);}

  // Safe to retain in the ROS context after the gate/owner itself is destroyed.
  auto shutdown_callback() const
  {
    return [state = state_] {close(state);};
  }

private:
  std::shared_ptr<State> state_{std::make_shared<State>()};
};
}  // namespace smartmicro::drivers::radar
#endif
