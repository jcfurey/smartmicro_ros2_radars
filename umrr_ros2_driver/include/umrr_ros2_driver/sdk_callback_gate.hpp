// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__SDK_CALLBACK_GATE_HPP_
#define UMRR_ROS2_DRIVER__SDK_CALLBACK_GATE_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <utility>

namespace smartmicro::drivers::radar
{
// The SDK retains callbacks without exposing high-level unregistration. Callbacks
// own only this shared gate state, and cannot enter the owner after close().
// close() must be called from outside a guarded callback, before owner destruction.
// This protects the owner, not unloading the library containing callback code.
class SdkCallbackGate
{
  struct State
  {
    std::mutex mutex;
    std::condition_variable idle;
    bool closed{false};
    size_t active{0};
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
             if (lease) {callback(std::forward<decltype(args)>(args)...);}
           };
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
