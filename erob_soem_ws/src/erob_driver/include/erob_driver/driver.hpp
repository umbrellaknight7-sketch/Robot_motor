#pragma once

#include "erob_driver/engine.hpp"
#include "erob_driver/handoff.hpp"

#include <atomic>
#include <thread>

namespace erob {
Nanoseconds monotonic_now() noexcept;

class Driver {
public:
  explicit Driver(BusConfig);
  ~Driver();
  void configure();
  bool activate();
  bool deactivate();
  void shutdown() noexcept;
  SubmitResult submit(const std::vector<Command> &) noexcept;
  bool resume(uint64_t generation) noexcept; // Non-RT, bounded worker acknowledgment.
  bool read(Snapshot &) noexcept;            // One consumer; lifecycle calls must not
                                             // overlap read().

  const BusConfig &config() const {
    return config_;
  }

private:
  enum class Request { None, Activate, Deactivate, Stop };
  void run();
  void request(Request) noexcept;
  BusConfig config_;
  std::unique_ptr<Transport> transport_;
  std::unique_ptr<Engine> engine_;
  Latest<CommandFrame> commands_;
  Latest<Snapshot> states_;
  CommandFrame outgoing_;
  std::thread worker_;
  std::atomic<Request> request_{Request::None};
  std::atomic<bool> ready_{false}, exited_{false};

  struct RejectionReport {
    uint64_t count = 0;
    RejectReason reason = RejectReason::None;
    int joint = -1;
  };

  Latest<RejectionReport> rejections_{RejectionReport{}};
  RejectionReport rejected_; // Only the command producer accesses this copy.
  std::atomic_flag resume_busy_ = ATOMIC_FLAG_INIT;
  std::atomic<uint64_t> resume_id_{0}, resume_ack_{0}, resume_generation_{0};
  std::atomic<Nanoseconds> resume_deadline_{0};
  std::atomic<bool> resume_ok_{false};
  std::exception_ptr configure_error_;
  bool wait_phase(Phase, Nanoseconds);
};
} // namespace erob
