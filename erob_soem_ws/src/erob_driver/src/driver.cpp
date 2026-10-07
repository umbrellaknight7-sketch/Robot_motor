#include "erob_driver/driver.hpp"
#include "erob_driver/config.hpp"
#include "erob_driver/pdo.hpp"

#include <chrono>
#include <sched.h>
#include <stdexcept>
#include <sys/mman.h>
#include <time.h>

namespace erob {
Nanoseconds monotonic_now() noexcept {
  timespec t{};
  clock_gettime(CLOCK_MONOTONIC, &t);
  return int64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}

namespace {
Snapshot initial_state(size_t n) {
  Snapshot s;
  s.joints.resize(n);
  return s;
}

CommandFrame initial_command(size_t n) {
  CommandFrame f;
  f.joints.resize(n);
  return f;
}
} // namespace

Driver::Driver(BusConfig c)
    : config_(std::move(c)), commands_(initial_command(config_.drives.size())),
      states_(initial_state(config_.drives.size())),
      outgoing_(initial_command(config_.drives.size())) {
  validate_config(config_);
}

Driver::~Driver() {
  shutdown();
}

void Driver::configure() {
  if (worker_.joinable() || ready_.load())
    throw std::runtime_error("Driver is already configured");
  worker_ = std::thread(&Driver::run, this);
  while (!ready_.load(std::memory_order_acquire))
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (configure_error_) {
    worker_.join();
    std::rethrow_exception(configure_error_);
  }
}

bool Driver::wait_phase(Phase phase, Nanoseconds duration) {
  Snapshot s = initial_state(config_.drives.size());
  const auto until = monotonic_now() + duration;
  while (monotonic_now() < until && !exited_) {
    states_.read(s);
    if (s.health.phase == phase)
      return true;
    if (s.health.phase == Phase::Fault)
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

void Driver::request(Request wanted) noexcept {
  auto current = request_.load();
  while (int(current) < int(wanted) && !request_.compare_exchange_weak(current, wanted)) {
  }
}

bool Driver::activate() {
  validate_activation(config_);
  if (!worker_.joinable() || exited_)
    return false;
  request(Request::Activate);
  return wait_phase(Phase::Active, config_.transition_timeout_ns + ms(100));
}

bool Driver::deactivate() {
  if (!worker_.joinable() || exited_)
    return true;
  request(Request::Deactivate);
  return wait_phase(Phase::Inactive, config_.transition_timeout_ns + ms(100));
}

void Driver::shutdown() noexcept {
  if (!worker_.joinable())
    return;
  request(Request::Stop);
  worker_.join();
}

SubmitResult Driver::submit(const std::vector<Command> &values) noexcept {
  if (exited_ || !ready_)
    return {SubmitResult::Unavailable};
  auto reject = [&](RejectReason reason, int joint) {
    ++rejected_.count;
    rejected_.reason = reason;
    rejected_.joint = joint;
    rejections_.publish(rejected_);
    return SubmitResult{SubmitResult::Rejected};
  };
  if (values.size() != outgoing_.joints.size())
    return reject(RejectReason::Shape, -1);
  for (size_t i = 0; i < values.size(); ++i) {
    if (!command_valid(values[i], config_.drives[i]))
      return reject(RejectReason::Value, int(i));
    if (config_.csp_hold_enabled && values[i].mode != Mode::CSP)
      return reject(RejectReason::Mode, int(i));
  }

  // Publish only after full validation; rejection leaves the last command and timestamp intact.
  outgoing_.joints = values;
  outgoing_.stamp_ns = monotonic_now();
  ++outgoing_.generation;
  commands_.publish(outgoing_);
  return {SubmitResult::Accepted};
}

bool Driver::resume(uint64_t generation) noexcept {
  if (exited_ || !ready_ || resume_busy_.test_and_set())
    return false;
  if (resume_ack_.load() != resume_id_.load()) {
    resume_busy_.clear();
    return false;
  }
  resume_generation_ = generation;
  resume_deadline_ = monotonic_now() + ms(1000);
  const auto id = resume_id_.fetch_add(1) + 1;
  while (!exited_ && resume_ack_.load() < id && monotonic_now() < resume_deadline_.load() + ms(100))
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const bool ok = resume_ack_.load() == id && resume_ok_.load();
  resume_busy_.clear();
  return ok;
}

bool Driver::read(Snapshot &s) noexcept {
  return states_.read(s);
}

// This worker owns transport setup, cyclic bus access and transport shutdown.
void Driver::run() {
  bool configured = false;
  try {
    transport_ = make_transport(config_);
    engine_ = std::make_unique<Engine>(config_, *transport_);
    engine_->configure();
    // Allocate all loop storage before requesting locked memory.
    CommandFrame frame = initial_command(config_.drives.size());
    RejectionReport rejected;
    if (config_.backend == "soem") {
      sched_param p{};
      p.sched_priority = config_.priority;
      bool rt = sched_setscheduler(0, SCHED_FIFO, &p) == 0;
      if (config_.cpu >= 0) {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(config_.cpu, &cpus);
        rt = (sched_setaffinity(0, sizeof(cpus), &cpus) == 0) && rt;
      }
      rt = (mlockall(MCL_CURRENT) == 0) && rt;
      if (!rt && config_.require_realtime)
        throw std::runtime_error("Realtime scheduling/affinity/memory locking unavailable");
    }
    engine_->tick(monotonic_now());
    states_.publish(engine_->state());
    configured = true;
    ready_.store(true, std::memory_order_release);
    auto next = monotonic_now();
    bool stopping = false;
    Nanoseconds stop_start = 0;
    uint64_t rejected_seen = 0, resume_seen = 0;

    // Use absolute monotonic deadlines with the engine's bounded DC correction.
    while (true) {
      next += config_.cycle_ns + engine_->state().health.dc_offset_ns;
      timespec ts{time_t(next / 1000000000), long(next % 1000000000)};
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
      }
      auto now = monotonic_now();
      engine_->monitor_deadline(now, next);
      if (now - next > config_.lateness_ns)
        next = now;
      if (rejections_.read(rejected)) {
        engine_->reject(rejected.reason, rejected.joint, rejected.count - rejected_seen);
        rejected_seen = rejected.count;
      }

      auto req = request_.exchange(Request::None);
      if (req == Request::Activate && !stopping)
        engine_->activate(now);
      if (req == Request::Deactivate)
        engine_->deactivate(now);
      if (req == Request::Stop) {
        stopping = true;
        stop_start = now;
        engine_->deactivate(now);
      }

      // Read the latest complete command without replaying a backlog.
      const bool got = commands_.read(frame);
      now = monotonic_now();
      engine_->tick(now, got ? &frame : nullptr);

      // Acknowledge resume only after the current bus cycle has updated engine readiness.
      const auto resume_id = resume_id_.load();
      if (resume_id != resume_seen) {
        resume_seen = resume_id;
        resume_ok_ = !stopping && req != Request::Deactivate && request_.load() == Request::None &&
                     monotonic_now() <= resume_deadline_.load() &&
                     engine_->resume(resume_generation_.load(), monotonic_now());
        resume_ack_ = resume_id;
      }
      const auto &snapshot = engine_->state();
      states_.publish(snapshot);

      // Keep cycling during shutdown until disabling is confirmed or the stop window expires.
      if (stopping && ((snapshot.health.phase == Phase::Inactive) ||
                       now - stop_start > config_.transition_timeout_ns ||
                       (snapshot.health.phase == Phase::Fault &&
                        now - stop_start > config_.disable_delay_ns + config_.cycle_ns * 2)))
        break;
    }
  } catch (...) {
    if (!configured)
      configure_error_ = std::current_exception();
    else if (engine_) {
      engine_->trip(Fault::TransportError, -1, monotonic_now());
      states_.publish(engine_->state());
    }
  }
  if (transport_)
    transport_->close();
  exited_ = true;
  ready_.store(true, std::memory_order_release);
}
} // namespace erob
