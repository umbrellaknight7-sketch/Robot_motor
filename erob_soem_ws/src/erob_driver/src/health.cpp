#include "erob_driver/types.hpp"

namespace erob {
const char *phase_text(Phase p) noexcept {
  switch (p) {
  case Phase::Inactive:
    return "inactive";
  case Phase::Enabling:
    return "enabling";
  case Phase::Active:
    return "active";
  case Phase::Disabling:
    return "disabling";
  case Phase::Fault:
    return "fault";
  case Phase::Decelerating:
    return "decelerating";
  case Phase::Holding:
    return "holding";
  case Phase::HoldUnconfirmed:
    return "hold_unconfirmed";
  }
  return "unknown";
}

const char *reject_text(RejectReason r) noexcept {
  switch (r) {
  case RejectReason::None:
    return "none";
  case RejectReason::Shape:
    return "wrong joint count";
  case RejectReason::Value:
    return "non-finite, out-of-limit or overflowing value";
  case RejectReason::Stale:
    return "stale or future timestamp";
  case RejectReason::Order:
    return "out-of-order generation";
  case RejectReason::Mode:
    return "non-CSP command";
  }
  return "unknown";
}
}
