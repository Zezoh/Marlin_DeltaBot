#pragma once
#include <stdint.h>

namespace deltacore {

constexpr uint8_t PHASE_FRAC_BITS = 15;
constexpr int32_t PHASE_ONE = int32_t(1L << PHASE_FRAC_BITS);
constexpr int32_t PHASE_HALF = int32_t(1L << (PHASE_FRAC_BITS - 1));

struct PhaseStepMask {
  uint8_t bits;
  uint8_t positive_bits;
};

class PhaseStep3Axis {
public:
  PhaseStep3Axis();

  bool begin(const int32_t phase_start_q15[3],
             const int32_t phase_end_q15[3],
             const int32_t phase_inc_q15[3],
             uint32_t total_events,
             bool anchor,
             const int32_t actual_steps[3]);

  // Fast block-to-block path. Fractional phase remains exactly where the
  // previous block ended, so no repeated phase_start array is required.
  bool beginContinuation(const int32_t phase_end_q15[3],
                         const int32_t phase_inc_q15[3],
                         uint32_t total_events,
                         const int32_t actual_steps[3]);

  PhaseStepMask next();
  bool active() const { return completed_events_ < total_events_; }
  bool valid() const { return valid_; }
  uint32_t totalEvents() const { return total_events_; }
  uint32_t completedEvents() const { return completed_events_; }
  uint32_t boundaryCorrections() const { return boundary_corrections_; }
  uint32_t anchors() const { return anchors_; }

  void invalidate();
  void syncOutputSteps(const int32_t actual_steps[3]);
  void clearDiagnostics() { boundary_corrections_ = 0; anchors_ = 0; }

private:
  int32_t phase_q15_[3];
  int32_t phase_end_q15_[3];
  int32_t phase_inc_q15_[3];
  int32_t output_steps_[3];
  uint32_t total_events_;
  uint32_t completed_events_;
  uint32_t boundary_corrections_;
  uint32_t anchors_;
  bool valid_;

  static int32_t roundedSteps(int32_t q15);
};

} // namespace deltacore
