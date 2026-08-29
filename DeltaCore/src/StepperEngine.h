#pragma once

#include <stdint.h>
#include "MotionQueue.h"
#include "PhaseStep3Axis.h"

namespace deltacore {

enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_ESTOP,
  FAULT_HOME_TRAVEL,
  FAULT_ENDSTOP_STUCK,
  FAULT_INTERNAL
};

enum HomeResult : int8_t {
  HOME_RESULT_NONE = 0,
  HOME_RESULT_RUNNING = 1,
  HOME_RESULT_DONE = 2,
  HOME_RESULT_FAILED = -1
};

struct StepperStats {
  uint32_t virtual_events;
  uint32_t blocks_loaded;
  uint32_t queue_empty_stops;
  uint32_t timer_guard_hits;
  uint32_t real_steps[3];
  uint32_t phase_anchors;
  uint32_t phase_boundary_corrections;
  uint32_t phase_faults;
  uint16_t min_interval_ticks;
  uint16_t max_interval_ticks;
  uint16_t max_isr_entry_ticks;
};

class StepperEngine {
public:
  StepperEngine();
  void begin(MotionQueue &queue);
  void enableMotors();
  void disableMotors();
  bool motorsEnabled() const { return motors_enabled_; }
  void servicePrefetch();
  void kickMotion();
  bool motionBusy() const;
  bool idle() const;
  bool startHomeSeek(bool slow);
  bool startHomeBackoff();
  HomeResult homeResult() const { return HomeResult(home_result_); }
  void clearHomeResult();
  bool endstopTriggered(uint8_t axis) const;
  uint8_t endstopMask() const;
  void emergencyStop(FaultCode code = FAULT_ESTOP);
  bool clearFault();
  FaultCode fault() const { return FaultCode(fault_); }
  void setMotorPositionSteps(const int32_t steps[3]);
  void getMotorPositionSteps(int32_t steps[3]) const;
  uint32_t completedStepEvents() const { return completed_step_events_; }
  void snapshotStats(StepperStats &stats) const;
  void clearStats();
  void onCompareA();
  void onCompareB();
  static StepperEngine *instance() { return instance_; }

private:
  enum Mode : uint8_t { MODE_IDLE = 0, MODE_MOTION, MODE_HOME };
  enum HomeKind : uint8_t { HOME_KIND_NONE = 0, HOME_KIND_SEEK, HOME_KIND_BACKOFF };

  MotionQueue *queue_;
  PhaseStep3Axis phase_;
  MotorBlock block_a_;
  MotorBlock block_b_;
  MotorBlock *active_block_;
  MotorBlock *prefetch_block_;
  volatile bool prefetch_valid_;
  volatile Mode mode_;
  volatile bool block_active_;
  volatile bool motors_enabled_;
  volatile uint8_t fault_;
  volatile int32_t motor_position_steps_[3];
  volatile uint32_t completed_step_events_;
  volatile uint32_t blocks_loaded_;
  volatile uint32_t queue_empty_stops_;
  volatile uint32_t timer_guard_hits_;
  volatile uint32_t real_steps_[3];
  volatile uint32_t phase_faults_;
  volatile uint16_t min_interval_ticks_;
  volatile uint16_t max_interval_ticks_;
  volatile uint16_t max_isr_entry_ticks_;
  volatile uint8_t *step_out_[3];
  volatile uint8_t *dir_out_[3];
  volatile uint8_t *enable_out_[3];
  volatile uint8_t *endstop_in_[3];
  uint8_t step_mask_[3], dir_mask_[3], enable_mask_[3], endstop_mask_[3];
  volatile uint8_t active_pulse_axes_;
  volatile bool direction_pending_;
  volatile uint8_t pending_direction_bits_;

  int32_t current_interval_q8_;
  int32_t interval_delta_q8_;

  volatile HomeKind home_kind_;
  volatile int8_t home_result_;
  volatile uint8_t home_active_axes_;
  volatile uint32_t home_events_done_;
  volatile uint32_t home_event_limit_;
  volatile uint16_t home_interval_ticks_;
  static StepperEngine *instance_;

  MotorBlock &currentBlock() { return *active_block_; }
  const MotorBlock &currentBlock() const { return *active_block_; }
  void configureFastPins();
  void writeFast(volatile uint8_t *reg, uint8_t mask, bool high);
  void writeStep(uint8_t axis, bool active);
  void applyDirectionBits(uint8_t positive_bits);
  void setEnableFast(bool enabled);
  void allStepsInactive();
  bool loadNextMotionBlock();
  void initBlockTiming(const MotorBlock &block);
  uint16_t currentMotionInterval() const;
  void advanceBlockTiming();
  void scheduleMotionInterval(uint16_t desired_ticks);
  void motionISR();
  void homeISR();
  void pulseAxes(uint8_t axes, bool positive);
  void startTimer(uint16_t first_ticks);
  void stopTimerFromISR();
  void finishHomeFromISR(bool success);
  uint16_t intervalForTowerSpeed(float mm_s) const;
};

} // namespace deltacore
