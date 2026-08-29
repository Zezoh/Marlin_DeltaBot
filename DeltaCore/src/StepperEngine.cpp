#include "StepperEngine.h"
#include "MachineConfig.h"
#include "HardwareConfig.h"
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

namespace deltacore {
StepperEngine *StepperEngine::instance_=nullptr;

StepperEngine::StepperEngine():queue_(nullptr),block_a_{},block_b_{},active_block_(&block_a_),prefetch_block_(&block_b_),prefetch_valid_(false),mode_(MODE_IDLE),block_active_(false),motors_enabled_(false),fault_(FAULT_NONE),motor_position_steps_{0,0,0},completed_step_events_(0),blocks_loaded_(0),queue_empty_stops_(0),timer_guard_hits_(0),real_steps_{0,0,0},min_interval_ticks_(0xFFFF),max_interval_ticks_(0),max_isr_entry_ticks_(0),step_out_{nullptr,nullptr,nullptr},dir_out_{nullptr,nullptr,nullptr},enable_out_{nullptr,nullptr,nullptr},endstop_in_{nullptr,nullptr,nullptr},step_mask_{0,0,0},dir_mask_{0,0,0},enable_mask_{0,0,0},endstop_mask_{0,0,0},active_pulse_axes_(0),direction_pending_(false),pending_direction_bits_(0),dda_accum_{0,0,0},event_index_(0),timing_accum_(0),home_kind_(HOME_KIND_NONE),home_result_(HOME_RESULT_NONE),home_active_axes_(0),home_events_done_(0),home_event_limit_(0),home_interval_ticks_(0){}

void StepperEngine::writeFast(volatile uint8_t*r,uint8_t m,bool h){if(h)*r|=m;else *r&=uint8_t(~m);} 
void StepperEngine::configureFastPins(){for(uint8_t a=0;a<3;++a){pinMode(hwcfg::STEP_PINS[a],OUTPUT);pinMode(hwcfg::DIR_PINS[a],OUTPUT);pinMode(hwcfg::ENABLE_PINS[a],OUTPUT);pinMode(hwcfg::MAX_ENDSTOP_PINS[a],INPUT_PULLUP);step_mask_[a]=digitalPinToBitMask(hwcfg::STEP_PINS[a]);dir_mask_[a]=digitalPinToBitMask(hwcfg::DIR_PINS[a]);enable_mask_[a]=digitalPinToBitMask(hwcfg::ENABLE_PINS[a]);endstop_mask_[a]=digitalPinToBitMask(hwcfg::MAX_ENDSTOP_PINS[a]);step_out_[a]=portOutputRegister(digitalPinToPort(hwcfg::STEP_PINS[a]));dir_out_[a]=portOutputRegister(digitalPinToPort(hwcfg::DIR_PINS[a]));enable_out_[a]=portOutputRegister(digitalPinToPort(hwcfg::ENABLE_PINS[a]));endstop_in_[a]=portInputRegister(digitalPinToPort(hwcfg::MAX_ENDSTOP_PINS[a]));}}

void StepperEngine::begin(MotionQueue&q){instance_=this;queue_=&q;configureFastPins();allStepsInactive();setEnableFast(false);clearStats();noInterrupts();TCCR1A=0;TCCR1B=_BV(WGM12)|_BV(CS11);TCNT1=0;OCR1A=cfg::STARTUP_EVENT_TICKS;OCR1B=cfg::STEP_PULSE_TICKS;TIMSK1=0;TIFR1=_BV(OCF1A)|_BV(OCF1B);interrupts();}

void StepperEngine::snapshotStats(StepperStats&s)const{noInterrupts();s.virtual_events=completed_step_events_;s.blocks_loaded=blocks_loaded_;s.queue_empty_stops=queue_empty_stops_;s.timer_guard_hits=timer_guard_hits_;for(uint8_t a=0;a<3;++a)s.real_steps[a]=real_steps_[a];s.phase_anchors=s.phase_boundary_corrections=s.phase_faults=0;s.min_interval_ticks=min_interval_ticks_==0xFFFF?0:min_interval_ticks_;s.max_interval_ticks=max_interval_ticks_;s.max_isr_entry_ticks=max_isr_entry_ticks_;interrupts();}
void StepperEngine::clearStats(){noInterrupts();completed_step_events_=blocks_loaded_=queue_empty_stops_=timer_guard_hits_=0;for(uint8_t a=0;a<3;++a)real_steps_[a]=0;min_interval_ticks_=0xFFFF;max_interval_ticks_=max_isr_entry_ticks_=0;interrupts();}

void StepperEngine::writeStep(uint8_t a,bool active){writeFast(step_out_[a],step_mask_[a],active?!hwcfg::STEP_INVERTING[a]:hwcfg::STEP_INVERTING[a]);}
void StepperEngine::allStepsInactive(){for(uint8_t a=0;a<3;++a)writeStep(a,false);active_pulse_axes_=0;}
void StepperEngine::applyDirectionBits(uint8_t bits){for(uint8_t a=0;a<3;++a){bool p=bits&(1U<<a);writeFast(dir_out_[a],dir_mask_[a],p?!hwcfg::DIR_INVERTING[a]:hwcfg::DIR_INVERTING[a]);}}
void StepperEngine::setEnableFast(bool e){for(uint8_t a=0;a<3;++a)writeFast(enable_out_[a],enable_mask_[a],e?hwcfg::ENABLE_ACTIVE_HIGH[a]:!hwcfg::ENABLE_ACTIVE_HIGH[a]);motors_enabled_=e;}
void StepperEngine::enableMotors(){noInterrupts();setEnableFast(true);interrupts();}
void StepperEngine::disableMotors(){if(!idle())return;noInterrupts();setEnableFast(false);interrupts();}

bool StepperEngine::endstopTriggered(uint8_t a)const{if(a>=3)return false;bool h=(*endstop_in_[a]&endstop_mask_[a])!=0;return h!=hwcfg::MAX_ENDSTOP_INVERTING[a];}
uint8_t StepperEngine::endstopMask()const{uint8_t m=0;for(uint8_t a=0;a<3;++a)if(endstopTriggered(a))m|=1U<<a;return m;}

void StepperEngine::startTimer(uint16_t t){TCNT1=0;OCR1A=t;TIFR1=_BV(OCF1A)|_BV(OCF1B);TIMSK1=_BV(OCIE1A);}
void StepperEngine::stopTimerFromISR(){TIMSK1=0;allStepsInactive();}

void StepperEngine::servicePrefetch(){
  if(!queue_)return;
  const uint8_t saved_sreg=SREG;
  cli();
  if(!prefetch_valid_&&mode_!=MODE_HOME&&!queue_->empty()){
    if(queue_->popFromISR(*prefetch_block_)){
      asm volatile("":::"memory");
      prefetch_valid_=true;
    }
  }
  SREG=saved_sreg;
}

bool StepperEngine::loadNextMotionBlock(){if(!prefetch_valid_){if(!queue_||!queue_->popFromISR(*prefetch_block_))return false;prefetch_valid_=true;}MotorBlock*old=active_block_;active_block_=prefetch_block_;prefetch_block_=old;prefetch_valid_=false;const MotorBlock&b=currentBlock();if(!b.event_count||!b.interval_base_ticks){fault_=FAULT_INTERNAL;return false;}for(uint8_t a=0;a<3;++a)dda_accum_[a]=b.event_count>>1;event_index_=0;timing_accum_=0;++blocks_loaded_;block_active_=true;return true;}

uint16_t StepperEngine::nextMotionInterval(){const MotorBlock&b=currentBlock();uint16_t t=b.interval_base_ticks;uint16_t x=uint16_t(timing_accum_+b.interval_remainder_ticks);if(x>=b.event_count){x=uint16_t(x-b.event_count);++t;}timing_accum_=x;return t;}

void StepperEngine::scheduleMotionInterval(uint16_t t){if(t<cfg::MIN_EVENT_INTERVAL_TICKS)t=cfg::MIN_EVENT_INTERVAL_TICKS;if(t>cfg::MAX_EVENT_INTERVAL_TICKS)t=cfg::MAX_EVENT_INTERVAL_TICKS;uint16_t e=TCNT1;uint32_t earliest=uint32_t(e)+cfg::TIMER_ISR_GUARD_TICKS;if(uint32_t(t)<=earliest){t=earliest>cfg::MAX_EVENT_INTERVAL_TICKS?cfg::MAX_EVENT_INTERVAL_TICKS:uint16_t(earliest);++timer_guard_hits_;}if(t<min_interval_ticks_)min_interval_ticks_=t;if(t>max_interval_ticks_)max_interval_ticks_=t;OCR1A=t;}

void StepperEngine::kickMotion(){if(fault_!=FAULT_NONE)return;servicePrefetch();if(!prefetch_valid_)return;noInterrupts();if(mode_==MODE_IDLE){mode_=MODE_MOTION;block_active_=false;setEnableFast(true);startTimer(cfg::STARTUP_EVENT_TICKS);}interrupts();}
bool StepperEngine::motionBusy()const{return mode_==MODE_MOTION||block_active_||prefetch_valid_||(queue_&&!queue_->empty());}
bool StepperEngine::idle()const{return mode_==MODE_IDLE&&!block_active_&&!prefetch_valid_&&(!queue_||queue_->empty());}

uint16_t StepperEngine::intervalForTowerSpeed(float mm_s)const{float r=mm_s*cfg::STEPS_PER_MM;if(r<1)r=1;uint32_t t=uint32_t(float(cfg::TIMER_HZ)/r+.5f);if(t<cfg::MIN_EVENT_INTERVAL_TICKS)t=cfg::MIN_EVENT_INTERVAL_TICKS;if(t>cfg::MAX_EVENT_INTERVAL_TICKS)t=cfg::MAX_EVENT_INTERVAL_TICKS;return uint16_t(t);}

bool StepperEngine::startHomeSeek(bool slow){if(!idle()||fault_!=FAULT_NONE)return false;noInterrupts();queue_->clearUnsafe();prefetch_valid_=false;setEnableFast(true);home_kind_=HOME_KIND_SEEK;home_result_=HOME_RESULT_RUNNING;home_active_axes_=7;home_events_done_=0;home_event_limit_=uint32_t(cfg::HOME_MAX_TRAVEL_MM*cfg::STEPS_PER_MM+.5f);home_interval_ticks_=intervalForTowerSpeed(slow?cfg::HOME_SLOW_MM_S:cfg::HOME_FAST_MM_S);applyDirectionBits(7);mode_=MODE_HOME;startTimer(cfg::STARTUP_EVENT_TICKS);interrupts();return true;}
bool StepperEngine::startHomeBackoff(){if(!idle()||fault_!=FAULT_NONE)return false;noInterrupts();prefetch_valid_=false;setEnableFast(true);home_kind_=HOME_KIND_BACKOFF;home_result_=HOME_RESULT_RUNNING;home_active_axes_=7;home_events_done_=0;home_event_limit_=uint32_t(cfg::HOME_BACKOFF_MM*cfg::STEPS_PER_MM+.5f);home_interval_ticks_=intervalForTowerSpeed(15);applyDirectionBits(0);mode_=MODE_HOME;startTimer(cfg::STARTUP_EVENT_TICKS);interrupts();return true;}
void StepperEngine::clearHomeResult(){noInterrupts();if(home_result_!=HOME_RESULT_RUNNING)home_result_=HOME_RESULT_NONE;interrupts();}

void StepperEngine::pulseAxes(uint8_t axes,bool positive){uint8_t p=0;for(uint8_t a=0;a<3;++a){uint8_t bit=1U<<a;if(!(axes&bit))continue;writeStep(a,true);p|=bit;motor_position_steps_[a]+=positive?1:-1;}active_pulse_axes_=p;if(p){uint16_t pe=uint16_t(TCNT1+cfg::STEP_PULSE_TICKS);if(pe>=OCR1A)pe=uint16_t(OCR1A-1);OCR1B=pe;TIFR1=_BV(OCF1B);TIMSK1|=_BV(OCIE1B);}}

void StepperEngine::motionISR(){
  if(!block_active_){if(!loadNextMotionBlock()){mode_=MODE_IDLE;if(fault_!=FAULT_NONE)setEnableFast(false);stopTimerFromISR();return;}applyDirectionBits(currentBlock().direction_bits);scheduleMotionInterval(nextMotionInterval());return;}
  const MotorBlock&b=currentBlock();uint8_t pulse=0;
  for(uint8_t a=0;a<3;++a){uint16_t x=uint16_t(dda_accum_[a]+b.steps[a]);if(x>=b.event_count){x=uint16_t(x-b.event_count);uint8_t bit=1U<<a;writeStep(a,true);pulse|=bit;++real_steps_[a];motor_position_steps_[a]+=(b.direction_bits&bit)?1:-1;}dda_accum_[a]=x;}
  ++event_index_;++completed_step_events_;active_pulse_axes_=pulse;
  if(pulse){uint16_t pe=uint16_t(TCNT1+cfg::STEP_PULSE_TICKS);if(pe>=OCR1A)pe=uint16_t(OCR1A-1);OCR1B=pe;TIFR1=_BV(OCF1B);TIMSK1|=_BV(OCIE1B);}
  if(event_index_>=b.event_count){block_active_=false;if(loadNextMotionBlock()){if(active_pulse_axes_){pending_direction_bits_=currentBlock().direction_bits;direction_pending_=true;}else{applyDirectionBits(currentBlock().direction_bits);direction_pending_=false;}scheduleMotionInterval(nextMotionInterval());}else if(fault_==FAULT_NONE){++queue_empty_stops_;mode_=MODE_IDLE;stopTimerFromISR();}else{mode_=MODE_IDLE;setEnableFast(false);stopTimerFromISR();}}
  else scheduleMotionInterval(nextMotionInterval());
}

void StepperEngine::finishHomeFromISR(bool ok){stopTimerFromISR();mode_=MODE_IDLE;home_kind_=HOME_KIND_NONE;home_result_=ok?HOME_RESULT_DONE:HOME_RESULT_FAILED;}
void StepperEngine::homeISR(){if(home_kind_==HOME_KIND_SEEK){uint8_t r=home_active_axes_;for(uint8_t a=0;a<3;++a){uint8_t bit=1U<<a;if((r&bit)&&endstopTriggered(a))r&=uint8_t(~bit);}home_active_axes_=r;if(!r){finishHomeFromISR(true);return;}if(home_events_done_>=home_event_limit_){fault_=FAULT_HOME_TRAVEL;setEnableFast(false);finishHomeFromISR(false);return;}pulseAxes(r,true);++home_events_done_;OCR1A=home_interval_ticks_;return;}if(home_kind_==HOME_KIND_BACKOFF){if(home_events_done_>=home_event_limit_){finishHomeFromISR(true);return;}pulseAxes(7,false);++home_events_done_;OCR1A=home_interval_ticks_;return;}fault_=FAULT_INTERNAL;setEnableFast(false);finishHomeFromISR(false);}

void StepperEngine::onCompareA(){uint16_t e=TCNT1;if(e>max_isr_entry_ticks_)max_isr_entry_ticks_=e;if(mode_==MODE_MOTION)motionISR();else if(mode_==MODE_HOME)homeISR();else stopTimerFromISR();}
void StepperEngine::onCompareB(){uint8_t p=active_pulse_axes_;for(uint8_t a=0;a<3;++a)if(p&(1U<<a))writeStep(a,false);active_pulse_axes_=0;if(direction_pending_){applyDirectionBits(pending_direction_bits_);direction_pending_=false;}TIMSK1&=uint8_t(~_BV(OCIE1B));}

void StepperEngine::emergencyStop(FaultCode c){noInterrupts();TIMSK1=0;allStepsInactive();if(queue_)queue_->clearUnsafe();prefetch_valid_=false;block_active_=false;mode_=MODE_IDLE;home_kind_=HOME_KIND_NONE;home_result_=HOME_RESULT_FAILED;fault_=c;setEnableFast(false);interrupts();}
bool StepperEngine::clearFault(){if(!idle())return false;noInterrupts();fault_=FAULT_NONE;home_result_=HOME_RESULT_NONE;prefetch_valid_=false;interrupts();return true;}
void StepperEngine::setMotorPositionSteps(const int32_t s[3]){noInterrupts();for(uint8_t a=0;a<3;++a)motor_position_steps_[a]=s[a];interrupts();}
void StepperEngine::getMotorPositionSteps(int32_t s[3])const{noInterrupts();for(uint8_t a=0;a<3;++a)s[a]=motor_position_steps_[a];interrupts();}
} // namespace deltacore

ISR(TIMER1_COMPA_vect){if(deltacore::StepperEngine::instance())deltacore::StepperEngine::instance()->onCompareA();}
ISR(TIMER1_COMPB_vect){if(deltacore::StepperEngine::instance())deltacore::StepperEngine::instance()->onCompareB();}
