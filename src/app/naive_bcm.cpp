#include "bcm/app/naive_bcm.hpp"

#include "bcm/can/bcm_db.hpp"

namespace bcm::app {

using hal::Din;
using hal::Dout;

void NaiveBcm::step() {
  const uint32_t now = hal_.tick_ms();
  if (hal_.can_state() == hal::CanState::BusOff) hal_.can_request_recovery();

  bool rke_lock = false, rke_unlock = false, crash_edge = false;
  can::Frame f;
  while (hal_.can_receive(f)) {
    switch (f.id) {
      case can::id::kEscVehicleSpeed: speed_ = can::get(f, "VehicleSpeed"); break;
      case can::id::kAcuCrashStatus: {
        const bool c = can::get(f, "CrashDetected") > 0.5;
        if (c && !crash_) crash_edge = true;
        crash_ = c;
        break;
      }
      case can::id::kRlsRainLight:
        rain_ = can::get(f, "RainIntensity");
        lux_ = can::get(f, "AmbientLight");
        break;
      case can::id::kRkeCommand:
        rke_lock = can::get(f, "RkeLockReq") > 0.5;
        rke_unlock = can::get(f, "RkeUnlockReq") > 0.5;
        break;
      default: break;
    }
  }

  // power mode straight from the key contacts
  const bool acc = hal_.read(Din::IgnAcc), run = hal_.read(Din::IgnRun), start = hal_.read(Din::IgnStart);
  hal_.write(Dout::AccRelay, acc && !start);
  hal_.write(Dout::RunRelay, run);
  hal_.write(Dout::StarterRelay, start);

  // turn / hazard
  const bool hz = hal_.read(Din::HazardSw);
  if (hz && !prev_hazard_) hazard_ = !hazard_;
  prev_hazard_ = hz;
  if (crash_edge) hazard_ = true;
  int req = 0;
  if (hazard_) {
    req = 3;
  } else if (run && hal_.read(Din::TurnLeft)) {
    req = 1;
  } else if (run && hal_.read(Din::TurnRight)) {
    req = 2;
  }
  if (req != req_) {
    req_ = req;
    flash_start_ = now;
  }
  const bool phase_on = ((now - flash_start_) % 666) < 333;
  hal_.write(Dout::TurnLeftLamp, phase_on && (req_ == 1 || req_ == 3));
  hal_.write(Dout::TurnRightLamp, phase_on && (req_ == 2 || req_ == 3));

  // lighting
  bool low = false;
  if (hal_.read(Din::HeadlampOn) && acc) low = true;
  if (hal_.read(Din::HeadlampAuto) && run && lux_ < 1000.0) low = true;
  hal_.write(Dout::LowBeam, low);
  hal_.write(Dout::HighBeam, low && hal_.read(Din::HighBeamSw));

  // locks
  const bool ls = hal_.read(Din::LockSw), us = hal_.read(Din::UnlockSw);
  if (pulse_ != 0 && now - pulse_start_ >= 200) {
    locked_ = pulse_ > 0;
    pulse_ = 0;
  }
  if (crash_edge || (us && !prev_unlock_) || rke_unlock) {
    pulse_ = -1;
    pulse_start_ = now;
  } else if (pulse_ == 0 && ((ls && !prev_lock_) || rke_lock || (run && speed_ > 15.0 && !locked_))) {
    pulse_ = 1;
    pulse_start_ = now;
  }
  prev_lock_ = ls;
  prev_unlock_ = us;
  hal_.write(Dout::LockMotor, pulse_ > 0);
  hal_.write(Dout::UnlockMotor, pulse_ < 0);

  // wipers
  bool wl = false, wh = false;
  if (acc && !start) {
    if (hal_.read(Din::WiperHigh)) {
      wh = true;
    } else if (hal_.read(Din::WiperLow)) {
      wl = true;
    } else if (hal_.read(Din::WiperAuto)) {
      if (rain_ >= 70) {
        wh = true;
      } else if (rain_ >= 30) {
        wl = true;
      } else if (rain_ >= 5) {
        if (now - int_start_ >= 4000) int_start_ = now;
        wl = (now - int_start_) < 1000;
      }
    }
    if (hal_.read(Din::WasherSw)) wl = true;
  }
  hal_.write(Dout::WiperLow, wl);
  hal_.write(Dout::WiperHigh, wh);
  hal_.write(Dout::WasherPump, acc && hal_.read(Din::WasherSw));

  if (now - last_status_ >= 100) {
    last_status_ = now;
    can::Frame s = can::make_frame(can::id::kBcmStatus);
    can::set(s, "LockState", locked_ ? 1 : 0);
    can::set(s, "HazardActive", hazard_ ? 1 : 0);
    can::set(s, "LowBeamOn", low ? 1 : 0);
    can::seal(s, counter_++);
    hal_.can_transmit(s);
  }
}

}  // namespace bcm::app
