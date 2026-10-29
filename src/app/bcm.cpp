#include "bcm/app/bcm.hpp"

#include "bcm/can/bcm_db.hpp"

namespace bcm::app {

using hal::Din;
using hal::Dout;

Bcm::Bcm(hal::IHal& hal) : Bcm(hal, Config{}) {}

Bcm::Bcm(hal::IHal& hal, const Config& cfg)
    : hal_(hal),
      cfg_(cfg),
      canif_(hal),
      esc_(can::message(can::id::kEscVehicleSpeed)),
      ecm_(can::message(can::id::kEcmEngineStatus)),
      acu_(can::message(can::id::kAcuCrashStatus)),
      rls_(can::message(can::id::kRlsRainLight)),
      rke_(can::message(can::id::kRkeCommand)),
      ign_acc_(cfg.ign_debounce_ms),
      ign_run_(cfg.ign_debounce_ms),
      ign_start_(cfg.ign_debounce_ms),
      stalk_l_(cfg.debounce_ms),
      stalk_r_(cfg.debounce_ms),
      hl_auto_(cfg.debounce_ms),
      hl_on_(cfg.debounce_ms),
      hb_sw_(cfg.debounce_ms),
      wip_auto_(cfg.debounce_ms),
      wip_low_(cfg.debounce_ms),
      wip_high_(cfg.debounce_ms),
      door_ajar_(cfg.debounce_ms),
      hazard_sw_(cfg.debounce_ms, cfg.momentary_stuck_ms),
      lock_sw_(cfg.debounce_ms, cfg.momentary_stuck_ms),
      unlock_sw_(cfg.debounce_ms, cfg.momentary_stuck_ms),
      washer_sw_(cfg.debounce_ms, 30000) {}

void Bcm::read_inputs(uint32_t now) {
  ign_acc_.update(hal_.read(Din::IgnAcc), now);
  ign_run_.update(hal_.read(Din::IgnRun), now);
  ign_start_.update(hal_.read(Din::IgnStart), now);
  stalk_l_.update(hal_.read(Din::TurnLeft), now);
  stalk_r_.update(hal_.read(Din::TurnRight), now);
  hl_auto_.update(hal_.read(Din::HeadlampAuto), now);
  hl_on_.update(hal_.read(Din::HeadlampOn), now);
  hb_sw_.update(hal_.read(Din::HighBeamSw), now);
  wip_auto_.update(hal_.read(Din::WiperAuto), now);
  wip_low_.update(hal_.read(Din::WiperLow), now);
  wip_high_.update(hal_.read(Din::WiperHigh), now);
  door_ajar_.update(hal_.read(Din::DriverDoorAjar), now);
  hazard_sw_.update(hal_.read(Din::HazardSw), now);
  lock_sw_.update(hal_.read(Din::LockSw), now);
  unlock_sw_.update(hal_.read(Din::UnlockSw), now);
  washer_sw_.update(hal_.read(Din::WasherSw), now);
}

void Bcm::process_can(uint32_t now) {
  can_.crash_event = false;
  can_.rke_lock = can_.rke_unlock = false;
  can::Frame f;
  while (canif_.receive(f)) {
    switch (f.id) {
      case can::id::kEscVehicleSpeed:
        if (esc_.on_frame(f, now)) {
          can_.speed_kph = can::get(f, "VehicleSpeed");
          can_.speed_valid = can::get(f, "SpeedValid") > 0.5;
        }
        break;
      case can::id::kEcmEngineStatus:
        if (ecm_.on_frame(f, now)) can_.engine_running = can::get(f, "EngineRunning") > 0.5;
        break;
      case can::id::kAcuCrashStatus:
        if (acu_.on_frame(f, now)) {
          const bool crash = can::get(f, "CrashDetected") > 0.5;
          crash_frames_ = crash ? static_cast<uint8_t>(crash_frames_ < 255 ? crash_frames_ + 1 : 255) : 0;
          if (crash_frames_ >= cfg_.crash_confirm_frames && !crash_latched_) {
            crash_latched_ = true;
            can_.crash_event = true;
          }
          if (!crash) crash_latched_ = false;
        } else {
          crash_frames_ = 0;
        }
        break;
      case can::id::kRlsRainLight:
        if (rls_.on_frame(f, now)) {
          can_.rain_pct = can::get(f, "RainIntensity");
          can_.ambient_lux = can::get(f, "AmbientLight");
          can_.rls_valid = can::get(f, "SensorFault") < 0.5;
        }
        break;
      case can::id::kRkeCommand:
        if (rke_.on_frame(f, now)) {
          can_.rke_lock = can::get(f, "RkeLockReq") > 0.5;
          can_.rke_unlock = can::get(f, "RkeUnlockReq") > 0.5;
        }
        break;
      default:
        break;
    }
  }
  esc_.update(now);
  ecm_.update(now);
  acu_.update(now);
  rls_.update(now);
}

void Bcm::update_dtcs() {
  using can::RxStatus;
  const bool bus_off = canif_.bus_off_active();
  dtc_.set(Dtc::CanBusOff, bus_off);
  // message timeouts caused by our own bus-off are not reported separately
  dtc_.set(Dtc::EscTimeout, !bus_off && esc_.status() == RxStatus::Timeout);
  dtc_.set(Dtc::EcmTimeout, !bus_off && ecm_.status() == RxStatus::Timeout);
  dtc_.set(Dtc::AcuTimeout, !bus_off && acu_.status() == RxStatus::Timeout);
  dtc_.set(Dtc::RlsTimeout, !bus_off && rls_.status() == RxStatus::Timeout);
  dtc_.set(Dtc::E2eError, esc_.status() == RxStatus::Invalid || ecm_.status() == RxStatus::Invalid ||
                              acu_.status() == RxStatus::Invalid || rls_.status() == RxStatus::Invalid);
  dtc_.set(Dtc::HazardSwStuck, hazard_sw_.stuck());
  dtc_.set(Dtc::LockSwStuck, lock_sw_.stuck());
  dtc_.set(Dtc::UnlockSwStuck, unlock_sw_.stuck());
  dtc_.set(Dtc::WasherSwStuck, washer_sw_.stuck());
  dtc_.set(Dtc::WiperStall, wipers_.stalled());
  dtc_.set(Dtc::CrankTimeout, power_.crank_timeout());
  dtc_.set(Dtc::LockThermal, locks_.thermal_limited());
  dtc_.set(Dtc::RainSensorFault, rls_.usable() && !can_.rls_valid);
  dtc_.set(Dtc::IgnitionImplausible, power_.implausible());
}

void Bcm::send_status(uint32_t now) {
  if (status_sent_once_ && now - last_status_ms_ < cfg_.status_period_ms) return;
  status_sent_once_ = true;
  last_status_ms_ = now;
  can::Frame f = can::make_frame(can::id::kBcmStatus);
  can::set(f, "PowerMode", static_cast<double>(power_.mode()));
  can::set(f, "LockState", locks_.locked() ? 1 : 0);
  can::set(f, "TurnLeftActive", turn_.request() == TurnSignalSm::Request::Left ? 1 : 0);
  can::set(f, "TurnRightActive", turn_.request() == TurnSignalSm::Request::Right ? 1 : 0);
  can::set(f, "HazardActive", turn_.hazard_active() ? 1 : 0);
  can::set(f, "LowBeamOn", lighting_.low_beam() ? 1 : 0);
  can::set(f, "HighBeamOn", lighting_.high_beam() ? 1 : 0);
  can::set(f, "WiperMode", wipers_.status_code());
  can::set(f, "FaultFlags", dtc_.active_mask());
  can::seal(f, tx_counter_);
  tx_counter_ = static_cast<uint8_t>((tx_counter_ + 1) & 0x0F);
  canif_.send(f);
}

void Bcm::step() {
  const uint32_t now = hal_.tick_ms();
  canif_.update(now);
  read_inputs(now);
  process_can(now);

  PowerModeSm::Inputs pin;
  pin.ign_acc = ign_acc_.value();
  pin.ign_run = ign_run_.value();
  pin.ign_start = ign_start_.value();
  pin.engine_running = can_.engine_running;
  pin.engine_status_valid = ecm_.usable();
  power_.step(pin, now);
  const PowerMode mode = power_.mode();

  TurnSignalSm::Inputs tin;
  tin.stalk_left = stalk_l_.value();
  tin.stalk_right = stalk_r_.value();
  tin.hazard_pressed_edge = hazard_sw_.pressed_edge();
  tin.crash_event = can_.crash_event;
  tin.mode = mode;
  turn_.step(tin, now);

  WiperSm::Inputs win;
  if (wip_high_.value()) {
    win.sw = WiperSwitch::High;
  } else if (wip_low_.value()) {
    win.sw = WiperSwitch::Low;
  } else if (wip_auto_.value()) {
    win.sw = WiperSwitch::Auto;
  }
  win.washer = washer_sw_.pressed();
  win.rain_pct = can_.rain_pct;
  win.rain_valid = rls_.usable() && can_.rls_valid;
  win.park = hal_.read(Din::WiperPark);
  win.mode = mode;
  wipers_.step(win, now);

  LightingSm::Inputs lin;
  lin.sw_auto = hl_auto_.value();
  lin.sw_on = hl_on_.value();
  lin.high_beam_sw = hb_sw_.value();
  lin.ambient_lux = can_.ambient_lux;
  lin.ambient_valid = rls_.usable() && can_.rls_valid;
  lin.wipers_continuous = wipers_.continuous();
  lin.mode = mode;
  lighting_.step(lin, now);

  DoorLockSm::Inputs din;
  din.lock_sw_edge = lock_sw_.pressed_edge();
  din.unlock_sw_edge = unlock_sw_.pressed_edge();
  din.rke_lock = can_.rke_lock;
  din.rke_unlock = can_.rke_unlock;
  din.speed_kph = can_.speed_kph;
  din.speed_valid = esc_.usable() && can_.speed_valid;
  din.crash_event = can_.crash_event;
  din.door_ajar = door_ajar_.value();
  din.mode = mode;
  locks_.step(din, now);

  const auto po = power_.outputs();
  hal_.write(Dout::AccRelay, po.acc_relay);
  hal_.write(Dout::RunRelay, po.run_relay);
  hal_.write(Dout::StarterRelay, po.starter);
  hal_.write(Dout::TurnLeftLamp, turn_.left_lamp());
  hal_.write(Dout::TurnRightLamp, turn_.right_lamp());
  hal_.write(Dout::LowBeam, lighting_.low_beam());
  hal_.write(Dout::HighBeam, lighting_.high_beam());
  hal_.write(Dout::WiperLow, wipers_.motor_low());
  hal_.write(Dout::WiperHigh, wipers_.motor_high());
  hal_.write(Dout::WasherPump, wipers_.pump());
  hal_.write(Dout::LockMotor, locks_.lock_motor());
  hal_.write(Dout::UnlockMotor, locks_.unlock_motor());

  update_dtcs();
  send_status(now);
  hal_.watchdog_kick();
}

}  // namespace bcm::app
