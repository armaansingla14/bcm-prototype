// Closed-loop test rig: BCM + simulated HAL + vehicle plant.
#pragma once
#include "bcm/app/bcm.hpp"
#include "bcm/app/naive_bcm.hpp"
#include "bcm/hal/sim_hal.hpp"
#include "bcm/sim/vehicle_sim.hpp"

struct Rig {
  bcm::hal::SimHal hal;
  bcm::sim::VehicleSim sim{hal, 42};
  bcm::app::Bcm bcm{hal};
  uint32_t t = 0;

  void tick() {
    sim.pre_step(t);
    bcm.step();
    sim.post_step(t);
    t += bcm::app::kTickMs;
  }
  void run(uint32_t ms) {
    const uint32_t end = t + ms;
    while (t < end) tick();
  }
  bool out(bcm::hal::Dout d) const { return hal.output(d); }
  void press(bcm::hal::Din pin, uint32_t hold_ms = 150) {
    sim.set_switch(pin, true);
    run(hold_ms);
    sim.set_switch(pin, false);
  }
  /// Key on to RUN with the engine started.
  void start_engine() {
    sim.set_key(bcm::sim::KeyPos::Acc);
    run(200);
    sim.set_key(bcm::sim::KeyPos::Run);
    run(200);
    sim.set_key(bcm::sim::KeyPos::Start);
    run(1200);
    sim.set_key(bcm::sim::KeyPos::Run);
    run(200);
  }
};
