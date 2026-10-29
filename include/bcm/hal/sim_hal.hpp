#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "bcm/hal/hal.hpp"

namespace bcm::hal {

/// Host simulation backend. Inputs are set by the test/plant model, outputs
/// are observed by it. Supports fault injection for stuck pins, receive
/// FIFO overflow and a persistent bus fault that drives the controller into
/// bus-off (recovery attempts fail while the fault is present).
class SimHal : public IHal {
 public:
  struct TxRecord {
    uint32_t t_ms;
    can::Frame frame;
  };

  SimHal() { inputs_.fill(false); outputs_.fill(false); }

  // --- IHal ---
  uint32_t tick_ms() const override { return now_; }
  bool read(Din pin) const override;
  void write(Dout pin, bool on) override { outputs_[static_cast<int>(pin)] = on; }
  bool can_transmit(const can::Frame& f) override;
  bool can_receive(can::Frame& f) override;
  CanState can_state() const override { return state_; }
  void can_request_recovery() override;
  void watchdog_kick() override { ++wdg_kicks_; }

  // --- simulation side ---
  void set_time(uint32_t t) { now_ = t; }
  void advance(uint32_t dt) { now_ += dt; }
  void set_input(Din pin, bool v) { inputs_[static_cast<int>(pin)] = v; }
  bool input(Din pin) const { return inputs_[static_cast<int>(pin)]; }
  void set_stuck(Din pin, std::optional<bool> v) { stuck_[static_cast<int>(pin)] = v; }
  void clear_stuck() { for (auto& s : stuck_) s.reset(); }
  bool output(Dout pin) const { return outputs_[static_cast<int>(pin)]; }

  /// Frame arriving from the bus. Dropped while bus-off; counted as overrun
  /// when the FIFO is full.
  bool deliver(const can::Frame& f);
  void set_bus_fault(bool active);
  bool bus_fault() const { return bus_fault_; }

  const std::vector<TxRecord>& tx_log() const { return tx_log_; }
  void clear_tx_log() { tx_log_.clear(); }
  uint32_t recovery_requests() const { return recovery_requests_; }
  uint32_t rx_overruns() const { return rx_overruns_; }
  uint32_t rx_dropped_bus_off() const { return rx_dropped_bus_off_; }
  uint32_t watchdog_kicks() const { return wdg_kicks_; }
  size_t rx_fifo_depth = 64;

 private:
  uint32_t now_ = 0;
  std::array<bool, kNumDin> inputs_{};
  std::array<std::optional<bool>, kNumDin> stuck_{};
  std::array<bool, kNumDout> outputs_{};
  std::deque<can::Frame> rx_;
  std::vector<TxRecord> tx_log_;
  CanState state_ = CanState::Active;
  bool bus_fault_ = false;
  uint32_t recovery_requests_ = 0;
  uint32_t rx_overruns_ = 0;
  uint32_t rx_dropped_bus_off_ = 0;
  uint32_t wdg_kicks_ = 0;
};

}  // namespace bcm::hal
