#pragma once
#include <cstdint>

#include "bcm/can/frame.hpp"

namespace bcm::hal {

/// Logical digital inputs. Active-high at this layer; pin polarity and the
/// physical port/pin mapping live in the backend (see stm32_hal.cpp).
enum class Din : uint8_t {
  IgnAcc,
  IgnRun,
  IgnStart,
  TurnLeft,
  TurnRight,
  HazardSw,
  HeadlampAuto,
  HeadlampOn,
  HighBeamSw,
  WiperAuto,
  WiperLow,
  WiperHigh,
  WasherSw,
  LockSw,
  UnlockSw,
  DriverDoorAjar,
  WiperPark,  ///< park cam switch on the wiper motor, closed = parked
  Count
};

/// Logical digital outputs (high-side driver / relay enables).
enum class Dout : uint8_t {
  TurnLeftLamp,
  TurnRightLamp,
  LowBeam,
  HighBeam,
  WiperLow,
  WiperHigh,
  WasherPump,
  LockMotor,
  UnlockMotor,
  AccRelay,
  RunRelay,
  StarterRelay,
  Count
};

constexpr int kNumDin = static_cast<int>(Din::Count);
constexpr int kNumDout = static_cast<int>(Dout::Count);

const char* to_string(Din d);
const char* to_string(Dout d);

/// bxCAN controller error state as reported by the ESR register.
enum class CanState : uint8_t { Active, ErrorPassive, BusOff };

/// Hardware abstraction used by the application. Mirrors the subset of the
/// STM32 HAL the BCM needs (GPIO, SysTick, bxCAN mailboxes/FIFO, IWDG).
class IHal {
 public:
  virtual ~IHal() = default;
  virtual uint32_t tick_ms() const = 0;
  virtual bool read(Din pin) const = 0;
  virtual void write(Dout pin, bool on) = 0;
  /// Queue a frame in a free TX mailbox. false = no mailbox / bus-off.
  virtual bool can_transmit(const can::Frame& f) = 0;
  /// Pop one frame from RX FIFO0. false = FIFO empty.
  virtual bool can_receive(can::Frame& f) = 0;
  virtual CanState can_state() const = 0;
  /// Software-triggered bus-off recovery (re-enter normal mode; the
  /// controller then waits for 128 x 11 recessive bits).
  virtual void can_request_recovery() = 0;
  virtual void watchdog_kick() {}
};

}  // namespace bcm::hal
