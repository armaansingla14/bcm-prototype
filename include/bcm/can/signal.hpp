#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "bcm/can/frame.hpp"

namespace bcm::can {

enum class ByteOrder : uint8_t { Intel, Motorola };

/// One DBC signal definition (SG_ line).
struct SignalDef {
  std::string name;
  uint16_t start_bit = 0;  ///< DBC start bit (LSB for Intel, MSB for Motorola)
  uint8_t length = 1;
  ByteOrder order = ByteOrder::Intel;
  bool is_signed = false;
  double factor = 1.0;
  double offset = 0.0;
  double min = 0.0;
  double max = 0.0;
  std::string unit;
};

/// One DBC message definition (BO_ line plus cycle/timeout attributes).
struct MessageDef {
  uint32_t id = 0;
  std::string name;
  uint8_t dlc = 8;
  std::string sender;
  uint32_t cycle_ms = 0;    ///< 0 = event driven
  uint32_t timeout_ms = 0;  ///< 0 = not supervised
  std::vector<SignalDef> signals;

  const SignalDef* find(const std::string& sig) const;
};

/// Raw (unscaled) bit-field access following DBC bit numbering.
uint64_t extract_raw(const uint8_t* data, const SignalDef& s);
void insert_raw(uint8_t* data, const SignalDef& s, uint64_t raw);

/// Physical value access: phys = raw * factor + offset, clamped to [min,max]
/// on encode when min < max.
double decode(const Frame& f, const SignalDef& s);
void encode(Frame& f, const SignalDef& s, double phys);

/// E2E protection used on every supervised message in bcm.dbc:
/// byte 6 low nibble = alive counter, byte 7 = checksum.
uint8_t compute_checksum(const Frame& f);
void seal(Frame& f, uint8_t alive_counter);  ///< writes counter + checksum
bool checksum_ok(const Frame& f);
uint8_t alive_counter(const Frame& f);

}  // namespace bcm::can
