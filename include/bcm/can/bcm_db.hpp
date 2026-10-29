#pragma once
#include <cstdint>
#include <vector>

#include "bcm/can/signal.hpp"

namespace bcm::can {

/// Message IDs from dbc/bcm.dbc.
namespace id {
constexpr uint32_t kEscVehicleSpeed = 0x100;
constexpr uint32_t kEcmEngineStatus = 0x120;
constexpr uint32_t kAcuCrashStatus = 0x200;
constexpr uint32_t kRlsRainLight = 0x210;
constexpr uint32_t kRkeCommand = 0x300;
constexpr uint32_t kBcmStatus = 0x400;
}  // namespace id

/// Compiled-in message database. It mirrors dbc/bcm.dbc; test_dbc parses the
/// DBC file and checks that both agree, so the two cannot silently drift.
const std::vector<MessageDef>& database();
const MessageDef& message(uint32_t id);

/// Convenience: encode/decode a named signal of a known message.
double get(const Frame& f, const char* signal);
void set(Frame& f, const char* signal, double value);
Frame make_frame(uint32_t id);

}  // namespace bcm::can
