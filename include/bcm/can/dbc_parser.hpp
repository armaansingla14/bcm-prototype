#pragma once
#include <string>
#include <vector>

#include "bcm/can/signal.hpp"

namespace bcm::can {

/// Minimal DBC reader: BO_, SG_ and the GenMsgCycleTime / RxTimeoutMs
/// message attributes. Enough to validate the compiled-in database and to
/// load the same file in host tools. Multiplexed signals are not supported.
struct DbcFile {
  std::vector<std::string> nodes;
  std::vector<MessageDef> messages;
  const MessageDef* find(uint32_t id) const;
};

DbcFile parse_dbc(const std::string& text);
DbcFile load_dbc(const std::string& path);

}  // namespace bcm::can
