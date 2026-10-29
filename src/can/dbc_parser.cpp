#include "bcm/can/dbc_parser.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace bcm::can {

const MessageDef* DbcFile::find(uint32_t msg_id) const {
  for (const auto& m : messages) {
    if (m.id == msg_id) return &m;
  }
  return nullptr;
}

DbcFile parse_dbc(const std::string& text) {
  DbcFile out;
  static const std::regex bo_re(R"(^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+))");
  static const std::regex sg_re(
      R"(^SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)\s*\[\s*([-+0-9.eE]+)\s*\|\s*([-+0-9.eE]+)\s*\]\s*\"([^\"]*)\")");
  static const std::regex ba_re(R"(^BA_\s+\"(\w+)\"\s+BO_\s+(\d+)\s+(\d+)\s*;)");
  static const std::regex bu_re(R"(^BU_\s*:(.*)$)");

  std::istringstream in(text);
  std::string line;
  MessageDef* current = nullptr;
  int lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    // trim leading whitespace
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos) {
      current = nullptr;
      continue;
    }
    line = line.substr(first);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::smatch m;
    if (std::regex_search(line, m, bo_re)) {
      MessageDef msg;
      msg.id = static_cast<uint32_t>(std::stoul(m[1]));
      msg.name = m[2];
      msg.dlc = static_cast<uint8_t>(std::stoi(m[3]));
      msg.sender = m[4];
      out.messages.push_back(msg);
      current = &out.messages.back();
    } else if (line.rfind("SG_", 0) == 0) {
      if (!std::regex_search(line, m, sg_re) || current == nullptr) {
        throw std::runtime_error("bad SG_ line " + std::to_string(lineno));
      }
      SignalDef s;
      s.name = m[1];
      s.start_bit = static_cast<uint16_t>(std::stoi(m[2]));
      s.length = static_cast<uint8_t>(std::stoi(m[3]));
      s.order = (m[4] == "1") ? ByteOrder::Intel : ByteOrder::Motorola;
      s.is_signed = (m[5] == "-");
      s.factor = std::stod(m[6]);
      s.offset = std::stod(m[7]);
      s.min = std::stod(m[8]);
      s.max = std::stod(m[9]);
      s.unit = m[10];
      current->signals.push_back(s);
    } else if (std::regex_search(line, m, ba_re)) {
      const uint32_t msg_id = static_cast<uint32_t>(std::stoul(m[2]));
      const uint32_t value = static_cast<uint32_t>(std::stoul(m[3]));
      for (auto& msg : out.messages) {
        if (msg.id != msg_id) continue;
        if (m[1] == "GenMsgCycleTime") msg.cycle_ms = value;
        if (m[1] == "RxTimeoutMs") msg.timeout_ms = value;
      }
    } else if (std::regex_search(line, m, bu_re)) {
      std::istringstream ns(m[1]);
      std::string n;
      while (ns >> n) out.nodes.push_back(n);
    }
  }
  return out;
}

DbcFile load_dbc(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_dbc(ss.str());
}

}  // namespace bcm::can
