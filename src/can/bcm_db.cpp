#include "bcm/can/bcm_db.hpp"

#include <stdexcept>

namespace bcm::can {

namespace {

SignalDef intel(const char* name, uint16_t start, uint8_t len, double factor, double offset,
                double mn, double mx, const char* unit = "", bool sgn = false) {
  SignalDef s;
  s.name = name;
  s.start_bit = start;
  s.length = len;
  s.order = ByteOrder::Intel;
  s.is_signed = sgn;
  s.factor = factor;
  s.offset = offset;
  s.min = mn;
  s.max = mx;
  s.unit = unit;
  return s;
}

SignalDef motorola(const char* name, uint16_t start, uint8_t len, double factor, double offset,
                   double mn, double mx, const char* unit = "") {
  SignalDef s = intel(name, start, len, factor, offset, mn, mx, unit);
  s.order = ByteOrder::Motorola;
  return s;
}

void add_e2e(MessageDef& m) {
  m.signals.push_back(intel("AliveCounter", 48, 4, 1, 0, 0, 15));
  m.signals.push_back(intel("Checksum", 56, 8, 1, 0, 0, 255));
}

std::vector<MessageDef> build() {
  std::vector<MessageDef> db;

  MessageDef esc{id::kEscVehicleSpeed, "ESC_VehicleSpeed", 8, "ESC", 20, 100, {}};
  esc.signals.push_back(intel("VehicleSpeed", 0, 16, 0.01, 0, 0, 655.35, "km/h"));
  esc.signals.push_back(intel("SpeedValid", 16, 1, 1, 0, 0, 1));
  add_e2e(esc);
  db.push_back(esc);

  MessageDef ecm{id::kEcmEngineStatus, "ECM_EngineStatus", 8, "ECM", 100, 500, {}};
  ecm.signals.push_back(intel("EngineRunning", 0, 1, 1, 0, 0, 1));
  ecm.signals.push_back(motorola("EngineSpeed", 15, 16, 0.25, 0, 0, 16383.75, "rpm"));
  ecm.signals.push_back(intel("CoolantTemp", 24, 8, 1, 0, -40, 127, "degC", true));
  add_e2e(ecm);
  db.push_back(ecm);

  MessageDef acu{id::kAcuCrashStatus, "ACU_CrashStatus", 8, "ACU", 50, 250, {}};
  acu.signals.push_back(intel("CrashDetected", 0, 1, 1, 0, 0, 1));
  acu.signals.push_back(intel("CrashSeverity", 1, 3, 1, 0, 0, 7));
  add_e2e(acu);
  db.push_back(acu);

  MessageDef rls{id::kRlsRainLight, "RLS_RainLight", 8, "RLS", 100, 500, {}};
  rls.signals.push_back(intel("RainIntensity", 0, 8, 0.5, 0, 0, 100, "%"));
  rls.signals.push_back(intel("AmbientLight", 8, 16, 1, 0, 0, 65535, "lux"));
  rls.signals.push_back(intel("SensorFault", 24, 1, 1, 0, 0, 1));
  add_e2e(rls);
  db.push_back(rls);

  MessageDef rke{id::kRkeCommand, "RKE_Command", 8, "RKE", 0, 0, {}};
  rke.signals.push_back(intel("RkeLockReq", 0, 1, 1, 0, 0, 1));
  rke.signals.push_back(intel("RkeUnlockReq", 1, 1, 1, 0, 0, 1));
  add_e2e(rke);
  db.push_back(rke);

  MessageDef st{id::kBcmStatus, "BCM_Status", 8, "BCM", 100, 0, {}};
  st.signals.push_back(intel("PowerMode", 0, 2, 1, 0, 0, 3));
  st.signals.push_back(intel("LockState", 2, 1, 1, 0, 0, 1));
  st.signals.push_back(intel("TurnLeftActive", 3, 1, 1, 0, 0, 1));
  st.signals.push_back(intel("TurnRightActive", 4, 1, 1, 0, 0, 1));
  st.signals.push_back(intel("HazardActive", 5, 1, 1, 0, 0, 1));
  st.signals.push_back(intel("LowBeamOn", 6, 1, 1, 0, 0, 1));
  st.signals.push_back(intel("HighBeamOn", 7, 1, 1, 0, 0, 1));
  st.signals.push_back(intel("WiperMode", 8, 3, 1, 0, 0, 7));
  st.signals.push_back(intel("FaultFlags", 16, 16, 1, 0, 0, 65535));
  add_e2e(st);
  db.push_back(st);

  return db;
}

}  // namespace

const std::vector<MessageDef>& database() {
  static const std::vector<MessageDef> db = build();
  return db;
}

const MessageDef& message(uint32_t msg_id) {
  for (const auto& m : database()) {
    if (m.id == msg_id) return m;
  }
  throw std::out_of_range("unknown CAN id");
}

double get(const Frame& f, const char* signal) {
  const SignalDef* s = message(f.id).find(signal);
  if (!s) throw std::out_of_range(signal);
  return decode(f, *s);
}

void set(Frame& f, const char* signal, double value) {
  const SignalDef* s = message(f.id).find(signal);
  if (!s) throw std::out_of_range(signal);
  encode(f, *s, value);
}

Frame make_frame(uint32_t msg_id) {
  Frame f;
  f.id = msg_id;
  f.dlc = message(msg_id).dlc;
  return f;
}

}  // namespace bcm::can
