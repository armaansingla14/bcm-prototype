#include "bcm/can/bcm_db.hpp"
#include "bcm/can/rx_monitor.hpp"
#include "testfw.hpp"

using namespace bcm::can;

namespace {
Frame esc(uint8_t ctr, double speed = 50.0) {
  Frame f = make_frame(id::kEscVehicleSpeed);
  set(f, "VehicleSpeed", speed);
  set(f, "SpeedValid", 1);
  seal(f, ctr);
  return f;
}
}  // namespace

TEST(RxMonitor, StartsNeverReceived) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  CHECK(m.status() == RxStatus::NeverReceived);
  m.update(10000);
  CHECK(m.status() == RxStatus::NeverReceived);  // no timeout before first frame
}

TEST(RxMonitor, FirstValidFrameUsable) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  CHECK(m.on_frame(esc(0), 0));
  CHECK(m.usable());
}

TEST(RxMonitor, TimeoutAfterConfiguredPeriod) {
  RxMonitor m(message(id::kEscVehicleSpeed));  // 100 ms
  m.on_frame(esc(0), 0);
  m.update(100);
  CHECK(m.usable());
  m.update(110);
  CHECK(m.status() == RxStatus::Timeout);
  CHECK_EQ(m.timeouts(), 1u);
}

TEST(RxMonitor, PeriodicFramesNeverTimeOut) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  for (uint32_t t = 0, c = 0; t < 5000; t += 20, ++c) {
    CHECK(m.on_frame(esc(static_cast<uint8_t>(c & 15)), t));
    m.update(t + 10);
  }
  CHECK(m.usable());
  CHECK_EQ(m.error_frames(), 0u);
}

TEST(RxMonitor, RecoveryNeedsTwoGoodFrames) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  m.on_frame(esc(0), 0);
  m.update(500);
  REQUIRE(m.status() == RxStatus::Timeout);
  CHECK(!m.on_frame(esc(5), 520));
  CHECK(m.on_frame(esc(6), 540));
  CHECK(m.usable());
}

TEST(RxMonitor, BadChecksumRejected) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  Frame f = esc(0);
  f.data[0] ^= 0x10;
  CHECK(!m.on_frame(f, 0));
  CHECK_EQ(m.error_frames(), 1u);
}

TEST(RxMonitor, ThreeBadFramesInvalid) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  m.on_frame(esc(0), 0);
  for (int i = 1; i <= 3; ++i) {
    Frame f = esc(static_cast<uint8_t>(i));
    f.data[1] ^= 0x01;
    m.on_frame(f, 20u * i);
  }
  CHECK(m.status() == RxStatus::Invalid);
  CHECK(!m.usable());
}

TEST(RxMonitor, SingleBadFrameToleratedButNotUsed) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  m.on_frame(esc(0), 0);
  Frame f = esc(1);
  f.data[2] ^= 0x80;
  CHECK(!m.on_frame(f, 20));
  CHECK(m.usable());
  CHECK(m.on_frame(esc(2), 40));
}

TEST(RxMonitor, RepeatedCounterRejected) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  CHECK(m.on_frame(esc(3), 0));
  CHECK(!m.on_frame(esc(3), 20));
}

TEST(RxMonitor, FrozenCounterGoesInvalid) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  m.on_frame(esc(3), 0);
  for (int i = 1; i <= 4; ++i) m.on_frame(esc(3), 20u * i);
  CHECK(m.status() == RxStatus::Invalid);
}

TEST(RxMonitor, CounterJumpOfTwoAccepted) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  m.on_frame(esc(14), 0);
  CHECK(m.on_frame(esc(0), 40));  // one frame lost across wrap-around
}

TEST(RxMonitor, LargeCounterJumpRejectedThenResync) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  m.on_frame(esc(0), 0);
  CHECK(!m.on_frame(esc(8), 20));
  CHECK(m.on_frame(esc(9), 40));  // resynchronised on the new counter
}

TEST(RxMonitor, EventMessageAllowsCounterJumps) {
  RxMonitor m(message(id::kRkeCommand));
  Frame a = make_frame(id::kRkeCommand);
  seal(a, 1);
  Frame b = make_frame(id::kRkeCommand);
  seal(b, 9);
  CHECK(m.on_frame(a, 0));
  CHECK(m.on_frame(b, 60000));
  m.update(120000);
  CHECK(m.usable());  // event-driven: no timeout supervision
}

TEST(RxMonitor, ResetClearsState) {
  RxMonitor m(message(id::kEscVehicleSpeed));
  m.on_frame(esc(0), 0);
  m.reset();
  CHECK(m.status() == RxStatus::NeverReceived);
  CHECK(m.on_frame(esc(0), 10));  // counter history cleared too
}

TEST_MAIN()
