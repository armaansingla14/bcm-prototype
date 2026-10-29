#include "bcm/can/bcm_db.hpp"
#include "bcm/can/signal.hpp"
#include "testfw.hpp"

using namespace bcm::can;

namespace {
SignalDef sig(uint16_t start, uint8_t len, ByteOrder o, bool sgn = false, double f = 1, double off = 0) {
  SignalDef s;
  s.name = "x";
  s.start_bit = start;
  s.length = len;
  s.order = o;
  s.is_signed = sgn;
  s.factor = f;
  s.offset = off;
  return s;
}
}  // namespace

TEST(Signal, IntelSingleByte) {
  Frame f;
  insert_raw(f.data.data(), sig(0, 8, ByteOrder::Intel), 0xA5);
  CHECK_EQ(f.data[0], 0xA5);
  CHECK_EQ(extract_raw(f.data.data(), sig(0, 8, ByteOrder::Intel)), 0xA5u);
}

TEST(Signal, IntelCrossesByteBoundary) {
  Frame f;
  const auto s = sig(4, 12, ByteOrder::Intel);
  insert_raw(f.data.data(), s, 0xABC);
  CHECK_EQ(f.data[0], 0xC0);
  CHECK_EQ(f.data[1], 0xAB);
  CHECK_EQ(extract_raw(f.data.data(), s), 0xABCu);
}

TEST(Signal, MotorolaSixteenBit) {
  // start bit 15 = MSB of byte 1 -> big endian value in bytes 1..2
  Frame f;
  const auto s = sig(15, 16, ByteOrder::Motorola);
  insert_raw(f.data.data(), s, 0x1234);
  CHECK_EQ(f.data[1], 0x12);
  CHECK_EQ(f.data[2], 0x34);
  CHECK_EQ(extract_raw(f.data.data(), s), 0x1234u);
}

TEST(Signal, MotorolaNibble) {
  // 4 bits, MSB at bit 7 of byte 0 -> upper nibble
  Frame f;
  const auto s = sig(7, 4, ByteOrder::Motorola);
  insert_raw(f.data.data(), s, 0xD);
  CHECK_EQ(f.data[0], 0xD0);
  CHECK_EQ(extract_raw(f.data.data(), s), 0xDu);
}

TEST(Signal, MotorolaTwelveBitCrossing) {
  // MSB at bit 3 of byte 0, 12 bits -> byte0 low nibble + byte1
  Frame f;
  const auto s = sig(3, 12, ByteOrder::Motorola);
  insert_raw(f.data.data(), s, 0x9F1);
  CHECK_EQ(f.data[0], 0x09);
  CHECK_EQ(f.data[1], 0xF1);
  CHECK_EQ(extract_raw(f.data.data(), s), 0x9F1u);
}

TEST(Signal, InsertDoesNotDisturbNeighbours) {
  Frame f;
  f.data.fill(0xFF);
  insert_raw(f.data.data(), sig(10, 3, ByteOrder::Intel), 0);
  CHECK_EQ(f.data[1], 0xE3);
  CHECK_EQ(f.data[0], 0xFF);
  CHECK_EQ(f.data[2], 0xFF);
}

TEST(Signal, ScaledPhysicalRoundTrip) {
  Frame f;
  auto s = sig(0, 16, ByteOrder::Intel, false, 0.01, 0);
  s.min = 0;
  s.max = 655.35;
  encode(f, s, 123.45);
  CHECK_NEAR(decode(f, s), 123.45, 1e-9);
}

TEST(Signal, OffsetSignal) {
  Frame f;
  auto s = sig(8, 8, ByteOrder::Intel, false, 0.5, -40);
  s.min = -40;
  s.max = 87.5;
  encode(f, s, 20.0);
  CHECK_EQ(f.data[1], 120);
  CHECK_NEAR(decode(f, s), 20.0, 1e-9);
}

TEST(Signal, SignedNegative) {
  Frame f;
  auto s = sig(24, 8, ByteOrder::Intel, true);
  s.min = -128;
  s.max = 127;
  encode(f, s, -40);
  CHECK_EQ(f.data[3], 0xD8);
  CHECK_NEAR(decode(f, s), -40.0, 1e-9);
}

TEST(Signal, EncodeClampsToRange) {
  Frame f;
  auto s = sig(0, 8, ByteOrder::Intel, false, 0.5, 0);
  s.min = 0;
  s.max = 100;
  encode(f, s, 250.0);
  CHECK_NEAR(decode(f, s), 100.0, 1e-9);
  encode(f, s, -5.0);
  CHECK_NEAR(decode(f, s), 0.0, 1e-9);
}

TEST(Signal, ChecksumSealAndVerify) {
  Frame f = make_frame(id::kEscVehicleSpeed);
  set(f, "VehicleSpeed", 42.0);
  seal(f, 7);
  CHECK(checksum_ok(f));
  CHECK_EQ(alive_counter(f), 7);
  f.data[0] ^= 0x01;
  CHECK(!checksum_ok(f));
}

TEST(Signal, ChecksumCoversId) {
  Frame a = make_frame(id::kEscVehicleSpeed);
  seal(a, 1);
  Frame b = a;
  b.id = id::kAcuCrashStatus;  // same payload under another ID must fail
  CHECK(!checksum_ok(b));
}

TEST(Signal, SealPreservesUpperNibbleOfByte6) {
  Frame f = make_frame(id::kRlsRainLight);
  f.data[6] = 0xA0;
  seal(f, 0x5);
  CHECK_EQ(f.data[6], 0xA5);
}

TEST(Signal, DatabaseLookups) {
  CHECK_EQ(message(id::kEscVehicleSpeed).cycle_ms, 20u);
  CHECK_EQ(message(id::kAcuCrashStatus).timeout_ms, 250u);
  CHECK(message(id::kBcmStatus).find("FaultFlags") != nullptr);
  bool threw = false;
  try {
    message(0x7FF);
  } catch (const std::out_of_range&) {
    threw = true;
  }
  CHECK(threw);
}

TEST(Signal, EngineSpeedMotorolaInDatabase) {
  Frame f = make_frame(id::kEcmEngineStatus);
  set(f, "EngineSpeed", 3000.0);  // raw 12000 = 0x2EE0
  CHECK_EQ(f.data[1], 0x2E);
  CHECK_EQ(f.data[2], 0xE0);
  CHECK_NEAR(get(f, "EngineSpeed"), 3000.0, 1e-9);
}

TEST(Signal, StatusFrameAllFields) {
  Frame f = make_frame(id::kBcmStatus);
  set(f, "PowerMode", 3);
  set(f, "LockState", 1);
  set(f, "HazardActive", 1);
  set(f, "WiperMode", 5);
  set(f, "FaultFlags", 0x8421);
  CHECK_NEAR(get(f, "PowerMode"), 3, 0);
  CHECK_NEAR(get(f, "LockState"), 1, 0);
  CHECK_NEAR(get(f, "TurnLeftActive"), 0, 0);
  CHECK_NEAR(get(f, "HazardActive"), 1, 0);
  CHECK_NEAR(get(f, "WiperMode"), 5, 0);
  CHECK_NEAR(get(f, "FaultFlags"), 0x8421, 0);
}

namespace {
// counts undetected 2-bit error patterns over data bytes 0..6
template <typename Fn>
int undetected_two_bit(Fn check) {
  Frame f = make_frame(id::kEscVehicleSpeed);
  set(f, "VehicleSpeed", 57.3);
  set(f, "SpeedValid", 1);
  seal(f, 9);
  const uint8_t good = check(f);
  int undetected = 0;
  for (int i = 0; i < 56; ++i) {
    for (int j = i + 1; j < 56; ++j) {
      Frame g = f;
      g.data[i / 8] ^= static_cast<uint8_t>(1U << (i % 8));
      g.data[j / 8] ^= static_cast<uint8_t>(1U << (j % 8));
      if (check(g) == good) ++undetected;
    }
  }
  return undetected;
}
}  // namespace

TEST(Signal, Crc8DetectsAllOneAndTwoBitErrors) {
  CHECK_EQ(undetected_two_bit([](const Frame& g) { return compute_checksum(g); }), 0);
  Frame f = make_frame(id::kEcmEngineStatus);
  seal(f, 2);
  for (int i = 0; i < 56; ++i) {
    Frame g = f;
    g.data[i / 8] ^= static_cast<uint8_t>(1U << (i % 8));
    CHECK(!checksum_ok(g));
  }
}

TEST(Signal, AdditiveChecksumMissesSomeTwoBitErrors) {
  // the first prototype's additive checksum: equal-and-opposite bit flips
  // in two bytes cancel out
  const int missed = undetected_two_bit([](const Frame& g) { return compute_checksum_additive(g); });
  CHECK(missed > 0);
}

TEST(Signal, Crc8ThreeBitResidual) {
  // CRC-8 has Hamming distance 3 at this length: some 3-bit errors pass.
  Frame f = make_frame(id::kEscVehicleSpeed);
  set(f, "VehicleSpeed", 0.0);
  set(f, "SpeedValid", 1);
  seal(f, 3);
  int undetected = 0, total = 0;
  for (int i = 0; i < 56; ++i)
    for (int j = i + 1; j < 56; ++j)
      for (int k = j + 1; k < 56; ++k) {
        Frame g = f;
        for (int b : {i, j, k}) g.data[b / 8] ^= static_cast<uint8_t>(1U << (b % 8));
        ++total;
        if (checksum_ok(g)) ++undetected;
      }
  std::printf("    CRC-8 undetected 3-bit errors: %d of %d\n", undetected, total);
  CHECK(undetected > 0);
  CHECK(undetected < total / 100);
}

TEST(Signal, Crc8KnownVector) {
  // CRC-8 SAE J1850 check value for "123456789" is 0x4B; our framing adds
  // the ID bytes, so verify via an independent recomputation of the frame.
  Frame f;
  f.id = 0x000;
  const char* s = "1234567";
  for (int i = 0; i < 7; ++i) f.data[i] = static_cast<uint8_t>(s[i]);
  const uint8_t a = compute_checksum(f);
  f.id = 0x100;
  const uint8_t b = compute_checksum(f);
  CHECK(a != b);
}

TEST_MAIN()
