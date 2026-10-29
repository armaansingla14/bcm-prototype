#include "bcm/can/bcm_db.hpp"
#include "bcm/can/dbc_parser.hpp"
#include "testfw.hpp"

using namespace bcm::can;

static const std::string kDbcPath = std::string(BCM_SOURCE_DIR) + "/dbc/bcm.dbc";

TEST(Dbc, LoadsFile) {
  const DbcFile dbc = load_dbc(kDbcPath);
  CHECK_EQ(dbc.messages.size(), database().size());
  CHECK_EQ(dbc.nodes.size(), 6u);
}

TEST(Dbc, CompiledDatabaseMatchesFile) {
  const DbcFile dbc = load_dbc(kDbcPath);
  for (const auto& m : database()) {
    const MessageDef* d = dbc.find(m.id);
    REQUIRE(d != nullptr);
    CHECK_EQ(d->name, m.name);
    CHECK_EQ(d->dlc, m.dlc);
    CHECK_EQ(d->sender, m.sender);
    CHECK_EQ(d->cycle_ms, m.cycle_ms);
    CHECK_EQ(d->timeout_ms, m.timeout_ms);
    REQUIRE(d->signals.size() == m.signals.size());
    for (size_t i = 0; i < m.signals.size(); ++i) {
      const SignalDef& a = d->signals[i];
      const SignalDef& b = m.signals[i];
      CHECK_EQ(a.name, b.name);
      CHECK_EQ(a.start_bit, b.start_bit);
      CHECK_EQ(a.length, b.length);
      CHECK(a.order == b.order);
      CHECK_EQ(a.is_signed, b.is_signed);
      CHECK_NEAR(a.factor, b.factor, 1e-12);
      CHECK_NEAR(a.offset, b.offset, 1e-12);
      CHECK_NEAR(a.min, b.min, 1e-9);
      CHECK_NEAR(a.max, b.max, 1e-9);
    }
  }
}

TEST(Dbc, ParsesMotorolaAndSigned) {
  const DbcFile dbc = load_dbc(kDbcPath);
  const MessageDef* ecm = dbc.find(0x120);
  REQUIRE(ecm != nullptr);
  const SignalDef* rpm = ecm->find("EngineSpeed");
  REQUIRE(rpm != nullptr);
  CHECK(rpm->order == ByteOrder::Motorola);
  const SignalDef* temp = ecm->find("CoolantTemp");
  REQUIRE(temp != nullptr);
  CHECK(temp->is_signed);
  CHECK_EQ(temp->unit, std::string("degC"));
}

TEST(Dbc, InlineText) {
  const char* txt =
      "BU_: A B\n"
      "BO_ 1234 Foo: 4 A\n"
      " SG_ Bar : 3|10@1+ (0.1,-5) [-5|97.3] \"V\" B\n"
      "\n"
      "BA_ \"GenMsgCycleTime\" BO_ 1234 50;\n";
  const DbcFile d = parse_dbc(txt);
  REQUIRE(d.messages.size() == 1);
  CHECK_EQ(d.messages[0].id, 1234u);
  CHECK_EQ(d.messages[0].dlc, 4);
  CHECK_EQ(d.messages[0].cycle_ms, 50u);
  REQUIRE(d.messages[0].signals.size() == 1);
  CHECK_NEAR(d.messages[0].signals[0].offset, -5.0, 1e-12);
  CHECK_EQ(d.messages[0].signals[0].unit, std::string("V"));
}

TEST(Dbc, BadSignalLineThrows) {
  bool threw = false;
  try {
    parse_dbc("BO_ 1 X: 8 A\n SG_ broken line\n");
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
}

TEST(Dbc, SignalOutsideMessageThrows) {
  bool threw = false;
  try {
    parse_dbc(" SG_ S : 0|1@1+ (1,0) [0|1] \"\" X\n");
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
}

TEST(Dbc, MissingFileThrows) {
  bool threw = false;
  try {
    load_dbc("/nonexistent/file.dbc");
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
}

TEST(Dbc, SignalsFitInPayload) {
  for (const auto& m : database()) {
    for (const auto& s : m.signals) {
      if (s.order == ByteOrder::Intel) {
        CHECK(s.start_bit + s.length <= m.dlc * 8);
      } else {
        CHECK(s.start_bit < m.dlc * 8);
      }
    }
  }
}

TEST(Dbc, NoOverlappingSignals) {
  for (const auto& m : database()) {
    uint8_t used[8] = {0};
    for (const auto& s : m.signals) {
      Frame f;
      insert_raw(f.data.data(), s, (s.length >= 64) ? ~0ULL : ((1ULL << s.length) - 1));
      for (int i = 0; i < 8; ++i) {
        CHECK_EQ(used[i] & f.data[i], 0);
        used[i] |= f.data[i];
      }
    }
  }
}

TEST(Dbc, SupervisedMessagesHaveE2e) {
  for (const auto& m : database()) {
    CHECK(m.find("AliveCounter") != nullptr);
    CHECK(m.find("Checksum") != nullptr);
    if (m.sender != "BCM" && m.cycle_ms > 0) CHECK(m.timeout_ms >= 3 * m.cycle_ms);
  }
}

TEST_MAIN()
