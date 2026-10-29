#include "bcm/can/signal.hpp"

#include <cmath>
#include <cstddef>

namespace bcm::can {

const SignalDef* MessageDef::find(const std::string& sig) const {
  for (const auto& s : signals) {
    if (s.name == sig) return &s;
  }
  return nullptr;
}

namespace {

inline bool get_bit(const uint8_t* d, unsigned pos) { return (d[pos / 8] >> (pos % 8)) & 1U; }

inline void set_bit(uint8_t* d, unsigned pos, bool v) {
  const uint8_t mask = static_cast<uint8_t>(1U << (pos % 8));
  if (v) {
    d[pos / 8] |= mask;
  } else {
    d[pos / 8] &= static_cast<uint8_t>(~mask);
  }
}

// Motorola (big-endian) signals in DBC: the start bit names the MSB, bits are
// walked towards lower significance; inside a byte that means bit index - 1,
// and when crossing a byte boundary we jump to bit 7 of the next byte.
inline unsigned motorola_next(unsigned pos) { return (pos % 8 == 0) ? pos + 15 : pos - 1; }

}  // namespace

uint64_t extract_raw(const uint8_t* data, const SignalDef& s) {
  uint64_t raw = 0;
  if (s.order == ByteOrder::Intel) {
    for (unsigned i = 0; i < s.length; ++i) {
      if (get_bit(data, s.start_bit + i)) raw |= (uint64_t{1} << i);
    }
  } else {
    unsigned pos = s.start_bit;
    for (unsigned i = 0; i < s.length; ++i) {
      raw = (raw << 1) | (get_bit(data, pos) ? 1U : 0U);
      pos = motorola_next(pos);
    }
  }
  return raw;
}

void insert_raw(uint8_t* data, const SignalDef& s, uint64_t raw) {
  if (s.order == ByteOrder::Intel) {
    for (unsigned i = 0; i < s.length; ++i) set_bit(data, s.start_bit + i, (raw >> i) & 1U);
  } else {
    unsigned pos = s.start_bit;
    for (int i = s.length - 1; i >= 0; --i) {
      set_bit(data, pos, (raw >> i) & 1U);
      pos = motorola_next(pos);
    }
  }
}

double decode(const Frame& f, const SignalDef& s) {
  uint64_t raw = extract_raw(f.data.data(), s);
  double v;
  if (s.is_signed && s.length < 64 && (raw >> (s.length - 1)) & 1U) {
    const int64_t sval = static_cast<int64_t>(raw) - (int64_t{1} << s.length);
    v = static_cast<double>(sval);
  } else {
    v = static_cast<double>(raw);
  }
  return v * s.factor + s.offset;
}

void encode(Frame& f, const SignalDef& s, double phys) {
  if (s.min < s.max) phys = std::fmin(std::fmax(phys, s.min), s.max);
  const double rawd = std::round((phys - s.offset) / s.factor);
  const uint64_t mask = (s.length >= 64) ? ~uint64_t{0} : ((uint64_t{1} << s.length) - 1);
  uint64_t raw;
  if (s.is_signed) {
    raw = static_cast<uint64_t>(static_cast<int64_t>(rawd)) & mask;
  } else {
    raw = static_cast<uint64_t>(rawd < 0 ? 0 : rawd) & mask;
  }
  insert_raw(f.data.data(), s, raw);
}

namespace {
// CRC-8 SAE J1850 (poly 0x1D, init 0xFF, final xor 0xFF), as used by the
// AUTOSAR E2E profile 1/2 family. Detects all 1-, 2- and 3-bit errors in a
// frame of this length, unlike a plain additive checksum.
uint8_t crc8_j1850(const uint8_t* p, size_t n, uint8_t crc) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x1DU) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}
}  // namespace

uint8_t compute_checksum(const Frame& f) {
  // the message ID is included (as a 2-byte data ID) so that a frame with a
  // valid CRC under another ID is rejected
  const uint8_t id_bytes[2] = {static_cast<uint8_t>(f.id & 0xFFU), static_cast<uint8_t>((f.id >> 8) & 0xFFU)};
  uint8_t crc = crc8_j1850(id_bytes, 2, 0xFF);
  crc = crc8_j1850(f.data.data(), 7, crc);
  return static_cast<uint8_t>(crc ^ 0xFFU);
}

uint8_t compute_checksum_additive(const Frame& f) {
  unsigned sum = (f.id & 0xFFU) + ((f.id >> 8) & 0xFFU);
  for (int i = 0; i < 7; ++i) sum += f.data[i];
  return static_cast<uint8_t>((sum & 0xFFU) ^ 0xFFU);
}

void seal(Frame& f, uint8_t counter) {
  f.data[6] = static_cast<uint8_t>((f.data[6] & 0xF0U) | (counter & 0x0FU));
  f.data[7] = compute_checksum(f);
}

bool checksum_ok(const Frame& f) { return f.dlc == 8 && f.data[7] == compute_checksum(f); }

uint8_t alive_counter(const Frame& f) { return f.data[6] & 0x0FU; }

}  // namespace bcm::can
