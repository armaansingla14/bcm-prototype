# bcm-prototype

Firmware prototype of an automotive body control module (BCM) in C++17. Lighting, turn indicators and hazards, central locking, front wipers and ignition power modes are each implemented as an explicit state machine. The code is structured like STM32 firmware: an application layer, a CAN message layer driven by a DBC file, and a hardware abstraction layer with two backends. One backend uses an STM32Cube-HAL-style interface (GPIO, SysTick, bxCAN, IWDG); the other is a host simulation used for testing.

Everything here runs on a host PC. The STM32 backend is compiled and unit-tested against stub HAL headers, but it has not been run on a microcontroller.

## Features

| Area | Behaviour |
|---|---|
| Power modes | OFF / ACC / RUN / CRANK decoded from three ignition contacts. CRANK is only entered from RUN, and not while the ECM reports the engine running. The starter is cut when the engine fires, when the key leaves START, or after a 10 s crank timeout (then inhibited until the key is released). ACC loads are shed while cranking. START without RUN is flagged as implausible. |
| Turn / hazard | Integrated 1.5 Hz flasher. A started flash is always completed and the off time is always respected, so switch chatter cannot cause flicker. Lane-change comfort: a short tap gives 3 flashes. Hazard is a latched toggle that works in every power mode. A crash forces hazard on immediately. |
| Lighting | Manual ON. AUTO uses ambient light with hysteresis (on below 1000 lx after 2 s, off above 2500 lx after 5 s). AUTO fails safe to ON when the light sensor is lost or reports a fault. Rain-light rule: continuous wiping for 5 s turns the lamps on. A 30 s exit delay applies after ignition off. High beam only works together with low beam. |
| Door locks | Fixed 200 ms actuator pulses. Auto-lock happens once per drive cycle at 15 km/h or more, using fresh, validated speed only. It re-arms when the vehicle is stationary with a door open. Crash unlock pulses twice and then inhibits locking for 10 s. Remote lock is refused while a door is ajar. Thermal protection allows at most 10 actuations per 30 s. |
| Wipers | OFF / AUTO / LOW / HIGH. AUTO maps rain intensity to intermittent (6 s down to 1.5 s), low or high, with hysteresis. Without a valid rain sensor it degrades to a fixed 4 s intermittent. Switching off always finishes the sweep to the park position. The washer runs the pump plus low speed, followed by 3 dry wipes. Stall detection stops the motor if the park switch shows no movement for 3 s. Wipers are suspended during CRANK. |
| CAN layer | DBC file (`dbc/bcm.dbc`) with Intel and Motorola, signed and scaled signals. The compiled-in database is checked against the DBC by a unit test. A minimal DBC parser is included. Every supervised message carries E2E protection: a 4-bit alive counter and a CRC-8 (SAE J1850) over the message ID and payload. Each receive monitor tracks timeout, counter and CRC failures (NeverReceived / Valid / Timeout / Invalid). The vehicle-speed signal also gets a plausibility filter (at most 5 km/h change per 20 ms frame). |
| Bus-off | Software-managed bxCAN bus-off recovery: 5 fast attempts 50 ms apart, then one attempt every 1 s. The attempt counter resets after 1 s of stable operation. Message-timeout DTCs are suppressed while the BCM itself is bus-off. |
| Diagnostics | 15 DTCs (timeouts, E2E, bus-off, stuck switches, wiper stall, crank timeout, lock thermal limit, ...) with active and latched state. They are broadcast in the `BCM_Status.FaultFlags` signal. |
| Input conditioning | Time-based debouncing (30 ms for switches, 50 ms for ignition). Momentary switches held for more than 10 s (washer: 30 s) are treated as stuck contacts and ignored until they actually release. A switch that is stuck at power-up never produces a press. |

## Layout

```
include/bcm/hal/      IHal interface, SimHal (host backend), Stm32Hal (STM32 HAL adapter)
include/bcm/can/      frame, signal encode/decode + CRC, DBC parser, message database, RX monitor, CanIf
include/bcm/app/      debounce, power_mode, turn_signal, lighting, door_locks, wipers, dtc, bcm, naive_bcm
include/bcm/sim/      vehicle/plant model, requirement monitor, scenario generator
src/...               implementations (same structure)
port/stm32/           stub HAL header/implementation for host builds, target main.cpp (not built)
dbc/bcm.dbc           CAN database (ESC, ECM, ACU, RLS, RKE -> BCM; BCM_Status)
tests/                166 test cases in 12 executables (small built-in test framework)
tools/scenario_runner.cpp  fault-scenario benchmark: state machine vs naive baseline
results/              scenario results (CSV + summary.md)
```

The application only talks to `bcm::hal::IHal`. On target, `Stm32Hal` maps the logical pins to GPIO ports and pins (switch inputs are active-low with pull-ups) and maps CAN to `HAL_CAN_AddTxMessage` / `HAL_CAN_GetRxMessage`. Bus-off is read from `CAN->ESR`, recovery uses `HAL_CAN_Stop`/`HAL_CAN_Start`, and the watchdog uses `HAL_IWDG_Refresh`. `port/stm32/main.cpp` shows the 10 ms super-loop that would run on the MCU.

## Build and test

The project uses only the C++ standard library. Tests use a small header-only framework (`tests/testfw.hpp`), so the build needs no network access.

```bash
cmake -S . -B build -G Ninja
cmake --build build
cd build && ctest --output-on-failure
```

I built it with g++ 13.3 and CMake 3.28 inside a container:

```bash
docker run --rm -v "$PWD":/work -w /work devtools:latest \
  bash -c "cmake -S . -B build -G Ninja && cmake --build build && cd build && ctest"
```

Result: 12/12 ctest executables pass, covering 166 test cases. Each executable prints its own count. Coverage by area:

| Test executable | Cases | Focus |
|---|---:|---|
| test_signal | 20 | Intel/Motorola bit packing, scaling, signed values, CRC-8 vs additive checksum |
| test_dbc | 10 | DBC parsing, DBC file vs compiled database, no overlapping signals |
| test_rx_monitor | 14 | timeout, CRC errors, frozen/jumping counters, recovery |
| test_can_if | 8 | bus-off detection, fast/slow recovery schedule, FIFO overrun |
| test_power_mode | 12 | key sequence, relays, starter cut-off, crank timeout, implausible inputs |
| test_turn_signal | 13 | flasher timing, comfort flash, hazard priority, chatter, crash |
| test_lighting | 13 | AUTO dwell/hysteresis, fail-safe, rain-light, exit delay |
| test_door_locks | 14 | auto-lock/re-arm, crash unlock/inhibit, RKE with door ajar, thermal limit |
| test_wipers | 16 | intermittent/low/high, AUTO hysteresis, park, washer, stall, crank |
| test_bcm_integration | 14 | closed-loop BCM + plant: start, status frame, locks, lamps, wipers |
| test_fault_injection | 26 | CAN timeouts, E2E bursts, frozen counter, implausible speed, bus-off, stuck/glitching switches, wiper stall, crank timeout, scenario checks |
| test_stm32_port | 6 | pin polarity/mapping, bxCAN via stub HAL, ESR bus-off, IWDG, BCM on the STM32 adapter |

## Fault-scenario benchmark

`tools/scenario_runner` generates randomised 120 s drive cycles. Each cycle covers unlock, entering the car, crank, driving with random turn-signal, hazard, headlamp and wiper use, ambient-light and rain profiles, stopping, exiting and a remote lock. About 25% of the cycles include a crash. Each cycle is assigned one of 14 fault classes:

`nominal`, `esc_timeout` (including an ESC dropout while decelerating, which leaves a stale speed), `acu_timeout`, `rls_timeout`, `e2e_burst` (all frames of one message corrupted for 2-10 s), `e2e_sporadic` (2-8% of all frames get 1-3 bit errors), `frozen_counter`, `bus_off` (1-3 bus faults of 0.2-4 s), `stuck_switch`, `input_glitch` (single-tick glitches plus 20 ms contact bounce), `wiper_stall`, `engine_no_start` (key held in START for 12 s), `rls_sensor_fault`, and `combined` (three of the above).

The same scenario, with the same random stream, is run against two controllers:

- **state machine**: `bcm::app::Bcm`, the design described above.
- **naive baseline**: `bcm::app::NaiveBcm`, a straightforward if/else prototype with the same features. It reads raw inputs and uses the last received CAN values without E2E or timeout checks. Its flasher restarts on every request change. Its intermittent wiping is timed rather than park-switch-based. It has no stall or crank supervision, it auto-locks whenever speed exceeds 15 km/h, and it retries bus-off recovery every tick.

A black-box requirement monitor (`src/sim/requirements.cpp`) checks 11 requirements against plant truth (actual lock state, wiper position, engine state) and the BCM's pin and bus outputs. It never looks at controller internals.

### Results (simulated)

Command: `./build/scenario_runner --n 700 --seed 2 --out results/heldout`. Full table: `results/heldout/summary.md`; per-scenario rows: `results/heldout/scenario_results.csv`.

| Requirement | State machine: scenarios violated / 700 | Naive: scenarios violated / 700 |
|---|---:|---:|
| R1 crash response (unlock <=300 ms, held 10 s; hazards <=500 ms) | 0 | 186 |
| R2 no uncommanded lock-state change | 0 | 155 |
| R3 starter protection | 0 | 700 |
| R4 flasher timing | 0 | 577 |
| R5 AUTO lamps on in darkness / sensor loss within 3 s | 0 | 163 |
| R6 no two AUTO lamp transitions within 2 s | 6 | 88 |
| R7 wiper stall cut-off | 0 | 59 |
| R8 wiper parks after switch-off | 0 | 597 |
| R9 lock actuator duty | 0 | 65 |
| R10 bus-off recovery rate | 0 | 62 |
| R11 status message period | 0 | 0 |
| Any requirement | 6 | 700 |

| Timing metric (same run) | State machine | Naive |
|---|---:|---:|
| Crash unlock latency (doors locked at crash), mean / max | 122 / 220 ms | 70 / 70 ms |
| Bus fault cleared to first BCM_Status, mean / max | 517 / 1050 ms | 42 / 90 ms |

How to read these results:

- **The naive baseline fails every scenario, including the 50 nominal ones.** Most of that comes from features it lacks rather than from faults. It keeps the starter engaged while the driver holds the key after the engine fires (R3). It stops the wiper wherever it happens to be (R8). It produces irregular flasher pulses when the request changes (R4). The fault-driven differences show up in R1, R2, R5, R7, R9 and R10. Example for R2: with stale or corrupted speed, the naive controller re-locks the doors after the driver unlocks them at a standstill. Example for R1: it re-locks the doors right after a crash while the stale speed is still above 15 km/h.
- **The state machine's residual failures are all R6 (lamp flicker).** There were 6 of 700 scenarios, 4 with sporadic frame corruption and 2 with bus-off. In these runs the light sensor became unavailable shortly after the lamps had switched off on a bright reading. The fail-safe then turned the lamps back on within 2 s. I left this as is: fail-safe-on is intentional, and suppressing it would trade a comfort requirement against a safety one.
- **The robustness has a cost in latency.**
  - Crash unlock takes about 50 ms longer, because the crash flag must be seen in two consecutive valid ACU frames.
  - After a long bus fault, the state machine can take up to about 1 s to come back on the bus because of its slow recovery phase. The naive controller retries every 10 ms, which in a real network would flood the bus with error frames (R10).
- **There is a development set and a held-out set.** I used the seed-1 set (`results/dev/summary.md`, 280 scenarios) while developing. It exposed several real design issues, which I fixed: stale AUTO-lamp state, rain-light release at dusk, a crash during an active turn signal delaying hazards, after-wash wipes, stalled-wiper parking, residual corruption passing an additive checksum, and an implausible speed jump that passed CRC-8. On that set the state machine shows 0 violations. The seed-2 set above was generated afterwards and is the one I quote.

Bit-error coverage of the E2E check (exhaustive count in `test_signal`): the CRC-8 detects all 1-bit and all 2-bit errors in the 56 protected data bits. The earlier additive checksum misses some 2-bit patterns. Some 3-bit patterns still get through the CRC: `test_signal` prints the count, which is 141 of 27,720 for its sample frame. That is why the speed plausibility check exists.

## Limitations and what is simulated

- All numbers above come from a host simulation: an idealised plant (lock actuator needs 80 ms of drive, wiper motor 1.0 s / 0.65 s per cycle with an 8% park zone, engine starts after 800 ms of cranking) and a simplified CAN bus. The bus has no arbitration, no error frames, and no error counters below bus-off. A bus fault forces bus-off immediately, and recovery succeeds immediately once the fault is gone.
- Frame corruption is injected above the CAN controller, i.e. errors that the CAN CRC-15 would not catch, such as those from a faulty sender or gateway. The rates used (up to 8% of frames, or 100% for bursts) are deliberately far harsher than any real bus.
- The requirements, their thresholds and the naive baseline were all written by me for this comparison. They are not taken from an OEM specification.
- The STM32 backend is type-checked and unit-tested against stub HAL headers only. `port/stm32/main.cpp` and the CubeMX-generated init code it expects were not built or run, and nothing has run on real hardware.
- There is no UDS diagnostic stack, NVM storage of DTCs, sleep/wake or network management. Rear wipers, interior lighting, multiple doors and lock feedback switches are also not modelled.

## License

MIT, see `LICENSE`.
