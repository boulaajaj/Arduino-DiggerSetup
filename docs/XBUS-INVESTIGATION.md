# X.BUS Investigation Log

History of X.BUS protocol investigation for the XC E10 ESC, from initial
probing through receiving the official protocol specification.

## Timeline

### Phase 1: Passive Listening (Mar 20-31, 2026)

**Hypothesis:** X.BUS might auto-broadcast telemetry (like Spektrum or
Hobbywing ESCs).

**What we tried:**
- `sketches/xbus_probe/xbus_probe.ino` — Passive listener on D0 (Serial RX)
- Scanned baud rates: 115200, 100000, 19200, 57600, 38400, 9600
- Looked for Spektrum X-Bus ESC telemetry packets (0x20 address, big-endian,
  16-byte packets)
- Direct wire connection: ESC X.BUS Yellow → D0, Brown → GND

**Result:** Zero data at all baud rates. No signal activity on the bus wire.

**Why it failed:**
1. X.BUS is master-slave — the ESC never transmits unless polled
2. We had no TX connection — couldn't send requests even if we knew the protocol
3. We were decoding the wrong protocol (Spektrum, not XC proprietary)

### Phase 2: Inverted Signal Test (Apr 1, 2026)

**Hypothesis:** Maybe the signal is inverted (like S.BUS) and we need to
invert it to see UART data.

**What we tried:**
- `sketches/xbus_inverted_test/xbus_inverted_test.ino` — Passive listener
  via NPN inverter circuit on D2
- Wiring: X.BUS Yellow → 1K → NPN base, 10K pull-up on collector → D2
- Same baud rate scan as Phase 1
- Tested while motors were running

**Result:** "X.BUS is electrically dead — zero toggles even while motors
running." No signal detected through the inverter either.

**Why it failed:**
1. Same master-slave issue — nothing to listen to
2. The inverter was wrong anyway — X.BUS is standard UART polarity (not
   inverted like S.BUS)

### Phase 3: Active Polling (Apr 3, 2026)

**Hypothesis:** X.BUS requires polling — we need to send request commands to
get responses.

**What we tried:**
- `sketches/xbus_poll_test/xbus_poll_test.ino` — Active polling on D2/D3
  via SoftwareSerial
- Half-duplex circuit: D3 (TX) via 1K resistor, D2 (RX) via NPN inverter
- Sent 16 different command patterns:
  - Hobbywing V4: `9B 03 00 00 00 9E`
  - Hobbywing V5: `9B 03 00 60 00 FE`
  - KISS ESC protocol
  - JR PROPO XBUS frames
  - Address queries
  - Common sync bytes
- Tested 6 baud rates: 115200, 19200, 9600, 38400, 57600, 100000
- 200ms polling interval

**Result:** Appeared to get a 4-byte response at 19200 baud to the Hobbywing
V4 command. Subsequently captured "875 polls, 33KB of data" with automatic
V4/V5 protocol switching.

**What actually happened (post-mortem):**
- The "data" was almost certainly **our own TX bytes echoing back** through
  the shared half-duplex bus connection. SoftwareSerial does not suppress
  echo on half-duplex wiring.
- The 4-byte "response" to a 6-byte command is consistent with partial echo
  due to SoftwareSerial timing jitter.
- The NPN inverter on D2 was inverting a non-inverted signal, producing
  garbled echo rather than clean echo — which made it look like different
  data.
- The real protocol (115200 baud, `0x0F` header) was never tried because we
  didn't know the correct protocol.

### Phase 4: Research & Conclusion (Apr 4-7, 2026)

**Research findings:**
- Investigated JR PROPO XBUS protocol: 250 kbps, 3.3V, 14ms packets, CRC —
  completely different protocol that shares the "XBUS" name
- Determined XC's "X.BUS" is a proprietary protocol, not JR PROPO XBUS
- Concluded (incorrectly) that X.BUS was "for PWM input and tuning only, not
  telemetry"
- Committed investigation sketches with message: "X.BUS investigation
  concluded: X.BUS is a PWM input for real-time parameter tuning, NOT a
  telemetry output. Telemetry only available via Bluetooth (XC-Link app)."

**Why the conclusion was wrong:**
- X.BUS absolutely does provide telemetry — but only when polled with the
  correct XC proprietary protocol
- The protocol was undocumented in English and not discoverable through web
  search or reverse engineering of common RC protocols
- We needed the official spec from XC-ESC Technology to proceed

### Phase 5: Official Protocol Received (Apr 14, 2026)

**What happened:** XC-ESC Technology (sales7@xc-bldc.com) responded to our
March 31 email with the complete X.BUS protocol documentation package:

1. **X.BUS Bus Control Protocol V1.0.0** (revised V1.0.1, 2025-12-03) —
   Full protocol spec with frame structure, function codes, register list
2. **Simplified Control Flowchart** — Master control loop reference
3. **Hardware Schematic (MCU Version)** — Half-duplex bus interface circuit
4. **XBusMain.exe** — LabVIEW desktop application + source code
5. Permission to publish the translated protocol in our public repository

**Key revelations from the official spec:**

| What | Our assumption | Reality |
|------|---------------|---------|
| Architecture | ESC might auto-broadcast | **Master-slave: ESC only responds to polls** |
| Protocol | Spektrum / Hobbywing / JR | **Custom modbus-like (XC proprietary)** |
| Baud rate | 115200 was one of our tests | **115200 confirmed** |
| Byte order | Big-endian (Spektrum) | **Little-endian** |
| Packet header | 0x20 (Spektrum ESC addr) | **0x0F (master) / 0xF0 (slave)** |
| Signal polarity | Tried inverted (NPN) | **Non-inverted standard UART** |
| Bus interface | Direct wire to RX | **Half-duplex with TX+RX on shared bus** |
| Telemetry | "Not available via X.BUS" | **17 bytes: RPM, current, voltage, temp** |
| Control | Separate from telemetry | **Combined: throttle command triggers telemetry response** |

## What We Need to Try Next

See GitHub issues for detailed task tracking. Summary:

1. **Build simplified half-duplex circuit** — TX through 1K resistor, RX
   direct, 3K-10K pull-up. No NPN inverter (X.BUS is not inverted).
2. **Write X.BUS master sketch** — Send proper `0x0F` frames with function
   code `0x50`, parse `0xF0` telemetry responses.
3. **Bench-test with single ESC** — Verify protocol at 115200, confirm
   checksum interpretation, measure response timing.
4. **Integrate into main controller** — Replace Servo PWM output with X.BUS
   throttle commands, use telemetry for PID feedback.

## Lessons Learned

1. **"Zero data" doesn't mean "dead wire"** — it can mean the device is
   waiting for you to speak first.
2. **Protocol name collisions are real** — "X.BUS", "XBUS", and "X-Bus" are
   used by at least 3 unrelated protocols (XC-ESC, JR PROPO, Spektrum).
3. **Echo on half-duplex looks like data** — Always account for TX echo when
   both TX and RX share the same wire through SoftwareSerial.
4. **When in doubt, ask the manufacturer** — The email to XC-ESC Technology
   was the breakthrough. No amount of reverse engineering would have
   discovered this modbus-like protocol from common RC protocol patterns.
