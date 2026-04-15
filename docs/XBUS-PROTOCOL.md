# X.BUS Protocol Reference

> Translated from the official **X.BUS Bus Control Protocol V1.0.0** (2024-03-26,
> revised V1.0.1 2025-12-03) provided by **Shenzhen XC-ESC Technology Co., Ltd**
> (known as XC-ESC Technology). Published with permission.
>
> Original author: Wu Changxu (吴昌旭), XC-ESC Technology.
>
> This translation covers the complete protocol specification, hardware
> interface schematic, control flowchart, and register list. The original
> Chinese documentation is archived in the project's issue tracker for
> reference.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Terminology](#2-terminology)
3. [Protocol Structure](#3-protocol-structure)
   - 3.1 [Hardware / UART Configuration](#31-hardware--uart-configuration)
   - 3.2 [Frame Structure](#32-frame-structure)
4. [Service Data Control (Point-to-Point)](#4-service-data-control-point-to-point)
   - 4.1 [Read Register](#41-read-register)
   - 4.2 [Write Register](#42-write-register)
5. [Broadcast Data / Control](#5-broadcast-data--control)
   - 5.1 [Bulk Write Throttle/Brake + Read Telemetry (0x50)](#51-bulk-write-throttlebrake--read-telemetry-0x50)
   - 5.2 [Broadcast State Control (0xA0-0xAF)](#52-broadcast-state-control-0xa0-0xaf)
6. [Register List](#6-register-list)
7. [Hardware Interface (MCU Version)](#7-hardware-interface-mcu-version)
8. [Control Flowchart](#8-control-flowchart)
9. [Desktop Application (XBusMain)](#9-desktop-application-xbusmain)

---

## 1. Overview

X.BUS is a communication protocol modeled after RS-485 Modbus, designed with
**transmission efficiency as the top priority**. It uses a single-wire
half-duplex bus where one master (the Arduino/MCU) controls one or more slave
devices (ESCs).

Key characteristics:

- Master-slave architecture — **slaves never transmit on their own**
- All data exchange is initiated and controlled by the master
- Single-wire half-duplex bus (idle state: HIGH via pull-up resistor)
- Supports up to 16 slave addresses (0x00-0x0F)
- Broadcast addressing (0xFF) for simultaneous multi-ESC control

## 2. Terminology

| Term | Definition |
| ------ | ----------- |
| **Master** | The controlling device in the master-slave network. Controls slave behavior. In our case: the Arduino. |
| **Slave** | The controlled device. Receives commands from the master and responds with data or performs actions. In our case: the XC E10 ESC(s). |
| **SAdr** | Slave Address (0x00-0x0F for point-to-point, 0xFF for broadcast). |
| **RegAdr** | Register Address. Used in service control to read/write ESC configuration parameters. |

## 3. Protocol Structure

### 3.1 Hardware / UART Configuration

| Parameter | Value | Notes |
| ----------- | ------- | ------- |
| Baud rate | **115200** | |
| Start bits | 1 | |
| Data bits | 8 | |
| Parity | None | |
| Stop bits | 1 | |
| Byte order | **Little-endian** | All multi-byte values |
| Bus idle state | **HIGH** | Maintained by pull-up resistor |
| Signal polarity | **Non-inverted** | Standard UART (idle HIGH, start bit LOW) |

### 3.2 Frame Structure

All frames (master and slave) share the same structure. The **header byte**
distinguishes direction.

```text
┌────────┬──────┬───────────┬──────────┬──────────┬───────────┬──────────┐
│ Header │ SAdr │ Extra     │ FuncCode │ DataLen  │ Data      │ Checksum │
│ 1 byte │ 1 B  │ 1 byte    │ 1 byte   │ 1 byte   │ 0-33 B    │ 1 byte   │
└────────┴──────┴───────────┴──────────┴──────────┴───────────┴──────────┘
```

#### Header (1 byte)

- `0x0F` — Frame sent by master (master → slave)
- `0xF0` — Frame sent by slave (slave → master)

#### SAdr — Slave Address (1 byte)

- `0x00`-`0x0F` — Point-to-point control (address a specific slave)
- `0xFF` — Broadcast control (address all slaves)

#### Extra Byte (1 byte)

- In service control (point-to-point): ignored, can be any value
- In broadcast control: interpreted as an auxiliary control address
  (specific meaning depends on the function code)

#### Function Code (1 byte)

- `0x10`-`0x1F` — Service control: register operations
  - `0x10` = Read register
  - `0x11` = Write register
- `0x50`-`0x9F` — Broadcast: bulk register write (each code has its own format)
- `0xA0`-`0xAF` — Broadcast: state control (restart, clear flags, etc.)

#### Data Length (1 byte)

- Number of bytes in the data segment (max 33)

#### Data Segment (0-33 bytes)

- Payload contents depend on the function code

#### Checksum (1 byte)

- Cumulative sum from SAdr through the Extra Byte (inclusive), taking the
  lowest 8 bits.
- `checksum = (SAdr + ExtraByte) & 0xFF`

> **Note:** The checksum description in the original document is ambiguous —
> it states "cumulative sum from SAdr to Extra Byte (inclusive)." This covers
> only 2 bytes, which seems insufficient for data integrity. During
> implementation, we will test both interpretations:
>
> 1. Sum of SAdr + Extra Byte only (as literally stated)
> 2. Sum of all bytes from SAdr through end of Data segment (as is standard
>    in Modbus-like protocols)
>
> The correct interpretation will be confirmed during bench testing.

## 4. Service Data Control (Point-to-Point)

Service data control is for point-to-point communication. The master reads or
writes registers on a **single specific slave**. Function codes: `0x10` (read)
and `0x11` (write).

### 4.1 Read Register

Read one or more register values from a specific slave.

Example: Read registers 0x03 and 0x23 from slave #2:

Master sends:

```text
0x0F  0x02  0x00  0x10  0x02  0x03 0x23  [checksum]
 │     │     │     │     │     └─────── Data: register addresses to read
 │     │     │     │     └──────────── Length: 2 bytes
 │     │     │     └────────────────── FuncCode: 0x10 (read register)
 │     │     └──────────────────────── Extra: ignored in service control
 │     └────────────────────────────── SAdr: slave #2
 └──────────────────────────────────── Header: master frame
```

Slave responds:

```text
0xF0  0x02  0x00  0x10  0x06  0x03 0x01 0x03  0x23 0x00 0x02  [checksum]
                               │              │
                               │              └── Reg 0x23 = 0x0200
                               └── Reg 0x03 = 0x0301 (little-endian: 0x0103)
```

Response data format: for each register, 1 byte address + 2 bytes value
(little-endian). So reading N registers returns 3*N data bytes.

**Notes:**

- All registers can be read
- Reading 0 registers returns a response identical to the request (except header)

### 4.2 Write Register

Write values to one or more registers on a specific slave.

Example: Write 0x2219 to register 0x07 and 0x0001 to register 0x04 on slave #3:

Master sends:

```text
0x0F  0x03  0x00  0x11  0x06  0x07 0x19 0x22  0x04 0x01 0x00  [checksum]
                               │                │
                               │                └── Reg 0x04 = 0x0001 (LE)
                               └── Reg 0x07 = 0x2219 (LE: 0x19, 0x22)
```

Write data format: for each register, 1 byte address + 2 bytes value
(little-endian). So writing N registers sends 3*N data bytes.

Slave responds:

```text
0xF0  0x03  0x00  0x11  0x02  0x07 0x04  [checksum]
                               └─────── Successfully written register addresses
```

**Write response status codes:**

| Return Value | Meaning |
| ------------- | --------- |
| `< 0xF0` | Success (the register address itself is echoed back) |
| `0xF1` | Register index does not exist |
| `0xF2` | Attribute error (e.g., writing to a read-only register) |
| `0xF3` | Value out of range (exceeds register's min/max bounds) |

**Notes:**

- Most registers are read-only and can only be configured via the XC-Link
  phone app
- Writing 0 registers returns a response identical to the request (except header)

**Timeouts:**

- Service control: master must receive a complete response within **10ms**

## 5. Broadcast Data / Control

When `SAdr = 0xFF`, the frame is broadcast to all slaves. The Extra Byte is
interpreted as an auxiliary control address (meaning varies by function code).

### 5.1 Bulk Write Throttle/Brake + Read Telemetry (0x50)

**This is the primary command for real-time motor control and telemetry.**

Function code `0x50` simultaneously writes throttle/brake values to multiple
ESCs and optionally requests telemetry from one specific ESC.

#### Throttle/Brake Encoding (2 bytes per ESC, little-endian)

Each ESC receives a 16-bit control value:

```text
Bit:  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
      ├──────── Throttle/Brake (int14) ────────┤ Rsvd │ T/B
```

| Bit | Function |
| ----- | ---------- |
| Bit 0 | **Type flag**: 0 = throttle, 1 = brake |
| Bit 1 | Reserved (set to 0) |
| Bits 2-15 | Signed 14-bit value (as int14) |

| Bit 0 | Bits 2-15 Meaning | Range |
| ------- | ------------------- | ------- |
| 0 (throttle) | Throttle value in 0.1% steps | -1000 to +1000 (-100.0% to +100.0%) |
| 1 (brake) | Brake value in 0.1% steps | 0 to +1000 (0% to +100.0%) |

**Encoding formula:**

```text
For throttle:  encoded = (throttle_value << 2) | 0    // bit0 = 0
For brake:     encoded = (brake_value << 2) | 1       // bit0 = 1
```

Equivalent: `(value * 4) | type_flag`

**Encoding examples:**

| Input | Calculation | Encoded (hex) | Bytes (LE) |
| ------- | ------------ | --------------- | ------------ |
| Throttle +56.2% | (562 * 4) \| 0 = 2248 | 0x08C8 | 0xC8, 0x08 |
| Brake 21.0% | (210 * 4) \| 1 = 841 | 0x0349 | 0x49, 0x03 |
| Throttle -12.8% | (-128 * 4) \| 0 = -512 | 0xFE00 | 0x00, 0xFE |

#### Master Request Frame

```text
┌──────┬──────┬────────────┬──────┬─────┬────────┬────────┬─────┬──────────┐
│ 0x0F │ 0xFF │ Target SAdr│ 0x50 │ Len │ Val[0] │ Val[1] │ ... │ Checksum │
│      │      │ (for telem)│      │     │ 2 bytes│ 2 bytes│     │          │
└──────┴──────┴────────────┴──────┴─────┴────────┴────────┴─────┴──────────┘
```

- **Target SAdr** (Extra Byte): The slave whose address matches this byte will
  respond with telemetry data. Set to an address not present on the bus to
  suppress telemetry responses.
- **Val[0]**: Throttle/brake for ESC at address 0
- **Val[1]**: Throttle/brake for ESC at address 1
- And so on for each ESC on the bus

**Data length** = 2 * (number of ESCs on the bus)

#### Slave Telemetry Response

The ESC whose address matches the Target SAdr responds with:

```text
┌──────┬──────┬──────┬──────┬──────┬──────────────────────────────┬──────────┐
│ 0xF0 │ 0xFF │ 0x03 │ 0x50 │  17  │ Telemetry data (17 bytes)    │ Checksum │
└──────┴──────┴──────┴──────┴──────┴──────────────────────────────┴──────────┘
```

> **Note:** The Extra Byte in the response is documented as `0x03` in the
> example. This may be a fixed value or may vary. To be confirmed during
> bench testing.

**Telemetry data fields (17 bytes, all little-endian):**

| Offset | Size | Type | Field | Unit | Notes |
| -------- | ------ | ------ | ------- | ------ | ------- |
| 0 | 2 | int16 | Output RPM | Hz | Electrical frequency, not mechanical RPM |
| 2 | 2 | int16 | Bus Current | 0.1 A | Battery/DC bus current |
| 4 | 2 | int16 | Phase Current | 0.1 A | Motor phase current |
| 6 | 2 | uint16 | Running Status | bitfield | See status bits table below |
| 8 | 2 | int16 | Received Throttle | 0.1 % | What the ESC received |
| 10 | 2 | int16 | Output Throttle | 0.1 % | What the ESC is actually outputting |
| 12 | 2 | int16 | Bus Voltage | 0.1 V | Battery/DC bus voltage |
| 14 | 2 | - | ESC Temperature | 1 degC | Actual temp = raw_value - 40 |
| 16 | 1 | - | Motor Temperature | - | Spare / reserved |

> **RPM Conversion:** The "Output RPM" field is in Hz (electrical frequency).
> For the XC E3665 motor (4-pole / 2 pole pairs):
> `Mechanical RPM = Hz * 60 / pole_pairs = Hz * 30`

#### Running Status Bitfield

| Bit | Name | 0 | 1 |
| ----- | ------ | --- | --- |
| 0 | Over-voltage | Normal | Over-voltage |
| 1 | Under-voltage | Normal | Under-voltage |
| 2 | Current limiting | Normal | Current limited |
| 3 | Throttle source | PWM | BUS (X.BUS control active) |
| 4 | Throttle lost | Normal | Signal lost |
| 5 | Throttle not zeroed | Zeroed | Not zeroed (safety: throttle must start at 0) |
| 6 | ESC over-temp | Normal | Over-temperature |
| 7 | Motor over-temp | Normal | Over-temperature |
| 8 | MOSFET error | Normal | Abnormal |
| 9 | Operating mode | Sensorless | Sensored |
| 10 | Reserved | - | - |
| 11 | Capacitor charge | Not full | Fully charged |
| 12 | Current amp error | Normal | Abnormal |
| 13 | Power output | Released (idle) | Outputting (motor driven) |
| 14 | Brake output | Released | Braking active |
| 15 | Restart status | Restarted | Restart flagged (cleared by 0xA1 command) |

**Timeout:** In broadcast control, the master must receive the slave's
response within **2ms**.

#### Worked Example

Simultaneously command 3 ESCs and request telemetry from ESC #2:

- ESC #0: throttle +56.2%
- ESC #1: brake 21.0%
- ESC #2: throttle -12.8% (also return telemetry)

**Master sends:**

```text
0x0F  0xFF  0x02  0x50  0x06  0xC8 0x08  0x49 0x03  0x00 0xFE  [checksum]
 │     │     │     │     │     └── ESC0   └── ESC1   └── ESC2
 │     │     │     │     └── Length: 6 bytes (3 ESCs x 2 bytes)
 │     │     │     └── FuncCode: 0x50 (throttle/brake + telemetry)
 │     │     └── Extra: 0x02 (request telemetry from ESC #2)
 │     └── SAdr: 0xFF (broadcast)
 └── Header: master frame
```

**ESC #2 responds with 17 bytes of telemetry** (see field table above).

### 5.2 Broadcast State Control (0xA0-0xAF)

State control commands for global or targeted ESC management.

**Frame format:**

```text
0x0F  0xFF  [target SAdr]  [func code]  0x00  [checksum]
```

| Function Code | Command | Description |
| -------------- | --------- | ------------- |
| `0xA0` | **Restart** | If Extra = specific SAdr: restart that ESC. If Extra = 0xFF: restart all ESCs. Data length = 0. |
| `0xA1` | **Clear restart flag** | Clears bit 15 (restart status) in the running status bitfield. Allows you to detect if an ESC has restarted since the last clear. If Extra = specific SAdr: clear for that ESC. If Extra = 0xFF: clear for all. Data length = 0. |
| `0xA2`-`0xAF` | Reserved | Not documented |

## 6. Register List

All registers are **read-only** (R) unless noted. Most configuration is done
via the XC-Link phone app and stored in flash. Registers can be read via
function code `0x10`.

| Addr | Name | Default | Min | Max | R/W | Description |
| ------ | ------ | --------- | ----- | ----- | ----- | ------------- |
| 0x00 | runMode | 1 | 0 | 2 | R | 0: Direct fwd/rev, 1: Fwd/rev with brake, 2: Forward-only with brake |
| 0x01 | throtOut | 0 | -1000 | +1000 | R | Current throttle output (x0.1%) |
| 0x02 | motSpeed | 0 | -32768 | +32767 | R | Motor speed (Hz) |
| 0x04 | ProtocolType | 0 | 0 | 2 | R | 1 = X.BUS, other values = other protocols |
| 0x05 | X.BUSId | 0 | 0 | 4 | R | X.BUS slave address of this ESC |
| 0x08 | BatCellNum | 6 | 0 | 12 | R | Battery cell count. 0 = auto-detect |
| 0x09 | BatTH | 33 | 31 | 34 | R | Battery protection: 31=off, 32=auto-low, 33=auto-mid, 34=auto-high |
| 0x0C | VbatBus | 0 | 0 | 0xFFFF | R | Battery/bus voltage (x0.1V) |
| 0x0D | BUS_CRT | 0 | -32768 | +32767 | R | Bus current (x0.1A) |
| 0x0E | BECVOLT | 0 | 0 | 0xFFFF | R | BEC output voltage (x0.1V) |
| 0x10 | MotPosDir | 0 | 0 | 1 | R | Motor positive direction: 0=CCW, 1=CW |
| 0x11 | BECVolt | 6 | 6 | 8 | R | BEC voltage setting: 6=6.0V, 7=7.4V, 8=8.0V |
| 0x12 | brakMaxPow | 0 | 0 | 1000 | R | Max brake force (x0.1%) |
| 0x13 | backMaxPow | 500 | 0 | 1000 | R | Max reverse power (x0.1%) |
| 0x14 | StartAlt | 9 | 0 | 7 | R | Startup force |
| 0x15 | StopBrakPow | 0 | 0 | 1000 | R | Drag brake force (x0.1%) |
| 0x16 | Turbo | 0 | 0 | 30 | R | Turbo timing advance (electrical degrees) |
| 0x17 | TurboTime | 500 | 0 | 1000 | R | Turbo trigger time (ms) |
| 0x18 | StartPow | 100 | 0 | 1000 | R | Idle PWM (x0.1%) |
| 0x20 | mos-Tem | 65 | 0 | 0xFF | R | ESC temperature. Actual = value - 40 degC |
| 0x22 | mot-Tem | 65 | 0 | 0xFF | R | Motor temperature. Actual = value - 40 degC |
| 0x23 | XC.BrakPow | 0 | 0 | 1000 | R | X.BUS-written drag brake data |

> **Note:** R/W/E columns in the original indicate Read/Write/EEPROM-save
> attributes. All registers listed above are read-only (R). Writable
> registers (if any) are configured through the XC-Link phone app.

## 7. Hardware Interface (MCU Version)

The official reference schematic provides a half-duplex UART-to-single-wire
bus converter. The bus connector (J1) is a 3-pin 2.54mm header:

| J1 Pin | Signal | Description |
| -------- | -------- | ------------- |
| 1 | TR.BUS | Single-wire data bus |
| 2 | (varies) | Power or NC |
| 3 | GND | Ground |

### TX Path (MCU → Bus)

```text
MCU_TX ──►|── D4 (SS1030HEWS) ──┬── R7 (0Ω jumper) ──► TR.BUS
                                 │
                          D3 ──►|── (clamp to bus/3.3V)
                          D6 ──►|── (clamp to GND)
```

- D4: Schottky diode acts as **open-drain emulation** — when MCU_TX is LOW,
  the diode conducts and pulls the bus LOW. When MCU_TX is HIGH, the diode
  blocks and the bus floats HIGH via the pull-up.
- D3, D6: ESD clamping diodes for bus protection.
- R7 (0Ω): jumper/placeholder for optional series resistance.

### RX Path (Bus → MCU)

```text
                    3.3V
                     │
                   ┌─┤ R8 (2K) ── reference divider
                   │ │
TR.BUS ── R9 (2K) ─┤ U3 (LMV331TP-TR comparator)
                   │   Pin 3 (IN-) ← reference
                   │   Pin 4 (IN+) ← bus signal
                   │   Pin 1 (OUT) ─── R10 (10K pull-up) ──► MCU_RX
                   │
                  C10 (filter cap)
                   │
                  GND
```

- LMV331: Single comparator used as a **level buffer** (NOT an inverter).
  Compares bus voltage against a reference threshold.
- Bus HIGH (idle) → comparator output released → pulled HIGH by R10 → MCU_RX HIGH
- Bus LOW (data) → comparator output pulls LOW → MCU_RX LOW
- **No signal inversion.** Standard UART polarity is preserved.

### Bus Pull-up

- R4: **3K pull-up to 3.3V** on the TR.BUS line
- Holds bus HIGH when idle (UART idle state)

### Simplified Wiring for Arduino Nano R4 (5V Tolerant)

The Nano R4 has 5V tolerant GPIO and operates its UART at 5V logic levels.
For bench testing, a simplified circuit can be used:

```text
                    5V (or 3.3V)
                     │
                    [3K-10K pull-up]
                     │
Arduino TX ──[1K]──┬─┤── ESC X.BUS Yellow wire
                   │
Arduino RX ────────┘
                        ESC X.BUS Brown ── GND (common ground)
                        ESC X.BUS Red ── NOT CONNECTED
```

- **TX**: Series resistor (1K) prevents bus contention and limits current
  when both master and slave drive the bus simultaneously during turnaround.
- **RX**: Direct connection. The Nano R4 GPIO is 5V tolerant, so no level
  shifting is needed.
- **Pull-up**: 3K-10K to 3.3V or 5V. Holds bus idle HIGH.
- **No NPN inverter needed.** X.BUS uses standard UART polarity (unlike
  S.BUS which is inverted).

> **Important:** The ESC X.BUS Red wire is BEC power output. **Never connect
> BEC power from two ESCs simultaneously** — this will cause damage. Leave
> the red wire disconnected.

## 8. Control Flowchart

The official control flowchart (translated from the companion document
"车模电调X.BUS最简控制流程.pdf") describes the recommended master control
loop running at **10ms period (10-50ms acceptable)**:

```text
┌─────────────────────────────────────┐
│  Control cycle start (10ms period)  │
│  Acceptable range: 10-50ms         │
└──────────────┬──────────────────────┘
               │
        ┌──────▼──────┐
        │ Received    │── No ──► Send throttle/brake data ──► End cycle
        │ telemetry   │
        │ response?   │
        └──────┬──────┘
               │ Yes
        ┌──────▼──────────────┐
        │ Master restart flag │
        │ is set?             │
        └──────┬──────────────┘
               │ Yes
        ┌──────▼──────────────────────┐     ┌──────────────────────────┐
        │ Check slave restart status  │     │ Clear master restart     │
        │ (bit 15 in running status)  │     │ flag                     │
        └──────┬──────────────────────┘     └──────────────────────────┘
               │                                       ▲
               ├── bit15 = 0 ──► slave restarted! ─────┘
               │                 Send clear restart
               │                 flag command (0xA1)
               │
               └── bit15 = 1 ──► slave has NOT restarted
                                 (flag still set from last clear)
               │
        ┌──────▼──────────────────────┐
        │ Capacitor charged?          │
        │ (bit 11 in running status)  │
        └──────┬──────────────────────┘
               │ Yes (bit11 = 1)
        ┌──────▼──────────────────────┐
        │ Throttle source = BUS?      │◄── Green box: "Further processing:
        │ (bit 3 in running status)   │    identify faults/alarms.
        └──────┬──────────────────────┘    Expandable here."
               │
               ├── Yes (BUS mode) ──────────────────────────┐
               │                                            │
               └── No (PWM mode) ──► Record recognition     │
                                     time (time2)           │
                                            │               │
                              ┌─────────────▼────────┐      │
                              │ Recognition time     │      │
                              │ exceeds 5 seconds?   │      │
                              └──────┬───────────────┘      │
                                     │ Yes                   │
                              ┌──────▼───────────────┐      │
                              │ Record error time    │      │
                              │ (time1)              │      │
                              └──────┬───────────────┘      │
                              ┌──────▼───────────────┐      │
                              │ Error time > 5 sec?  │      │
                              └──────┬───────────────┘      │
                                     │ Yes                   │
                              ┌──────▼───────────────┐      │
                              │ Send restart command │      │
                              │ Set restart flag = 1 │      │
                              └──────┬───────────────┘      │
                              ┌──────▼───────────────┐      │
                              │ Set throttle array   │      │
                              │ to 0 (zero throttle) │      │
                              └──────┬───────────────┘      │
                                     │                      │
        ┌────────────────────────────┐│   ┌─────────────────▼────────────┐
        │ Red box: "Other handling:  ││   │ Throttle zeroed? (bit 5)    │
        │ identify protection types  ││   └──────┬──────────────────────┘
        │ (expandable for control)"  ││          │
        └────────────────────────────┘│   ┌──────▼──────────────────────┐
                                      │   │ No ──► Record timeout       │
                                      │   │        (time3)              │
                                      │   │        If > 5s: send       │
                                      │   │        restart command      │
                                      │   └──────┬──────────────────────┘
                                      │          │ Yes (throttle is zero)
                                      │   ┌──────▼──────────────────────┐
                                      │   │ Set throttle array to      │
                                      │   │ target throttle/brake      │
                                      │   └──────┬──────────────────────┘
                                      │          │
                                      └──────────┤
                                                 │
                                          ┌──────▼────────────────────┐
                                          │ Send throttle/brake data  │
                                          └──────┬────────────────────┘
                                                 │
                                          ┌──────▼────────────────────┐
                                          │ End control cycle         │
                                          └───────────────────────────┘
```

**Key takeaways from the flowchart:**

1. **10ms control period** (100Hz) — acceptable range is 10-50ms
2. **Restart detection**: use bit 15 + command 0xA1 to detect/clear ESC restarts
3. **Capacitor charge wait**: don't send throttle until bit 11 = 1 (caps charged)
4. **BUS mode check**: bit 3 tells you if the ESC is accepting X.BUS commands
   (vs PWM input). If not in BUS mode after 5 seconds, send a restart.
5. **Throttle zero check**: bit 5 — the ESC requires throttle to start at
   zero before accepting non-zero values (safety interlock)
6. **Timeout recovery**: if the ESC doesn't switch to BUS mode or zero
   throttle within 5 seconds, restart it and try again

## 9. Desktop Application (XBusMain)

The guide includes `XBusMain.exe`, a LabVIEW-based desktop application for
X.BUS testing and configuration. **This application is not needed for our
Arduino implementation** but is documented here for completeness.

**Requirements:**

- LabVIEW runtime environment (download from XC-ESC's Baidu cloud share)
- XC925 USB-to-X.BUS adapter (purchase from XC-ESC aftermarket support)

**Source code:** `XBusMain.zip` contains 3 LabVIEW VI files:

- `XCpBusMain.vi` — Main application
- `selSerial.vi` — Serial port selector
- `showSta.vi` — Status display

The desktop app screenshot (Appendix 4 in the original document) shows:

- Real-time throttle sliders for multiple ESCs
- RX/TX data hex display
- Register value readout panel
- Waveform chart for monitoring motor response

---

## Revision History

| Version | Date | Author | Changes |
| --------- | ------ | -------- | --------- |
| V1.0.0 | 2024-03-26 | Wu Changxu | Initial release |
| V1.0.1 | 2025-12-03 | Wu Changxu | 1. Modified running status frame data length. 2. Reorganized status flag bits. |

---

*This document was translated from Chinese to English for the Arduino-DiggerSetup
project. The original documentation remains the authoritative reference.
Protocol provided by Shenzhen XC-ESC Technology Co., Ltd. Published with
permission.*
