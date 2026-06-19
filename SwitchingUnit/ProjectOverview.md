# WPSwitch Receiver — Switching Unit

> **Project:** `WPSwitch_Receiver_Submersible / SwitchingUnit`  
> **MCU:** CH32V003A4M6 (SOP16, RISC-V RV32EC, 48 MHz, 16 KB Flash, 2 KB SRAM)  
> **Build System:** PlatformIO (`noneos-sdk` bare-metal WCH SPL)  
> **Role in System:** Lower tier of the two-tier Receiver Unit — AC switching, relay sequencing, and energy metering

---

## 1. Big Picture: The Two-Tier Receiver Architecture

The WPSwitch Receiver Unit is split into two cooperating microcontrollers. The Switching Unit is the **hardware-facing, safety-critical lower tier**:

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         RECEIVER UNIT (Physical Box)                     │
│                                                                          │
│  ┌─────────────────────┐   UART1 (115200 8N1)   ┌───────────────────┐   │
│  │   CONTROL UNIT      │◄──────────────────────►│  SWITCHING UNIT   │   │
│  │   (ESP32)           │   + Shared Presence     │  (CH32V003A4M6)   │   │
│  │                     │     wire (PC3/GPIO35)   │                   │   │
│  │  Cloud / MQTT / OTA │                         │  • Relay FSM      │   │
│  │  PumpController     │  Commands / Queries ──► │  • HLW8032 meter  │   │
│  │  Sensor data        │  ◄── Responses/Events   │  • Motor detect   │   │
│  └─────────────────────┘                         │  • Presence HB    │   │
│                                                   │  • Cooldown timer │   │
│                                             AC mains contacts ──►[Motor]  │
└──────────────────────────────────────────────────────────────────────────┘
```

**Design Rationale:**  
The CH32V003 runs without an OS (`noneos-sdk`), uses no dynamic memory, and controls AC mains relays directly. Isolating this from the ESP32's Wi-Fi and RTOS stack ensures the high-voltage actuation path remains simple, deterministic, and safe — even if the Control Unit crashes or loses Wi-Fi.

---

## 2. Hardware Summary

| Property | Value |
|---|---|
| MCU | CH32V003A4M6 (SOP16) |
| Core | QingKe RISC-V2A (RV32EC — 16 registers only) |
| Clock | 48 MHz (HSI + PLL) |
| Flash | 16 KB |
| SRAM | 2 KB |
| Package | SOP16, 14 usable GPIOs |
| Debug/Upload | SWIO on PD1 via WCH-LinkE (RISC-V mode) |

> **Critical SRAM constraint:** All variables must be statically allocated. No `malloc`/`free`. No recursion.

---

## 3. Pin Assignment (`board_pins.h`)

| Pin | Port | Function |
|---|---|---|
| 1 | PC1 | Small Relay NO — contactor coil parallel switch |
| 2 | PC2 | Relay NC — series with Red stop button |
| 3 | PC3 | Presence pin (open-drain, shared with Control Unit) |
| 4 | PC4 | Not used |
| 5 | PC6 | Software UART RX — HLW8032 data |
| 6 | PC7 | HLW8032 CF pulse (energy pulse output) |
| 7 | PD1 | SWIO debug (⚠️ do not reconfigure — see Rule 3 below) |
| 8 | PD4 | AC line feedback (optoisolator; HIGH = motor running) |
| 9 | PD5 | USART1 TX → Control Unit |
| 10 | PD6 | USART1 RX ← Control Unit |
| 11 | PD7 | NRST (reset) |
| 12–13 | PA1–PA2 | Not used |
| 14 | VSS | Ground |
| 15 | VDD | Power |
| 16 | PC0 | Large Relay NO — starter winding parallel switch |

---

## 4. Firmware Modules

### 4.1 `main.c` — Main Loop
The superloop executes five ticks in strict order every iteration:
1. `comm_protocol_tick()` — parse incoming UART bytes and dispatch commands
2. `motor_detect_tick()` — debounce the AC optoisolator feedback
3. `relay_tick()` — advance the relay state machine (uses motor state)
4. `hlw8032_tick()` — poll HLW8032 energy data via software UART
5. `presence_tick()` — output heartbeat pulses and monitor CU presence

### 4.2 `comm_protocol.c` — UART Binary Protocol
Implements the shared binary frame format with the Control Unit.

**Frame format:**
```
┌──────┬──────┬─────┬─────┬─────────────┬──────────┐
│ SOF  │ Type │ ID  │ Len │ Payload     │ Checksum │
│ 0xAA │ 1 B  │ 1 B │ 1 B │ 0..16 bytes │ 1 B      │
└──────┴──────┴─────┴─────┴─────────────┴──────────┘
Checksum = Type XOR ID XOR Len XOR (all payload bytes)
```

**Parser:** Six-state FSM with a **50 ms inter-byte timeout** (measured between consecutive bytes) that resets to `PARSE_WAIT_SOF` on a stale/incomplete frame.

**Dispatched commands (Control Unit → SU):**

| Message | Type | ID | Payload | Response |
|---|---|---|---|---|
| Pump ON | COMMAND | 0x01 | — | `[RESP_OK]`, `[RESP_ERR_COOLDOWN, cd_hi, cd_lo]`, or `[RESP_ERR_LOCKED]` |
| Pump OFF | COMMAND | 0x02 | — | `[RESP_OK]` or `[RESP_ERR_LOCKED]` |
| Ping | COMMAND | 0x05 | — | `[RESP_OK]` |
| Pump LOCK | COMMAND | 0x07 | — | `[RESP_OK]` |
| Pump UNLOCK | COMMAND | 0x08 | — | `[RESP_OK]`, `[RESP_ERR_ALREADY_OFF]`, or `[RESP_ERR_FAULT]` |
| Get Status | REQUEST | 0x03 | — | `[relay_state, motor_running, cu_present, cd_hi, cd_lo]` (5 bytes) |
| Get Energy | REQUEST | 0x04 | — | `[v_mv(4), i_ma(4), p_mw(4)]` (12 bytes, big-endian) or `[RESP_ERR_NOT_AVAILABLE]` |
| Get Version | REQUEST | 0x06 | — | `[major, minor, patch]` (3 bytes) |

**Response Status Codes (payload byte 0 in responses):**
- `RESP_OK` (0x00) — Command succeeded
- `RESP_ERR_COOLDOWN` (0x01) — Pump start rejected; cooldown active
- `RESP_ERR_ALREADY_ON` (0x02) — Pump is already running
- `RESP_ERR_ALREADY_OFF` (0x03) — Pump/lock is already off/unlocked
- `RESP_ERR_FAULT` (0x04) — Hardware or logic fault
- `RESP_ERR_UNKNOWN_CMD` (0x05) — Command ID not recognized
- `RESP_ERR_NOT_AVAILABLE` (0x06) — Metering data not available
- `RESP_ERR_LOCKED` (0x07) — Command rejected; pump is locked

**Unsolicited events (SU → Control Unit):**

| Event | ID | Payload |
|---|---|---|
| `EVT_PUMP_STARTED` | 0x01 | `[EVT_SRC_*]` — trigger source |
| `EVT_PUMP_STOPPED` | 0x02 | `[EVT_SRC_*]` |
| `EVT_HIGH_ENERGY` | 0x03 | — |
| `EVT_COOLDOWN_STARTED` | 0x04 | `[cd_hi, cd_lo]` — seconds remaining |
| `EVT_COOLDOWN_ENDED` | 0x05 | — |
| `EVT_FAULT` | 0x06 | — |
| `EVT_PRESENCE_CHANGE` | 0x07 | `[1=CU present, 0=absent]` |

Event source codes (payload byte 0 for pump started/stopped):
- `EVT_SRC_COMMAND` = 0x00 — triggered by a UART command
- `EVT_SRC_MANUAL` = 0x01 — triggered by physical button press
- `EVT_SRC_FAULT` = 0x02 — triggered by fault/external stop

### 4.3 `relay_ctrl.c` — Relay State Machine

```
                    CMD_PUMP_LOCK (from any state)
                          │
                          ▼
   IDLE ──(pump_on)──▶ STARTING ──(3 s)──▶ RUNNING
     ▲                    │                    │
     │                    │(pump_off)          │(pump_off / ext. stop)
     │                    ▼                    ▼
     └──(170 s)── COOLDOWN ◀── STOPPING ──(500 ms)──┘
     ▲
     │
   LOCKED ──(pump_unlock)──┘
```

**Start sequence** (simulates pressing Green button):
1. Activate Large NO (PC0) — energises starter winding
2. Activate Small NO (PC1) — energises contactor coil
3. After 3 s: release Large NO — starter winding disconnects

**Stop sequence** (simulates pressing Red button):
1. Deactivate both NO relays
2. Pulse NC relay (PC2) LOW for 500 ms — breaks the coil-hold circuit
3. Enter COOLDOWN (170 s)

**Pump Lockout state (LOCKED):**
- Triggered by `CMD_PUMP_LOCK` command from any state.
- Safes the pump: immediately de-energizes both NO relays (Large NO and Small NO) and holds the NC relay energized continuously to break the external contactor's self-holding coil-hold circuit.
- State machine remains locked in this state; start and stop commands are rejected with `RESP_ERR_LOCKED`.
- Persists until a `CMD_PUMP_UNLOCK` command is received.

**Unlock sequence:**
- Triggered by `CMD_PUMP_UNLOCK` command.
- De-energizes the NC relay, allowing the self-holding contactor circuit to close when commanded.
- Transitions back to `IDLE` state.

**External stop detection:** During `RUNNING`, if `motor_is_running()` returns false after the 2 s grace period, the system automatically enters COOLDOWN and fires `EVT_COOLDOWN_STARTED`.

### 4.4 `presence.c` — Heartbeat Protocol
Implements a bidirectional open-drain presence detection on PC3 with a software-based UART fallback:

- **SU output:** Drives PC3 LOW for 2 ms every 100 ms as its heartbeat
- **CU detection:** EXTI interrupt on PC3 falling edge; records timestamp
- **CU timeout:** If no CU pulse within 500 ms → `cu_present = 0`
- **Events:** Fires `EVT_PRESENCE_CHANGE` on state transitions
- **Collision avoidance:** Only records a CU pulse when `pulse_active == 0` (SU not currently outputting its own pulse)
- **UART Activity Fallback:** To mitigate the physical heartbeat line limitation (where ESP32's GPIO35 is input-only and cannot pulse the line), the Switching Unit tracks the arrival of valid, checksum-verified UART frames from the Control Unit.
- **Combined Presence:** `presence_is_control_unit_present()` returns `1` (present) if either a physical heartbeat pulse was received within the last 500 ms OR a valid UART frame was received within the last 2000 ms.

> **Note:** The physical presence line has an asymmetry where the ESP32's GPIO35 is input-only; the CU cannot drive the shared line. The UART-activity fallback serves as the primary mechanism for detecting Control Unit presence in practice.

### 4.5 `motor_detect.c` — AC Motor Feedback
Debounces the PD4 optoisolator signal (HIGH = motor running). Provides `motor_is_running()` used by the relay state machine for external-stop detection.

### 4.6 `hlw8032.c` — Energy Metering (optional)
Reads 24-byte HLW8032 packets over software UART (PC6, 4800 baud):
- Validates checksum (sum of bytes 2–22 == byte 23)
- Stores raw calibration registers and data registers.
- Calculates calibrated physical parameters (voltage in mV, current in mA, active power in mW) using high-precision calculations with 64-bit intermediates to prevent overflow.
- Calibrated values are returned as three 32-bit big-endian integers (voltage, current, power) in `REQ_GET_ENERGY` responses.
- Enabled/disabled via `FEATURE_HLW8032` build flag.

### 4.7 `hw_uart.c` / `sw_uart.c` — UART Drivers
- `hw_uart`: Wraps CH32 USART1 (PD5/PD6) for Control Unit communication
- `sw_uart`: Bit-bang software UART receiver for HLW8032 data (PC6, 4800 baud)

### 4.8 `systick.c` — Timebase
Uses SysTick + TIM2 to provide `millis()` — the common time reference for all tick-based state machines.

### 4.9 `board_init.c` — GPIO & Clock Init
Enables APB clocks for all GPIO ports and configures all pins to their correct initial states (relays off, inputs floating/pull-up as appropriate).

---

## 5. Build Variants (PlatformIO environments)

| Environment | `FEATURE_HLW8032` | Included sources |
|---|---|---|
| `SwitchingUnit_PLUS` | 1 (enabled) | All files |
| `SwitchingUnit` | 0 (disabled) | Excludes `hlw8032.c`, `sw_uart.c` |

**Build commands:**
```bash
# Full build (with HLW8032)
~/.platformio/penv/bin/pio run -e SwitchingUnit_PLUS

# Minimal build (without HLW8032)
~/.platformio/penv/bin/pio run -e SwitchingUnit

# Upload
~/.platformio/penv/bin/pio run -e SwitchingUnit_PLUS --target upload
```

---

## 6. Firmware Versioning
Defined in `platformio.ini` build flags:
```ini
-D FW_VERSION_MAJOR=0
-D FW_VERSION_MINOR=1
-D FW_VERSION_PATCH=0
```
Reported in response to `REQ_GET_VERSION` (3-byte payload: `[major, minor, patch]`).

---

## 7. Key Hardware Rules & Gotchas

### Rule 1 — HSI Clock Drift
The 48 MHz internal oscillator drifts with temperature. Software UART (`sw_uart.c`) must apply dynamic calibration by measuring start-bit widths to compensate timing errors at 4800 baud.

### Rule 2 — PFIC Interrupt Priority
Only `NVIC_PriorityGroup_1` is valid on CH32V003 (2-bit priority). Using group 2 or higher causes undefined behaviour.

### Rule 3 — SWIO Pin (PD1)
Never reconfigure PD1 as a general-purpose GPIO. Doing so locks out the WCH-LinkE programmer. Recovery requires pressing the physical Mode button on the programmer or power-cycling during link-utility connection.

### Rule 4 — No Dynamic Memory
2 KB SRAM and the RV32E register file (16 GPRs) make heap allocation and deep stack usage fatal. All state is statically allocated.

---

## 8. Protocol Compatibility Notes (cross-reference with ControlUnit)

See [ControlUnit/ProjectOverview.md](../ControlUnit/ProjectOverview.md) Section 7 for the full list of known mismatches. Summary:

| # | Issue | Severity / Status |
|---|---|---|
| 1 | **Checksum SUM vs XOR** — ControlUnit uses arithmetic sum; SU uses XOR. All frames are mutually rejected. | 🔴 Breaking |
| 2 | **REQ_GET_STATUS cooldown offset** — ControlUnit reads `payload[2..3]` as cooldown; SU sends `cu_present` at `[2]` and cooldown at `[3..4]`. | 🔴 Breaking |
| 3 | **REQ_GET_ENERGY format** — SU sends raw HLW8032 register bytes; ControlUnit expects pre-computed `voltage_mv`/`current_ma`/`power_mw` uint32s. | 🟢 **Resolved** (SU now calculates and sends 12-byte big-endian calibrated values: voltage_mv, current_ma, power_mw) |
| 4 | **Frame timeout absent in CU** — SU has 50 ms inter-byte timeout; ControlUnit parser can stall on partial frames. | 🟡 Robustness |
| 5 | **Presence asymmetry** — ESP32 GPIO35 is input-only; `cu_present` will always be 0. | 🟢 **Mitigated** (SU has 2000 ms UART-activity fallback to detect CU presence) |

---

## 9. Related Projects

| Path | Description |
|---|---|
| `../ControlUnit/` | ESP32 firmware — cloud, MQTT, OTA, pump logic, sensor fusion |
| `WPSwitch_Receiver_Rough` | Earlier monolithic prototype — superseded by this two-tier architecture |
