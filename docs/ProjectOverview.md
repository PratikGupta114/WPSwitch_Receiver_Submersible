# WPSwitch Receiver Submersible — Project Overview

> **Project:** `WPSwitch_Receiver_Submersible`  
> **Architecture:** Two-tier Distributed Controller (ESP32 + CH32V003)  
> **Target Application:** AI-powered Smart Submersible Water Pump Controller

---

## 1. System Architecture

The WPSwitch Receiver Submersible is physically and logically split into two microcontrollers to achieve safety, isolation, and robustness:

```
                  ┌──────────────────────────────────────────────────────────┐
                  │               WPSWITCH RECEIVER (Physical Box)           │
                  │                                                          │
                  │  ┌─────────────────────────┐                             │
                  │  │      CONTROL UNIT       │                             │
                  │  │         (ESP32)         │                             │
                  │  │                         │                             │
                  │  │ • Wi-Fi & MQTT 5 (TLS)  │                             │
                  │  │ • Firebase OTA Updates  │                             │
                  │  │ • RF Level Sensor UART  │                             │
                  │  │ • Pump State Machine    │                             │
                  │  │ • Local Display & UI    │                             │
                  │  └────────────┬────────────┘                             │
                  │               │                                          │
                  │               │ UART1 (115200 8N1)                       │
                  │               │ + Presence Detection Fallback            │
                  │               ▼                                          │
                  │  ┌─────────────────────────┐                             │
                  │  │     SWITCHING UNIT      │                             │
                  │  │      (CH32V003)         │                             │
                  │  │                         │                             │
                  │  │ • Hardware Relay FSM    │                             │
                  │  │ • AC Opto Motor Detect  │                             │
                  │  │ • HLW8032 Calibration   │                             │
                  │  │ • Cooldown Timer Guard  │                             │
                  │  └────────────┬────────────┘                             │
                  │               │                                          │
                  └───────────────┼──────────────────────────────────────────┘
                                  │ AC Contactor Control / Mains
                                  ▼
                            [Submersible Pump]
```

### 1.1 The Control Unit (CU) — Brain & Cloud Interface
- **Microcontroller:** ESP32 (Dual-core Xtensa LX6, 240 MHz)
- **Software Stack:** ESP-IDF v5.x, FreeRTOS, C++
- **Role:** High-level supervisor. Manages cloud communication (MQTT 5 over TLS to EMQX broker), OTA firmware updates via Firebase Cloud Functions, local display/indication (74HC595 7-segment display, WS2812B NeoPixel, buzzer), RTC timekeeping (DS1307 I2C), and RF receiver parsing for water level sensors. It evaluates water levels and makes decisions to turn the pump on or off.

### 1.2 The Switching Unit (SU) — Safe & Deterministic Actuator
- **Microcontroller:** CH32V003A4M6 (RISC-V RV32EC, 48 MHz, 16 KB Flash, 2 KB SRAM)
- **Software Stack:** Bare-metal C (`noneos-sdk` SPL), zero dynamic memory allocation
- **Role:** Direct hardware actuator. Controls three relays (Large NO for starter winding, Small NO for contactor coil, NC series break relay) to sequence pump start/stop without arcing or stalling. Monitored by an optocoupler for AC motor running feedback. Performs on-chip calibration of electrical parameters from the HLW8032 energy sensor. Ensures a mandatory 170-second cooldown timer between motor runs and supports a hardware-enforced pump lockout state.

---

## 2. Sub-Project Directory Structure

```
WPSwitch_Receiver_Submersible/
├── ControlUnit/             # ESP32 Firmware Sub-Project
│   ├── main/                # Application source code
│   │   ├── https/           # Local Captive Portal & Firebase REST API
│   │   ├── mqtt/            # MQTT 5 connectivity & command parser
│   │   ├── ota/             # Firebase Cloud Function OTA manager
│   │   ├── peripherals/     # Buttons, Buzzer, I2C RTC, RF UART
│   │   ├── pump_controller/ # State machine for auto pump gating
│   │   ├── switching_unit/  # IPC bridge to CH32 Switching Unit
│   │   └── wifi/            # WiFi station/AP credential manager
│   ├── partitions.csv       # Custom OTA partitioning layout
│   └── ProjectOverview.md   # Detailed CU specification & documentation
│
├── SwitchingUnit/           # CH32V003 Firmware Sub-Project
│   ├── src/                 # Superloop modules (main, relays, UART, presence)
│   ├── include/             # Peripheral drivers & protocol header specs
│   ├── test/                # Local UART & parser testing modules
│   ├── platformio.ini       # PlatformIO build configurations
│   └── ProjectOverview.md   # Detailed SU specification & documentation
│
├── ProjectOverview.md       # (This file) Root architecture guide
└── ProjectStatus.md         # Current tasks, bugs, and release state
```

---

## 3. Inter-Unit Communication Protocol

The CU and SU communicate over a custom serial protocol designed for deterministic framing, low parsing overhead, and noise rejection.

### 3.1 Serial Parameters
- **Interface:** UART1 (ESP32 IO26/IO25 ◄► CH32 PD5/PD6)
- **Speed:** 115200 Baud, 8 Data Bits, No Parity, 1 Stop Bit (8N1)

### 3.2 Frame Structure
Every message consists of a binary packet structured as:
```
┌──────┬──────┬─────┬─────┬─────────────────┬──────────┐
│ SOF  │ Type │ ID  │ Len │ Payload (Bytes) │ Checksum │
│ 0xAA │ 1 B  │ 1 B │ 1 B │   0..16 Bytes   │   1 B    │
└──────┴──────┴─────┴─────┴─────────────────┴──────────┘
```
- **SOF (Start of Frame):** Fixed byte `0xAA`
- **Type:** `0x01` (Command), `0x02` (Request), `0x03` (Response), `0x04` (Event)
- **ID:** Action/Query identifier (e.g., `CMD_PUMP_ON` = `0x01`)
- **Len:** Payload byte length (max 16)
- **Checksum:** XOR of `Type`, `ID`, `Len`, and all `Payload` bytes.

### 3.3 Message Catalog
- **`CMD_PUMP_ON` (`0x01`):** Requests the SU to start the pump. Returns `RESP_OK` or `RESP_ERR_COOLDOWN` (with remaining cooldown seconds) or `RESP_ERR_LOCKED`.
- **`CMD_PUMP_OFF` (`0x02`):** Requests the SU to stop the pump. Returns `RESP_OK` or `RESP_ERR_LOCKED`.
- **`CMD_PING` (`0x05`):** Keep-alive/presence check command. Returns `RESP_OK`.
- **`CMD_PUMP_LOCK` (`0x07`):** Locks the pump relay state machine into `LOCKED` status (opens NC break relay, disables start switches).
- **`CMD_PUMP_UNLOCK` (`0x08`):** Unlocks the pump back to `IDLE`.
- **`REQ_GET_STATUS` (`0x03`):** Queries SU state. Returns `[relay_state, motor_running, cu_present, cd_hi, cd_lo]`.
- **`REQ_GET_ENERGY` (`0x04`):** Queries energy metrics. Returns a 12-byte payload `[v_mv(4BE), i_ma(4BE), p_mw(4BE)]` representing calibrated voltage, current, and active power. Returns `RESP_ERR_NOT_AVAILABLE` if HLW8032 data is not ready.
- **`REQ_GET_VERSION` (`0x06`):** Queries SU firmware version. Returns `[major, minor, patch]`.

### 3.4 Presence Signaling & Robustness
1. **Physical Presence Line (PC3 ◄► GPIO35):** The SU pulses the open-drain line LOW for 2 ms every 100 ms to declare its presence. The CU detects these falling edges via hardware ISR.
2. **UART Activity Fallback (Bidirectional):** Because ESP32's GPIO35 is hardware-constrained as input-only, the CU cannot drive the physical presence line. A software fallback is implemented:
   - The CU's `SwitchingUnitManager` runs a keep-alive timer sending a `CMD_PING` every 1000 ms.
   - The SU tracks the reception of valid UART packets; receiving a packet within the last 2000 ms flags the CU as present.
3. **Inter-Byte Parsing Timeout (50 ms):** Both parser state machines reset to `PARSE_WAIT_SOF` if a byte does not arrive within 50 ms of the last byte in a partially completed frame, preventing stalls caused by line noise or MCU resets.

---

## 4. Key Submersible Pump Protection Safeguards
- **Starter Winding Sequencer:** On start, the SU energizes the starter relay alongside the main contactor coil relay. After 3.0 seconds (once the motor attains operational speed), the starter winding relay is safely opened, protecting the starter capacitor from overheating.
- **Coil-Hold Contactor Interruption:** Standard stop is performed by pulsing the NC relay for 500 ms to interrupt the contactor's self-holding coil-hold circuit.
- **Stall & Dry Run Gating:** The ESP32 monitors the water level trends and prevents motor startup if reservoir level is below a safe threshold, or stops the pump if it runs but water level fails to rise.
- **Mandatory Cooldown:** A 170-second motor cooldown timer is enforced by the CH32 to prevent pump start/stop cycles from damaging the submersible pump motor.
