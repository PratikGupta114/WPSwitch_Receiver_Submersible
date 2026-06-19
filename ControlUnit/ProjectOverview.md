# WPSwitch Receiver — Control Unit

> **Project:** `WPSwitch_Receiver_Submersible / ControlUnit`  
> **MCU:** ESP32 (ESP-IDF v5.x, FreeRTOS, C/C++)  
> **Build System:** CMake via `idf.py`  
> **Role in System:** Upper tier of the two-tier Receiver Unit — cloud brain, UI, and supervisor

---

## 1. Big Picture: The Two-Tier Receiver Architecture

The WPSwitch Receiver is physically split into **two cooperating units** that together form the complete pump control device:

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         RECEIVER UNIT (Physical Box)                     │
│                                                                          │
│  ┌─────────────────────────────────┐   UART (115200 8N1)  ┌───────────┐ │
│  │         CONTROL UNIT            │◄────────────────────►│ SWITCHING │ │
│  │         (ESP32)                 │   + Presence Pin      │   UNIT    │ │
│  │                                 │      (GPIO35)         │ (CH32V003)│ │
│  │  • Wi-Fi / MQTT / HTTPS / OTA   │                       │           │ │
│  │  • MQTT Cloud Commands          │  CMD_PUMP_ON/OFF      │ • Relays  │ │
│  │  • Water Level Sensor (UART RF) │  REQ_GET_STATUS       │ • HLW8032 │ │
│  │  • PumpController state machine │  REQ_GET_ENERGY       │ • Motor   │ │
│  │  • LED / Buzzer / 7-Segment     │  REQ_GET_VERSION      │   detect  │ │
│  │  • RTC (DS1307 I2C)             │  EVT_PUMP_STARTED     │ • Cooldown│ │
│  │  • NVS storage                  │  EVT_PUMP_STOPPED     │   timer   │ │
│  └─────────────────────────────────┘  EVT_COOLDOWN_*       └───────────┘ │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

**Why two units?**  
The AC mains switching and contactor control is handled by a minimal, safety-focused microcontroller (CH32V003) that has no wireless connectivity and no OS. This isolation ensures the high-voltage side is not exposed to the complexity of a full RTOS + Wi-Fi stack. The ESP32 Control Unit provides all intelligence and cloud connectivity, delegating actuation commands to the Switching Unit over a compact binary UART protocol.

---

## 2. Relationship to `WPSwitch_Receiver_Rough`

The `ControlUnit` is the **production-grade, submersible-pump-specific** successor to the earlier `WPSwitch_Receiver_Rough` prototype. Key distinctions:

| Dimension | `WPSwitch_Receiver_Rough` (prototype) | `ControlUnit` (this project) |
|---|---|---|
| **Architecture** | Monolithic single-MCU design | Two-tier: ESP32 + CH32V003 |
| **Actuation** | ESP32 drives relays directly | Dedicated CH32 Switching Unit over UART |
| **Communication** | RF/UART only (no inter-MCU protocol) | Compact binary frame protocol over UART1 |
| **Protocol** | No well-defined inter-unit protocol | `comm_protocol` with SOF, type, ID, len, XOR checksum |
| **Energy metering** | Not present | HLW8032 on CH32 (optional, `FEATURE_HLW8032`) |
| **Pump specifics** | Generic water-pump logic | Submersible-specific: cooldown, stall detection, water level gating |
| **Hardware target** | Rough/prototype board | Dedicated submersible receiver PCB |
| **MQTT** | Basic | MQTT 5 with TLS, Firebase OTA, EMQX broker |
| **OTA** | Not present | Full OTA via Firebase Cloud Functions |
| **Build system** | Arduino/PlatformIO | ESP-IDF CMake (`idf.py`) |

The ControlUnit was written from scratch to align with ESP-IDF best practices (ESP_LOGx, `gpio_config_t`, FreeRTOS tasks, NVS) rather than adapting Arduino-style code.

---

## 3. ESP32 Target & Build System

- **SoC:** ESP32 (dual-core Xtensa LX6)
- **Framework:** ESP-IDF v5.x
- **Build:** `idf.py build` / `idf.py flash monitor`
- **Project name:** `ControlUnit` (set in root `CMakeLists.txt`)
- **Custom partition table:** `partitions.csv` (OTA-aware layout)
- **Kconfig menu:** `WPSwitch Receiver Configuration` — all tunable parameters exposed via `idf.py menuconfig`

---

## 4. Hardware I/O Map (`board_pins.h`)

| Function | GPIO | Notes |
|---|---|---|
| Switching Unit UART TX | IO26 | UART1 TX → CH32 RX |
| Switching Unit UART RX | IO25 | UART1 RX ← CH32 TX |
| Switching Unit Presence | IO35 | Input-only; detects SU heartbeat pulses |
| RF Wireless TX (WIR-1186) | IO4 | UART2 TX |
| RF Wireless RX | IO16 | UART2 RX |
| RF PRG Pin | IO17 | Programming mode |
| I2C SDA (DS1307) | IO32 | |
| I2C SCL (DS1307) | IO27 | |
| WS2812B NeoPixel | IO23 | |
| Buzzer | IO33 | PWM/GPIO |
| Config Button | IO34 | Input-only |
| WS Switch Button | IO21 | |
| Button LED | IO22 | |
| 74HC595 SHCP (clock) | IO5 | 7-segment display |
| 74HC595 DS (data) | IO18 | |
| 74HC595 STCP (latch) | IO19 | |

---

## 5. Software Module Breakdown

All source lives in `main/`. Modules compile as a single `idf_component_register()` unit.

### 5.1 Application Entry (`main.cc`)
The top-level orchestrator. Initialises all subsystems in order and creates the FreeRTOS scheduler. The bulk of high-level state logic (connection handling, MQTT reconnect, pump decision gating) is orchestrated here with event-driven callbacks.

### 5.2 Switching Unit Manager (`switching_unit/`)
C++ class `SwitchingUnitManager` — the IPC bridge to the CH32 Switching Unit.

- **UART1** at 115 200 8N1 for binary frame exchange.
- **Presence detection** via falling-edge ISR on GPIO35 (`presenceIsrHandler` stores timestamp; `isSwitchingUnitPresent()` checks 500 ms window).
- **Blocking command API**: `sendPumpOn()`, `sendPumpOff()`, `sendPing()` use `executeTransaction()` — sends a frame and waits on a FreeRTOS binary semaphore until the async RX task delivers the matching response.
- **Async event delivery**: unsolicited `EVT_*` frames dispatched through a registered callback, allowing `PumpController` to react to pump-started/stopped events from the SU.
- See [Protocol Compatibility Notes](#7-protocol-compatibility-notes) below.

### 5.3 Pump Controller (`pump_controller/PumpController.cc`)
The core state machine for automatic water-pump management:
- **Water level gating**: only starts pump if level ≥ `PUMP_MIN_WATER_LEVEL_TO_START`, stops when ≥ `PUMP_MAX_WATER_LEVEL_TO_STOP`
- **Stall detection**: monitors water level trend; alerts if pump is on but level isn't rising
- **Manual lockout**: prevents unintended re-start after manual intervention
- **Event-driven**: reacts to `EVT_PUMP_STOPPED` / `EVT_COOLDOWN_STARTED` from the SU

### 5.4 Peripherals (`peripherals/`)

| File | Purpose |
|---|---|
| `PeripheralManager.cc` | Coordinates all local GPIO peripherals (buttons, LED, buzzer, 7-seg) |
| `UartManager.cc` | Manages UART2 (RF wireless, WIR-1186) for water-level sensor data |
| `SensorDataRepo.cc` | In-memory repository for latest sensor readings (water level, temperature, humidity) |
| `BuzzerControl.cc` | Buzzer tone patterns with PWM |
| `I2CManager.cc` | DS1307 RTC over I2C |

### 5.5 Connectivity

| Module | File | Purpose |
|---|---|---|
| Wi-Fi | `wifi/WiFiManager.cpp` | STA + AP dual mode; NVS-backed credentials; reconnect task |
| Dynamic Creds | `wifi/DynamicCredentialGenerator.cpp` | HKDF-SHA256 AP SSID/password from MAC |
| MQTT | `mqtt/MQTTManager.cc` | MQTT 5 over TLS to EMQX broker |
| Command Proc | `mqtt/MqttCommandProcessor.cc` | Parses incoming MQTT topics/payloads into pump/config commands |
| HTTPS Server | `https/HttpsServer.cpp` | Captive portal / local config API |
| HTTPS Client | `https/HttpsClient.cc` | Firebase REST API calls |
| OTA | `ota/otaManager.cc` | OTA firmware update via Firebase Cloud Functions |

### 5.6 Storage & Utilities

| Module | File | Purpose |
|---|---|---|
| NVS | `storage/nvsManager.cpp` | Non-volatile config: Wi-Fi credentials, device ID, pump settings |
| Heap Monitor | `utils/HeapMonitor.cc` | Periodic free-heap logging/alerting |
| Filters | `filters/` | Median + Kalman filters for water level sensor data |

### 5.7 LED Indication (`ledIndication/`, `ws2812/`)
WS2812B NeoPixel driver with status patterns (connecting, pumping, fault, OTA, etc.).

### 5.8 UART Protocol Framing (`uart_protocol/UartFraming.cc`)
Shared framing utilities for building/parsing binary frames (used by SwitchingUnitManager and UartManager).

### 5.9 Custom Components (`components/`)
- `esp_idf_lib_helpers`: helper wrappers around common ESP-IDF idioms.

---

## 6. FreeRTOS Task Map

| Task | Priority | Core | Stack | Purpose |
|---|---|---|---|---|
| `switching_unit_rx_task` | `MAX-2` | APP_CPU | 3072 B | UART1 byte-by-byte frame parser |
| MQTT task (IDF internal) | 12 (Kconfig) | – | 8192 B | MQTT client |
| OTA task | 8 (Kconfig) | – | 8192 B | Firmware update |
| SNTP sync task | 25 (Kconfig) | – | 2560 B | NTP time synchronization |
| WiFi reconnect task | 5 (Kconfig) | – | 2560 B | Station reconnection loop |

---

## 7. Protocol Compatibility Notes

> This section summarises the inter-unit UART protocol and **known discrepancies** found between `SwitchingUnitManager.cc` and the SwitchingUnit's `comm_protocol.c`.  See `SwitchingUnit/ProjectOverview.md` for the SU-side description.

### Frame Format (identical on both sides ✅)
```
┌──────┬──────┬─────┬─────┬─────────────┬──────────┐
│ SOF  │ Type │ ID  │ Len │ Payload     │ Checksum │
│ 0xAA │ 1 B  │ 1 B │ 1 B │ 0..16 bytes │ 1 B      │
└──────┴──────┴─────┴─────┴─────────────┴──────────┘
```

### Message Type, Command, Response & Event constants ✅
All `PROTO_TYPE_*`, `CMD_*`, `REQ_*`, `RESP_*`, and `EVT_*` values match exactly between the two sides.

### Protocol Bug Status (All Resolved)

#### ✅ BUG 1 — Checksum Algorithm Mismatch — **RESOLVED**
Both `sendFrame()` and `handleUartRx()` now use **XOR** checksum, matching the SwitchingUnit's `comm_protocol.c`.

#### ✅ BUG 2 — `REQ_GET_STATUS` Cooldown Payload Misparse — **RESOLVED**
`queryStatus()` now correctly reads cooldown from `payload[3..4]`, skipping `payload[2]` (cu_present).

#### ✅ BUG 3 — `REQ_GET_ENERGY` Format — **RESOLVED**
The SwitchingUnit now performs on-chip calibration and sends **12 bytes** of calibrated data:
`[v_mv(u32 BE)][i_ma(u32 BE)][p_mw(u32 BE)]`. The ControlUnit unpacks these directly.
A 1-byte error response `RESP_ERR_NOT_AVAILABLE` (`0x06`) is handled gracefully when the HLW8032 is not ready.

#### ✅ Frame Timeout — **RESOLVED**
`handleUartRx()` now tracks `frame_start_us` and resets the parser to `PARSE_WAIT_SOF` if no byte arrives within **50 ms** mid-frame, matching the SwitchingUnit's inter-byte timeout.

#### ✅ Presence Protocol — **RESOLVED (Software Fallback)**
GPIO35 hardware constraint remains. A software-based **UART presence fallback** is now active:
- A keep-alive timer sends `CMD_PING` every **1000 ms**, keeping the CU marked as present on the SU for its 2000 ms window.
- `queryStatus()` now parses the `cu_present` field from the SU status response (`payload[2]`).

### New Protocol Features

#### Pump Lockout (`CMD_PUMP_LOCK` / `CMD_PUMP_UNLOCK`)
- `CMD_PUMP_LOCK` (`0x07`): Forces the SU into `LOCKED` state — NC relay opens, breaking the contactor circuit.
- `CMD_PUMP_UNLOCK` (`0x08`): Returns the SU to `IDLE` state.
- `RESP_ERR_LOCKED` (`0x07`): Returned if `CMD_PUMP_ON` or `CMD_PUMP_OFF` is sent while locked.
- `RELAY_STATE_LOCKED` (`5`): New relay state value in `REQ_GET_STATUS` responses.
- API: `lockPump()`, `unlockPump()` methods on `SwitchingUnitManager`.


---

## 8. Kconfig / `sdkconfig` Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `PUMP_MIN_WATER_LEVEL_TO_START` | 15% | Won't auto-start below this level |
| `PUMP_MAX_WATER_LEVEL_TO_STOP` | 85% | Auto-stops at this level |
| `PUMP_DEFAULT_MANUAL_LOCKOUT_DURATION_MS` | 7 200 000 ms (2 h) | Post-manual-stop lockout |
| `MQTT_BROKER_URL` | EMQX TLS prod URL | Production MQTT endpoint |
| `OTA_UPDATE_CHECK_INTERVAL_HOURS` | 4 h | Background OTA polling |
| `SNTP_TIME_ZONE` | `IST-5:30` | India Standard Time |

---

## 9. Developer Quick-Start

```bash
# Navigate to the project root
cd WPSwitch_Receiver_Submersible/ControlUnit

# Configure
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

**Secrets:** Edit `main/secrets.h` with your MQTT credentials, device ID, and AP credentials before building.

---

## 10. Related Projects

| Path | Description |
|---|---|
| `../SwitchingUnit/` | CH32V003 firmware — relay actuation, energy metering, presence heartbeat |
| `WPSwitch_Receiver_Rough` | Earlier prototype (monolithic, Arduino/PlatformIO) — superseded by this project |
