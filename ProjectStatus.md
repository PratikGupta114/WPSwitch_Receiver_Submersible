# WPSwitch Receiver Submersible — Project Status

> **Last Updated:** 2026-06-19  
> **Overall Status:** Production-Ready Transition  
> **Architecture State:** Two-tier distributed model (ESP32 + CH32V003) fully integrated. Core communication issues are 100% resolved.

---

## 1. Feature Completion Matrix

| Sub-System / Feature | Component | Status | Target MCU | Notes |
|---|---|---|---|---|
| **Relay Sequencing FSM** | Switching Unit | 🟢 Complete | CH32V003 | Handles Large/Small NO relay startup and NC stop pulse |
| **Motor Run Detection** | Switching Unit | 🟢 Complete | CH32V003 | Debounces optocoupled feedback from AC motor line |
| **HLW8032 Energy Metering** | Switching Unit | 🟢 Complete | CH32V003 | Performs 64-bit integer fixed-point calibration on-chip |
| **Protocol FSM & Timeouts** | Both | 🟢 Complete | Both | XOR-checksum with 50 ms inter-byte frame reset |
| **Presence HB & Fallback** | Both | 🟢 Complete | Both | Uses physical pulse + UART-activity ping keep-alive |
| **Pump Lockout Safety** | Both | 🟢 Complete | Both | Relays safe-off; NC relay holds contactor break |
| **WiFi Manager & Portal** | Control Unit | 🟢 Complete | ESP32 | STA + AP modes, captive portal, NVS credentials |
| **MQTT 5 Connectivity** | Control Unit | 🟢 Complete | ESP32 | TLS-secured EMQX connection with command parser |
| **Firebase OTA Update** | Control Unit | 🟢 Complete | ESP32 | Secure REST version checking and bootloader write |
| **UI Displays & Indicators** | Control Unit | 🟢 Complete | ESP32 | NeoPixel patterns, 74HC595 7-segment, I2C RTC, buzzer |

---

## 2. Protocol Integration & Compatibility (Resolved Issues)

The initial integration tests revealed five key discrepancies between the Control Unit (`SwitchingUnitManager.cc`) and the Switching Unit (`comm_protocol.c`). All of these have been resolved:

1. **Checksum Algorithm Mismatch (XOR):**
   - *Status:* 🟢 **Resolved**. Both sides now use bitwise XOR of the type, ID, length, and payload bytes, replacing the old arithmetic sum checksum.
2. **`REQ_GET_STATUS` Cooldown Offset:**
   - *Status:* 🟢 **Resolved**. The Control Unit correctly extracts the presence status byte first, then reads the 2-byte cooldown counter from payload bytes `[3..4]`.
3. **`REQ_GET_ENERGY` Format:**
   - *Status:* 🟢 **Resolved**. The Switching Unit calculates the voltage (mV), current (mA), and power (mW) using 64-bit fixed-point math and returns them as a 12-byte big-endian payload. The Control Unit successfully unpacks these values.
4. **Frame Timeout:**
   - *Status:* 🟢 **Resolved**. Both the ESP32 UART parser and the CH32V003 parser reset to `PARSE_WAIT_SOF` if a byte in a partial frame is delayed by more than 50 ms.
5. **Presence Asymmetry:**
   - *Status:* 🟢 **Resolved**. ESP32's GPIO35 is input-only. A software-based UART activity fallback has been added: the CU sends a `CMD_PING` every 1000 ms, and the SU marks the CU as present if a valid packet arrives within 2000 ms.

---

## 3. Host Verification Tests
Host unit tests for the Switching Unit (`SwitchingUnit/test/test_comms_hlw.c`) mock out hardware peripherals (SysTick, UART, relays, HLW8032) to verify communication logic.
- **HLW8032-Enabled Test Suite (`FEATURE_HLW8032=1`):** 🟢 **12/12 Tests Passing**
- **HLW8032-Disabled Test Suite (`FEATURE_HLW8032=0`):** 🟢 **10/10 Tests Passing**

Tests cover PING, checksum failure recovery, inter-byte parsing timeouts, status/version queries, energy calibrations, lock/unlock behaviors, and lockout command overrides.

---

## 4. Open Tasks & Backlog (Control Unit TODOs)

While the core functionality and communication layer are robust, the following tasks remain in the Control Unit code backlog:

### 4.1 UI & Button Interaction
- [ ] **Short Press Behavior:** Implement the config button short press callback inside [main.cc](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/main.cc#L1187). (Long press currently triggers config mode/restart).
- [ ] **Transmitter Info Endpoints:** Implement displays and response handling for post-deployment RF transmitter APIs:
  - `REQUEST_TYPE_TRANSMITTER_APP_INFO` in [main.cc:L3212](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/main.cc#L3212)
  - `REQUEST_TYPE_TRANSMITTER_DEVICE_INFO` in [main.cc:L3217](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/main.cc#L3217)
  - `REQUEST_TYPE_INVALID` handler in [main.cc:L3222](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/main.cc#L3222)

### 4.2 Security & Validation
- [ ] **RF Input Validation:** Add length and validity checks for RF credentials parsed from the local server configuration:
  - Validate `hardwareID` in [UartManager.cc:L1384](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/peripherals/UartManager.cc#L1384)
  - Validate `NetworkID` in [UartManager.cc:L1405](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/peripherals/UartManager.cc#L1405)
  - Validate `DestinationID` in [UartManager.cc:L1426](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/peripherals/UartManager.cc#L1426)
  - Validate `encryption Key` in [UartManager.cc:L1447](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/peripherals/UartManager.cc#L1447)

### 4.3 OTA Integration
- [ ] **Task Comments:** Clean up developer comments inside [otaManager.cc:L136](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/ota/otaManager.cc#L136).
- [ ] **Main App Clearance Coordination:** Integrate and complete the `onNewUpdateAvailableListener` logic to verify that the main application state (e.g. pump not currently running) allows update execution safely ([otaManager.cc:L330](file:///home/pratik/WPSwitch_Receiver_Submersible/ControlUnit/main/ota/otaManager.cc#L330)).
