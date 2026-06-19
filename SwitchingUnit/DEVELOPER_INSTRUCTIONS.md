# SwitchingUnit — Developer Instructions for Remaining Protocol Issues

> **Audience:** SwitchingUnit firmware team (CH32V003A4M6)  
> **Related project:** `WPSwitch_Receiver_Submersible/SwitchingUnit`  
> **Cross-reference:** [ControlUnit/ProjectOverview.md](../ControlUnit/ProjectOverview.md) Section 7  

This document provides detailed implementation specifications and code-level instructions for resolving the three remaining protocol and hardware-level issues on the **Switching Unit (SU)** side.

---

## 1. HLW8032 Energy Data Processing & Format Alignment

### The Problem
Currently, the SU responds to `REQ_GET_ENERGY` by sending 13 bytes containing the raw, uncalibrated registers of the HLW8032 metering IC. However, the Control Unit (CU) expects a 12-byte payload consisting of pre-calculated, physical electrical parameters in standard unit scales (big-endian 32-bit integers):
1. `voltage_mv` (Voltage in millivolts, e.g. `230000` for 230.0V)
2. `current_ma` (Current in milliamperes, e.g. `4500` for 4.5A)
3. `power_mw` (Active Power in milliwatts, e.g. `1035000` for 1035.0W)

The CH32V003 must perform these calculations locally using **fixed-point integer arithmetic** to fit within its extremely tight SRAM (2 KB) and flash constraints, avoiding float overhead.

### Mathematical Formulation
The HLW8032 provides a factory-calibrated parameter (coefficient) and a measured data (frequency) for each quantity:
*   **Voltage ($V$):**
    $$V\text{ (mV)} = \frac{V_{\text{reg\_coeff}} \times V_{\text{scale}}}{V_{\text{reg\_data}}}$$
*   **Current ($I$):**
    $$I\text{ (mA)} = \frac{I_{\text{reg\_coeff}} \times I_{\text{scale}}}{I_{\text{reg\_data}}}$$
*   **Active Power ($P$):**
    $$P\text{ (mW)} = \frac{P_{\text{reg\_coeff}} \times P_{\text{scale}}}{P_{\text{reg\_data}}}$$

#### Constructing 24-bit Register Values
Each of the raw parameters and data values are stored across 3 bytes in the `hlw8032_data_t` structure. Convert them to 32-bit unsigned integers:
```c
uint32_t v_coeff = ((uint32_t)edata->voltage_reg[0] << 16) | ((uint32_t)edata->voltage_reg[1] << 8) | edata->voltage_reg[2];
uint32_t v_data  = ((uint32_t)edata->voltage_data[0] << 16) | ((uint32_t)edata->voltage_data[1] << 8) | edata->voltage_data[2];

uint32_t i_coeff = ((uint32_t)edata->current_reg[0] << 16) | ((uint32_t)edata->current_reg[1] << 8) | edata->current_reg[2];
uint32_t i_data  = ((uint32_t)edata->current_data[0] << 16) | ((uint32_t)edata->current_data[1] << 8) | edata->current_data[2];

uint32_t p_coeff = ((uint32_t)edata->power_reg[0] << 16) | ((uint32_t)edata->power_reg[1] << 8) | edata->power_reg[2];
uint32_t p_data  = ((uint32_t)edata->power_data[0] << 16) | ((uint32_t)edata->power_data[1] << 8) | edata->power_data[2];
```

#### Determining Scale Factors
The scale factors ($V_{\text{scale}}$, $I_{\text{scale}}$, $P_{\text{scale}}$) are determined by the analog component selection on your PCB:
*   **Voltage Divider Ratio:** Typ. $R_{\text{up}} = 4 \times 1\,\text{M}\Omega = 4000\,\text{k}\Omega$, $R_{\text{down}} = 1\,\text{k}\Omega$. Ratio = 4001.
*   **Current Shunt Resistor:** Typ. $R_{\text{shunt}} = 2\,\text{m}\Omega$ ($0.002\,\Omega$).
*   **Internal Constants:** Reference voltage $V_{\text{ref}} = 1.218\,\text{V}$.

Using these typical values:
*   **Voltage Scale ($V_{\text{scale}}$):** $\approx 1.88 \times \text{Ratio} \approx 7521.88 \rightarrow$ use **`7522`** (multiplied by 1000 to output millivolts).
*   **Current Scale ($I_{\text{scale}}$):** $\approx \frac{1.88}{R_{\text{shunt}}} \approx 940 \rightarrow$ use **`940000`** (multiplied by 1,000,000 to output milliamps).
*   **Power Scale ($P_{\text{scale}}$):** $\approx \frac{3.53}{R_{\text{shunt}}} \times \text{Ratio} \approx 7061760 \rightarrow$ use **`7061760`** (multiplied by 1000 to output milliwatts).

*Note: These constants must be fine-tuned during production calibration for each board batch.*

### Proposed Implementation (`comm_protocol.c` / `hlw8032.c`)
To prevent integer overflow during multiplication, perform the intermediate products in 64-bit unsigned math (`uint64_t`) before dividing by the 24-bit data register:

```c
#define V_SCALE_FACTOR  7522ULL
#define I_SCALE_FACTOR  940000ULL
#define P_SCALE_FACTOR  7061760ULL

void calculate_energy_parameters(const hlw8032_data_t *edata, uint32_t *voltage_mv, uint32_t *current_ma, uint32_t *power_mw) {
    if (!edata || !edata->valid) {
        *voltage_mv = 0;
        *current_ma = 0;
        *power_mw = 0;
        return;
    }

    uint32_t v_coeff = ((uint32_t)edata->voltage_reg[0] << 16) | ((uint32_t)edata->voltage_reg[1] << 8) | edata->voltage_reg[2];
    uint32_t v_data  = ((uint32_t)edata->voltage_data[0] << 16) | ((uint32_t)edata->voltage_data[1] << 8) | edata->voltage_data[2];

    uint32_t i_coeff = ((uint32_t)edata->current_reg[0] << 16) | ((uint32_t)edata->current_reg[1] << 8) | edata->current_reg[2];
    uint32_t i_data  = ((uint32_t)edata->current_data[0] << 16) | ((uint32_t)edata->current_data[1] << 8) | edata->current_data[2];

    uint32_t p_coeff = ((uint32_t)edata->power_reg[0] << 16) | ((uint32_t)edata->power_reg[1] << 8) | edata->power_reg[2];
    uint32_t p_data  = ((uint32_t)edata->power_data[0] << 16) | ((uint32_t)edata->power_data[1] << 8) | edata->power_data[2];

    *voltage_mv = (v_data == 0) ? 0 : (uint32_t)(((uint64_t)v_coeff * V_SCALE_FACTOR) / v_data);
    *current_ma = (i_data == 0) ? 0 : (uint32_t)(((uint64_t)i_coeff * I_SCALE_FACTOR) / i_data);
    *power_mw   = (p_data == 0) ? 0 : (uint32_t)(((uint64_t)p_coeff * P_SCALE_FACTOR) / p_data);
}
```

Then update `REQ_GET_ENERGY` in `comm_protocol.c` to send these 12 big-endian bytes:
```c
    case REQ_GET_ENERGY: {
#if FEATURE_HLW8032
        const hlw8032_data_t *edata = hlw8032_get_data();
        if (edata->valid) {
            uint32_t v_mv, i_ma, p_mw;
            calculate_energy_parameters(edata, &v_mv, &i_ma, &p_mw);
            
            resp[0]  = (uint8_t)(v_mv >> 24);
            resp[1]  = (uint8_t)(v_mv >> 16);
            resp[2]  = (uint8_t)(v_mv >> 8);
            resp[3]  = (uint8_t)(v_mv & 0xFF);
            
            resp[4]  = (uint8_t)(i_ma >> 24);
            resp[5]  = (uint8_t)(i_ma >> 16);
            resp[6]  = (uint8_t)(i_ma >> 8);
            resp[7]  = (uint8_t)(i_ma & 0xFF);
            
            resp[8]  = (uint8_t)(p_mw >> 24);
            resp[9]  = (uint8_t)(p_mw >> 16);
            resp[10] = (uint8_t)(p_mw >> 8);
            resp[11] = (uint8_t)(p_mw & 0xFF);
            
            comm_send_response(REQ_GET_ENERGY, resp, 12);
        } else {
            resp[0] = RESP_ERR_NOT_AVAILABLE;
            comm_send_response(REQ_GET_ENERGY, resp, 1);
        }
#else
        resp[0] = RESP_ERR_NOT_AVAILABLE;
        comm_send_response(REQ_GET_ENERGY, resp, 1);
#endif
        break;
    }
```

---

## 2. UART RX Frame Timeout Handler

### The Problem
If the communication line receives noise or an incomplete frame (e.g. if the ESP32 restarts mid-transmission), the UART RX parser FSM in `comm_protocol_tick()` can get stuck in a non-idle state (e.g. waiting for payload bytes or a checksum). This will cause it to discard or corrupt the next legitimate frame.

### Proposed Implementation
Add an inter-byte timeout checking mechanism using the sys_tick timer. If more than **50 ms** passes between two bytes of the same frame, reset the parser FSM to `PARSE_WAIT_SOF`.

In `comm_protocol.c`:
```c
static uint32_t last_rx_byte_ms = 0;

void comm_protocol_tick(void) {
    uint32_t now = millis();

    // Check for frame timeout (50ms)
    if (state != PARSE_WAIT_SOF && (now - last_rx_byte_ms > 50UL)) {
        state = PARSE_WAIT_SOF;
    }

    int16_t b = hw_uart_rx_byte(); // Or your non-blocking UART read function
    if (b < 0) {
        return; // No byte received
    }

    last_rx_byte_ms = now; // Update timestamp on byte receipt
    uint8_t byte = (uint8_t)b;

    switch (state) {
        case PARSE_WAIT_SOF:
            if (byte == SOF_BYTE) {
                calculated_checksum = 0;
                state = PARSE_WAIT_TYPE;
            }
            break;
        // ... (rest of FSM)
    }
}
```

---

## 3. Presence Line Asymmetry & Fallback Strategy

### The Problem
The physical presence detection wire is connected to **PC3** on the CH32V003 and **GPIO35** on the ESP32. 
GPIO35 on the ESP32 is **input-only** (it lacks output driving circuitry). As a result:
*   The SU can successfully signal its presence to the ESP32 by pulsing PC3.
*   The ESP32 **cannot** pulse the line back to the SU.
*   Therefore, `presence_is_control_unit_present()` on the SU will always return `0`.

### Proposed Implementation
Until a future hardware revision routes the presence line to a bidirectional GPIO on the ESP32 (e.g. GPIO25/26/27), you must implement a software fallback:
1.  **Software Heartbeat over UART:** The SU should treat any valid UART packet received from the CU as a confirmation of the CU's presence.
2.  **Reset timer on UART Packet:** Each time `comm_protocol_tick()` successfully parses a valid frame from the CU, reset a `last_cu_uart_ms` timestamp.
3.  **Modified Presence Logic:**
    ```c
    uint8_t presence_is_control_unit_present(void) {
        // If we have received a valid UART message in the last 2000 ms,
        // treat the CU as present even if the physical presence line is silent.
        if (millis() - last_cu_uart_ms < 2000UL) {
            return 1;
        }
        
        // Otherwise, fall back to physical pin heartbeat detection (if hardware gets fixed)
        return physical_presence_detected; 
    }
    ```
This ensures the `cu_present` status reported in `REQ_GET_STATUS` (payload byte 2) correctly indicates the CU's online status, allowing the relay logic to operate normally.

---

## 4. Verification & Testing Checklist

After implementing these fixes, compile and run the following tests:
1.  **Unit Tests:** Run the PlatformIO tests to verify the binary parser handles XOR checksums and returns the corrected responses:
    ```bash
    pio test -e SwitchingUnit_PLUS
    ```
2.  **UART Verification:** Use a logic analyzer or serial monitor on PD5/PD6 to verify that `REQ_GET_ENERGY` returns exactly a 12-byte payload instead of 13 bytes.
3.  **Float-less Math Performance:** Verify that calculations do not introduce significant execution lag in the main superloop. The division of 64-bit integers on a 32-bit RISC-V core without hardware division (RV32EC uses software emulation for division) takes roughly 200-400 clock cycles. This is well within the 48 MHz budget when run once per 50 ms.
