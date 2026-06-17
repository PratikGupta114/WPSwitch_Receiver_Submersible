# Developer Guidelines & Skills: CH32V003A4M6 Firmware Development

This file serves as a reference manual and set of rules for any AI agent or developer implementing firmware for the **CH32V003A4M6** microcontroller in this project workspace.

---

## 1. Hardware Specifications & Constraints

* **Core**: QingKe 32-bit RISC-V2A (RV32EC Instruction Set)
  * *Note*: The "E" extension restricts the registers to 16 general-purpose registers (`x0` through `x15`) instead of 32. Heavy stack usage or deep recursion must be strictly avoided.
* **Frequency**: Up to 48 MHz (using HSI + PLL)
* **Flash Memory**: 16 KB (16384 bytes)
* **SRAM**: 2 KB (2048 bytes)
  * *Critical Constraint*: Avoid dynamic memory allocation (`malloc`, `free`) entirely. All variables, buffers, and state trackers should be statically allocated.
* **Package**: SOP16 (16 pins, 14 usable GPIOs)
* **Debugging & Upload Interface**: Single-Wire Interface (SWIO) on `PD1`, typically programmed via WCH-LinkE (must be in RISC-V mode).

---

## 2. PlatformIO Configuration

The project is configured under PlatformIO as follows in [platformio.ini](file:///home/pratik/WPSwitch_Receiver_Submersible/SwitchingUnit/platformio.ini):

```ini
[env:genericCH32V003A4M6]
platform = ch32v
board = genericCH32V003A4M6
framework = noneos-sdk
```

* **Framework (`noneos-sdk`)**: This is the bare-metal WCH Standard Peripheral Library (SPL), containing standard driver wrappers (`ch32v00x_gpio.h`, `ch32v00x_usart.h`, etc.).

---

## 3. Peripheral Initialization Code Snippets (WCH SPL / NoneOS-SDK)

### A. System & Clock Setup
Always initialize the priority group (only Group 1 is valid/supported for CH32V003 due to 2-bit interrupt priority limitations in PFIC) and set up the system clock/delay utilities at the start of `main()`:

```c
#include "debug.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    
    // ... application logic ...
}
```

### B. GPIO Configuration
Before modifying a GPIO port, its peripheral clock must be enabled via the APB2 bus command:

```c
// Enable GPIO Clock
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

// Initialize Pin (Example: PD0 as Push-Pull Output)
GPIO_InitTypeDef GPIO_InitStructure = {0};
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOD, &GPIO_InitStructure);
```

#### GPIO Modes:
* Output Push-Pull: `GPIO_Mode_Out_PP`
* Output Open-Drain: `GPIO_Mode_Out_OD`
* Alternate Function Push-Pull (e.g. UART TX): `GPIO_Mode_AF_PP`
* Alternate Function Open-Drain (e.g. I2C): `GPIO_Mode_AF_OD`
* Floating Input: `GPIO_Mode_IN_FLOATING`
* Input Pull-Up/Pull-Down: `GPIO_Mode_IPD` (Pull-down) / `GPIO_IPU` (Pull-up) or configuration via `GPIO_CFGLR_IN_PUPD` inside the register.

#### Pin Level Manipulation:
```c
GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);   // Write HIGH
GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET); // Write LOW
uint8_t pin_state = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0); // Read State
```

### C. USART Configuration
The CH32V003 has one hardware USART interface (`USART1`).
Default Pinout for USART1:
* **TX**: `PD5`
* **RX**: `PD6`

```c
#include "ch32v00x_usart.h"

void USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    // TX pin (PD5) - Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // RX pin (PD6) - Floating Input / Input Pull-Up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Configure USART1
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    // Enable USART1
    USART_Cmd(USART1, ENABLE);
}
```

#### Polled Data Handling:
```c
// Send a Byte
void USART1_SendByte(uint8_t data)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, data);
}

// Receive a Byte (blocking)
uint8_t USART1_ReceiveByte(void)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USART1);
}
```

---

## 4. Key Implementation Rules & Troubleshooting Gotchas

### Rule 1: HSI Clock Drift vs UART Timings
* **Problem**: The chip's internal 48MHz high-speed internal (HSI) oscillator is uncalibrated at the factory and drifts with temperature and voltage. This causes standard baud rate timings to fail.
* **Rule**: When using asynchronous software serial or tight UART timing loops without an external crystal, you **must** implement a dynamic calibration loop.
* **Calibration Pattern**: Measure the incoming start bit's minimum width (low pulse duration) in SysTick clock cycles over a series of transitions, and set the bit duration to this measured minimum.

### Rule 2: Interrupt Grouping
* **Problem**: PFIC (Programmable Fast Interrupt Controller) on CH32V003 only supports a maximum of 2 bits for priority (Preemption and Subpriority). Specifying `NVIC_PriorityGroup_2` or higher will fail compilation or cause undefined behavior.
* **Rule**: Always use `NVIC_PriorityGroup_1` when configuring interrupt priority groups.

### Rule 3: Single-Wire Debug (SWIO) Lockup
* **Problem**: `PD1` is the Single-Wire Debug (SWIO) pin used to flash the chip. If you configure `PD1` as a general-purpose output or change its mode register in your user code, the WCH-LinkE programmer will fail to connect/upload subsequently.
* **Rule**: Never reconfigure `PD1` as a regular GPIO unless debug mode is completely disabled and not needed. If locked out, you must use the physical "Mode" button on the WCH-LinkE programmer to wipe/reset the chip's option bytes or power-cycle the board during link utility connection.

---

## 5. Standard CLI Operations

Use these command patterns during project maintenance:

* **Compile Code**:
  ```bash
  ~/.platformio/penv/bin/pio run
  ```
* **Upload/Flash**:
  ```bash
  ~/.platformio/penv/bin/pio run --target upload
  ```
* **Clean Build Directories**:
  ```bash
  ~/.platformio/penv/bin/pio run --target clean
  ```
