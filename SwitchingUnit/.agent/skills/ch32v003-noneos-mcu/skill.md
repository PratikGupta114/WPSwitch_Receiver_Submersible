---
name: ch32v003-noneos-mcu
description: Deep reference for developing firmware on CH32V003A4M6 using the WCH NoneOS SDK in PlatformIO. Includes GPIO, timers, ADC, interrupts, UART/I2C/SPI, software UART, and state machine patterns.
---

# CH32V003A4M6 NoneOS MCU Skill

Use this skill whenever you are:
- Creating or modifying firmware for CH32V003A4M6 with the WCH NoneOS SDK in PlatformIO.
- Working with GPIO, timers, ADC, interrupts, or serial buses (USART, I2C, SPI).
- Designing state machines or higher‑level logic on this MCU.

## 0. Operating stance

- Act as a **senior, experienced CH32 firmware engineer** specializing in CH32V003-class devices and the WCH NoneOS SDK.
- Use strong engineering judgment around clocking, GPIO muxing, peripheral availability, interrupt behavior, timing accuracy, package limitations, and safe startup states.
- Prefer solutions that are robust on real hardware, easy to debug, and compatible with future extension.

## Build and repair workflow

Whenever you generate or modify project code:
- Build after every meaningful change whenever a local build command is available.
- If the build fails, read the exact error output, identify the most likely root cause, research unclear SDK/toolchain issues, apply a focused fix, and build again.
- Continue this loop until the build passes or an external blocker prevents further progress.
- Do not declare the task complete while build errors remain unresolved if the project can still be built locally.

## 1. Environment & Project Setup

- Framework: `framework = noneos-sdk` in `platformio.ini`.
- Board: `board = genericCH32V003A4M6` (or equivalent generic CH32V003 board).
- Include the WCH debug helpers where needed: `#include "debug.h"`.
- Always verify `SystemCoreClock` and call `SystemCoreClockUpdate()` early in `main()`.

When generating new code, scaffold files as:
- `src/board_init.c` / `.h` – clock tree, GPIO mux, peripheral clocks.
- `src/drivers/*` – peripheral abstractions.
- `src/app/*` – domain logic and state machines.

## 2. GPIO Patterns

### Basic output (LED / relay)

1. Enable GPIO clock with `RCC_APB2PeriphClockCmd`.
2. Configure pin:
   - `GPIO_Mode_Out_PP` for push‑pull.
   - `GPIO_Speed_50MHz` unless you have a reason to lower it.
3. Initialize with `GPIO_Init()` and drive a known reset value using `GPIO_WriteBit()`.

When writing code:
- Use named macros like `LED_PORT`, `LED_PIN`, `RELAY_PORT`, `RELAY_PIN` defined in a `board_pins.h`.
- Prefer small inline helpers:
  - `static inline void led_on(void) { GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET); }`
  - `static inline void led_off(void) { GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET); }`

### Input with interrupt (button / float switch)

- Configure as `GPIO_Mode_IPU` or `GPIO_Mode_IPD` depending on wiring.
- Use EXTI line with falling/rising edge trigger.
- In the EXTI ISR:
  - Clear the interrupt flag immediately.
  - Debounce in software with a timer or by ignoring events for a few milliseconds.
  - Convert to a clean event and push into an event queue or set a flag for the main loop.

## 3. Timers & Scheduling

### System tick

- Use **SysTick** or a general‑purpose timer (e.g. TIM2) to build a millisecond tick.
- In the timer ISR:
  - Increment a `volatile uint32_t` tick counter.
  - Optionally run very short periodic tasks (e.g. software timers, debounce counters).

Expose helpers:
- `uint32_t mcu_millis(void);` returning the tick counter.
- `bool mcu_elapsed(uint32_t *last, uint32_t period_ms);` that checks and updates timestamps for periodic tasks.

### PWM / capture

- Use the advanced timer for PWM (e.g. motor, pump, buzzer control).
- Keep timer configuration in one function that documents frequency and resolution.

## 4. ADC Usage

### Single conversion

- Configure ADC clock, mode, and channel sample time (e.g. `ADC_SampleTime_55Cycles5` for general use).
- Use a simple API:
  - `void adc_init(void);`
  - `uint16_t adc_read(enum adc_channel ch);`

Inside `adc_read`:
- Select the channel.
- Start conversion.
- Poll end‑of‑conversion with a timeout.
- Return the raw 10‑bit result.

### Filtering and scaling

- For noisy inputs, perform N‑sample averaging (4–32 samples) in the driver.
- Provide helpers to convert to engineering units, for example:
  - `float adc_to_voltage(uint16_t raw);`
  - `float adc_to_current(uint16_t raw);`

Document resistor ratios or CT/VT scaling clearly in comments.

## 5. USART (Hardware UART)

### Init pattern

- Enable GPIO and USART clocks.
- Configure TX as alternate‑function push‑pull, RX as input with pull‑up.
- Set baud rate, word length, stop bits, and enable RXNE interrupt.

Provide a small driver API:
- `void usart_init(uint32_t baud);`
- `void usart_write_byte(uint8_t b);` (blocking or buffered).
- `void usart_write(const uint8_t *data, size_t len);`
- `int usart_read_byte(void);` (returns `-1` if no data).

Implementation notes:
- Use a ring buffer for RX inside the USART ISR.
- Keep ISR short: read `USART_ReceiveData()`, push into buffer, clear flags.
- For debug logging, wrap `printf` so it can be compiled out in release.

## 6. I2C

### Master transfers

- Configure I2C in standard mode (100 kHz) by default.
- Expose high‑level helpers:
  - `bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value);`
  - `bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);`
  - `bool i2c_write(uint8_t addr, const uint8_t *buf, size_t len);`
  - `bool i2c_read(uint8_t addr, uint8_t *buf, size_t len);`

When generating code for I2C devices (sensors, EEPROM, OLED, etc.),
- Keep device drivers thin and layered on top of these helpers.
- Handle timeouts and NACK gracefully and surface a clear error to the caller.

## 7. SPI

> Note: Check board/package selection; some CH32V003 variants have limited/no usable SPI pins. Always verify pin mapping before using SPI.

- Provide:
  - `void spi_init(void);`
  - `uint8_t spi_transfer_byte(uint8_t b);`
  - `void spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len);`
- Manage chip‑select pins as normal GPIO, not inside the SPI driver.

For SPI‑based flash or sensors, prefer blocking transfers but minimize size to avoid long blocking times.

## 8. Software UART (Bit‑Bang)

Use only when the hardware USART is already committed or extra UART ports are needed.

Design assumptions:
- One GPIO for TX, optional one GPIO + EXTI for RX.
- A timer (or SysTick) providing bit‑time resolution.

Transmit outline:
1. Compute `bit_ticks` from timer frequency and baud.
2. For each byte:
   - Drive line low for one start bit.
   - Shift out 8 data bits (LSB first) at `bit_ticks` spacing.
   - Drive line high for at least one stop bit.

Receive outline:
- Detect falling edge (start bit) on RX via EXTI.
- Sample in the middle of each bit using a timer.
- Reconstruct the byte and push into a small RX buffer.

When generating such code, be explicit about timing assumptions and note that high CPU load or disabled interrupts can corrupt the stream.

## 9. Interrupts & Concurrency

- Mark shared variables as `volatile`.
- For multi‑byte values written in ISR and read in main, either:
  - Disable interrupts briefly around the read, or
  - Use double‑buffering or message queues.
- Keep ISR functions in a dedicated `isr.c` file or clearly sectioned in drivers.

## 10. State Machines & Application Architecture

### General pattern

- Define an enum for states, e.g. `APP_STATE_IDLE`, `APP_STATE_FILL`, `APP_STATE_DRAIN`, etc.
- Keep a struct for application context:
  - Current state.
  - Timers / timestamps.
  - Cached sensor readings.
  - Output commands.

Implement:
- `void app_init(void);`
- `void app_tick(void);` called from the main loop at a fixed rate.

Inside `app_tick`:
- Read inputs (GPIO, ADC, UART/I2C messages) via drivers.
- Compute next state based on conditions and timers.
- Write outputs via driver APIs only (no direct register access).

### Complex logic

For more complex behavior:
- Use hierarchical state machines or separate small state machines for subsystems (pump control, comms, UI).
- Keep transition rules near each other for readability.

## 11. Diagnostics & Logging

- Use a conditional debug logging macro, e.g. `MCU_LOG(...)`, that maps to `printf` over USART in debug builds and nothing in release.
- Include at least one lightweight self‑test mode that can be triggered by a GPIO strap or serial command.
- For critical faults, define a `fault_handler(reason)` that:
  - Drives outputs to safe states.
  - Optionally prints a fault code over UART.
  - Optionally blinks an LED with a pattern encoding the reason.

## 12. Firmware version management

### Version source of truth

- Treat the firmware version defined in `platformio.ini` as the single source of truth when the project already declares one there.
- Do not duplicate hard-coded version strings across multiple C files unless there is a generated or synchronized mechanism.
- Prefer a build-time define sourced from `platformio.ini`, for example through `build_flags`, rather than manually editing version strings in multiple places.

### Recommended pattern

Use `platformio.ini` to define firmware version values and expose them to C code through preprocessor macros. A typical pattern is:

```ini
[env:genericCH32V003A4M6]
platform = ch32v
board = genericCH32V003A4M6
framework = noneos-sdk
build_flags =
    -D FW_VERSION_STR="1.2.3"
    -D FW_VERSION_MAJOR=1
    -D FW_VERSION_MINOR=2
    -D FW_VERSION_PATCH=3
```

Then in firmware code:

```c
#ifndef FW_VERSION_STR
#define FW_VERSION_STR "0.0.0-unknown"
#endif

const char *fw_version_get_string(void) {
    return FW_VERSION_STR;
}
```

### Access pattern in code

- Create a dedicated module such as `src/app/fw_version.c` and `fw_version.h`.
- Expose simple accessors such as:
  - `const char *fw_version_get_string(void);`
  - `uint8_t fw_version_get_major(void);`
  - `uint8_t fw_version_get_minor(void);`
  - `uint8_t fw_version_get_patch(void);`
- Use these accessors for UART diagnostics, boot banners, protocol responses, fault reports, and manufacturing/debug commands.

### Agent behavior for version handling

When modifying a PlatformIO-based CH32 NoneOS project:
- Check `platformio.ini` before inventing any firmware version macro or version variable.
- Reuse the existing version source if it is already declared in `build_flags`, `extra_scripts`, or custom config values.
- If version information is needed and no mechanism exists, add one in `platformio.ini` first, then wire firmware code to it.
- Keep naming consistent across the project, for example `FW_VERSION_STR`, `FW_VERSION_MAJOR`, `FW_VERSION_MINOR`, and `FW_VERSION_PATCH`.
- Never leave version reporting logic dependent on manually edited duplicate strings in multiple files.

### If custom PlatformIO variables are used

Some projects keep version values in custom `platformio.ini` options and inject them through scripts. In such projects:
- Preserve the existing convention instead of replacing it blindly.
- If an `extra_scripts` build step is present, inspect it before changing version logic.
- Make the firmware read the generated macro or header, not the raw ini file directly from runtime code.

## 12. When in Doubt

When you are unsure about a register or peripheral detail:
- Prefer using the WCH SDK helper functions (`GPIO_Init`, `USART_Init`, `ADC_Init`, etc.) rather than hand‑coding register values.
- Cross‑check pin maps and peripheral availability with the CH32V003 datasheet and reference manual.

## 13. Build-first completion rule

When working inside an existing firmware project:
- Treat a clean build as part of the definition of done.
- After editing drivers, board config, ISR code, linker-sensitive code, or peripheral setup, always rebuild before considering the task finished.
- If repeated build errors point to a deeper mismatch in SDK APIs, startup files, board definitions, or package capabilities, pause feature work and resolve that structural issue first.
