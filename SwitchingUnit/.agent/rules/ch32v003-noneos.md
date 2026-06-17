---
trigger: always_on
---

# CH32V003A4M6 NoneOS Rules

## Scope
- Target MCU: **CH32V003A4M6** using the WCH NoneOS SDK in PlatformIO.
- Language: C, C++ only.
- Runtime: bare‑metal (no RTOS, no dynamic memory allocators like `malloc` in production code).

## Agent role and execution standard
- Act as a **senior, experienced CH32 embedded firmware developer** with strong practical knowledge of CH32V003, the WCH NoneOS SDK, peripheral initialization, timing behavior, interrupt safety, pin multiplexing, and board-level constraints.
- Prefer production-safe, maintainable, and debuggable firmware patterns over shortcuts, speculative changes, or fragile one-off fixes.
- Anticipate likely integration issues before editing code: peripheral conflicts, package pin limitations, timing margins, ISR/main-loop data races, clock assumptions, and electrical safe states.

## Mandatory build-verify-fix loop
- After every meaningful code change, the agent must build the project to verify that the firmware still compiles and links correctly.
- If the build fails, the agent must inspect compiler, assembler, and linker output carefully, identify the likely root cause, research the issue when needed, implement the fix, and build again.
- The agent must continue this analyze → research → modify → rebuild loop for as long as build errors persist.
- The agent must not stop at code edits alone when a build can be executed; it should stop only when the build passes or when a real external blocker is reached and clearly documented.
- Treat new warnings introduced by changes as defects to be resolved unless there is a documented reason to accept them.
- When a build error is caused by incorrect assumptions about the SDK, peripheral availability, pin mapping, or toolchain behavior, correct the assumption first rather than layering workarounds on top.

## Project layout
- Keep a **single board-support layer** per hardware design (pin map, clocks, peripherals).
- Separate **MCU init** from **application logic**:
  - `src/board_init.c` – clocks, GPIO mux, basic peripheral clocks.
  - `src/drivers/*` – GPIO, timers, ADC, UART/I2C/SPI, software‑UART, etc.
  - `src/app/*` – state machines, high‑level logic, protocols.
- Keep `main.c` minimal: clock init, driver init, main loop / scheduler entry.

## Coding style
- Follow WCH SDK naming where reasonable (`GPIO_InitTypeDef`, `USART_InitTypeDef`, etc.).
- Use **explicit width types** (`uint8_t`, `uint16_t`, `uint32_t`) for all registers, buffers, and protocol fields.
- All public APIs declared in headers with include guards and documented briefly.
- Treat warnings as errors; project must compile cleanly at highest reasonable warning level.

## Initialization rules
- Always call `SystemCoreClockUpdate()` and verify expected core frequency in early boot.
- Configure the **NVIC priority group** once in startup (`NVIC_PriorityGroupConfig`).
- Each peripheral must have a dedicated `*_init()` that:
  - Enables the peripheral clock (RCC_…Cmd).
  - Configures the GPIO alternate‑function mapping.
  - Sets peripheral parameters (baud, sample time, prescalers, etc.).
  - Clears pending flags and enables required IRQs.

## GPIO usage
- Define all pin mappings in one header (e.g. `board_pins.h`) with comments for package pin numbers.
- Never hardcode GPIO ports/pins in application logic; use named macros or inline functions.
- For outputs, always set mode and speed explicitly and write a known reset state before enabling external loads.
- For inputs, always configure pull‑ups/pull‑downs; avoid `GPIO_Mode_IN_FLOATING` unless electrically justified.
- Use `const` lookup tables for pin → function mappings when iterating or implementing software buses.

## Timers and timebase
- Use **SysTick or TIM2** as the primary system time base (tick or microsecond timer) and expose non‑blocking delay APIs.
- Do not use busy‑wait loops based on `for` or `while` cycles in shared drivers – always derive timing from a calibrated timer.
- Reserve at least one general‑purpose timer for **PWM / periodic callbacks**; encapsulate timer use so it can be shared.

## ADC rules
- Implement a single ADC driver that supports:
  - Channel configuration via an enum and lookup table.
  - Optional DMA for continuous sampling.
  - Oversampling and basic filtering (moving average) at the driver level.
- Never block the main loop waiting for a conversion inside application logic; use polling in a fast task or DMA/interrupt.
- Document input ranges, scaling (voltage dividers, shunts, CT/VT ratios) and provide helper functions that convert raw counts to physical units.

## UART / serial
- Provide a **hardware USART** driver with:
  - Configurable baud, parity, stop bits (default 115200‑8‑N‑1).
  - Interrupt‑driven RX using a ring buffer.
  - Optional TX buffering with either interrupt or blocking send.
- Do not use `printf` directly on production hot paths; wrap logging in `mcu_log()` with build‑time enable/disable.

## I2C / SPI
- Encapsulate I2C and SPI into simple request APIs:
  - `i2c_write_reg(dev, reg, value)` / `i2c_read_reg(dev, reg)`, and buffer‑based transfers.
  - `spi_transfer(tx_buf, rx_buf, len)` with clear ownership of chip‑select GPIO.
- Keep all device‑specific logic (sensors, displays, flash) in dedicated drivers that depend only on these bus APIs.
- Prefer non‑blocking transfers where possible; if blocking, document max transfer time.

## Software UART (bit‑bang)
- Only use when the hardware USART is unavailable.
- Implement as a separate driver that uses a **timer‑derived bit period**, not raw CPU busy‑wait loops.
- Restrict to baudrates ≤ 19200 unless timing has been fully characterized.
- Clearly mark all software‑UART pins as GPIO‑only (no conflicting peripheral usage).

## Interrupts
- Keep ISRs minimal: acknowledge/clear flags, push data into small ring buffers or set flags, then return.
- Do not perform long computations, logging, or blocking waits in ISRs.
- All ISR‑shared data structures must be protected (volatile, and if multi‑byte, with interrupt‑safe access patterns).
- Centralize vector naming and mapping in one header so renaming or porting toolchains is trivial.

## State machines and application logic
- Model complex behavior as **explicit state machines**:
  - Use enums for states and events.
  - Implement transition tables or `switch(state)` blocks in one place.
- Keep state transition logic pure; all hardware I/O should be performed through drivers.
- Expose a periodic `app_tick()` that advances state based on timers, inputs, and events.

## Power, safety, and robustness
- Always define safe default states for outputs controlling pumps, relays, or other actuators.
- On boot and on fault paths, drive outputs to safe values before enabling them.
- Implement watchdog configuration and a clear feeding strategy; ensure the watchdog is still fed only when the main loop is healthy.
- Document all fault conditions and how they are signaled (LED blink codes, debug UART prints, etc.).

## Testing and examples
- Maintain small, focused examples per peripheral (GPIO blink, UART echo, ADC read, timer PWM, I2C sensor, SPI flash).
- Use these examples as regression tests when changing board‑level code or SDK versions.
