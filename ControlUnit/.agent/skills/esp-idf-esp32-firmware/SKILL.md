---
name: esp-idf-esp32-firmware
description: Guidance for building robust ESP-IDF firmware (ESP32-class SoCs) using FreeRTOS, CMake, components, GPIO/ADC/UART/I2C/SPI drivers, and application versioning.
---

# ESP-IDF Firmware Skill

Use this skill when working on firmware for ESP32/ESP32-C2/C3/S2/S3 using ESP-IDF and the CMake-based build system. It covers project structure, components, GPIO, timers, ADC, UART/I2C/SPI, tasks and state machines, and firmware version access.

## 0. Operating stance

- Act as a **senior, experienced ESP-IDF firmware engineer** with deep familiarity with:
  - ESP32-class SoC peripherals and pin mux.
  - FreeRTOS tasking model, queues, semaphores, event groups.
  - ESP-IDF driver APIs (GPIO, UART, I2C, SPI, ADC, timers, NVS, Wi-Fi/BLE where relevant).
  - The ESP-IDF CMake build system, components, and `idf.py` workflows.
- Prefer solutions that are robust on real hardware, easy to debug and extend, and aligned with Espressif’s style and APIs rather than ad hoc register pokes.

## Build and repair workflow

Whenever you generate or modify ESP-IDF project code:
- Build after every meaningful change (`idf.py build` or equivalent).
- If the build fails, read the full error output (including CMake configuration errors) and identify the most likely root cause.
- Research unclear errors (for example, changed APIs between IDF versions, missing Kconfig options, or component dependencies) and apply targeted fixes.
- Repeat this analyze → fix → rebuild loop until the build passes or a real external blocker stops further progress.
- Do not treat tasks as complete while build errors persist.

## 1. Environment & project setup

### Project structure

A typical ESP-IDF project has:
- Root `CMakeLists.txt` with project-level configuration.
- `main/` directory with `main.c` or `main.cpp` and `main/CMakeLists.txt` using `idf_component_register()`.
- Optional `components/` directory with custom components, each containing:
  - `CMakeLists.txt`.
  - `include/` (public headers).
  - `src/` (implementation files).
- `sdkconfig` and optionally `sdkconfig.defaults` for configuration.

When generating or editing code:
- Keep `main/` focused on startup logic, high-level orchestration, and top-level state machines.
- Move reusable logic (drivers, protocol stacks, libraries) into components so they can be reused and unit-tested.
- Use `idf.py` (or the ESP-IDF VS Code extension) for configuration, building, flashing, and monitoring.

### CMake basics

- In the root `CMakeLists.txt`, set project metadata and then include the ESP-IDF project CMake:
  - `cmake_minimum_required(VERSION 3.xx)`.
  - Optionally set `PROJECT_VER` (see version section below).
  - `include($ENV{IDF_PATH}/tools/cmake/project.cmake)`.
  - `project(<project_name>)`.
- In `main/CMakeLists.txt`, register sources and include dirs:
  - `idf_component_register(SRCS "main.c" "other.c" INCLUDE_DIRS ".")`.
- In component `CMakeLists.txt`, declare `SRCS`, `INCLUDE_DIRS`, `PRIV_REQUIRES`, and `REQUIRES` and then call `idf_component_register()`.

## 2. GPIO patterns

### Basic output (LED / relay)

1. Define GPIO numbers and masks in a board header, e.g. `BOARD_LED_GPIO`, `BOARD_RELAY_GPIO`.
2. In initialization code, zero-initialize `gpio_config_t` and configure the pin:
   - `io_conf.intr_type = GPIO_INTR_DISABLE;`
   - `io_conf.mode = GPIO_MODE_OUTPUT;`
   - `io_conf.pin_bit_mask = 1ULL << BOARD_LED_GPIO;`
   - `io_conf.pull_up_en = 0;`, `io_conf.pull_down_en = 0;` (or as needed).
   - Call `gpio_config(&io_conf);`.
3. Use `gpio_set_level(BOARD_LED_GPIO, 0/1)` to drive outputs.

### Input with interrupt (button / sensor)

- Configure `gpio_config_t` with `GPIO_MODE_INPUT` (plus pull-ups/pull-downs as required).
- Set interrupt type with `gpio_set_intr_type(gpio_num, GPIO_INTR_POSEDGE/NEGEDGE/ANYEDGE)`.
- Install ISR service (for example, via `gpio_install_isr_service()`) and add an ISR handler with `gpio_isr_handler_add()`.
- In the ISR:
  - Read and clear the interrupt source.
  - Push an event to a queue or set a flag; do not perform long computations in the ISR.

Always check the specific SoC’s GPIO matrix and pin limitations (input-only pins, strapping pins, flash pins, etc.) before assigning functions.

## 3. Tasks and scheduling

### Task structure

- Create tasks with `xTaskCreatePinnedToCore()` or `xTaskCreate()` and give them descriptive names.
- Choose stack sizes conservatively at first, then tune based on stack high-water marks.
- Use task priorities to reflect urgency; avoid running everything at high priority.

### Timing and delays

- Use `vTaskDelay()` or `vTaskDelayUntil()` with `pdMS_TO_TICKS(ms)` for non-blocking delays inside tasks.
- Avoid busy-wait loops for timing; instead, design tasks around periodic wake-ups or event notifications.

### State machines

- Represent application logic as explicit state machines:
  - Use an enum for states and events.
  - Keep a context struct holding current state, timers, and cached IO.
- Implement a periodic `app_task()` that:
  - Receives events (queue, event group, or direct notifications).
  - Updates the state based on inputs and time.
  - Issues commands through drivers (GPIO, UART, I2C, etc.).

## 4. UART

### Initialization pattern

- Define a `uart_config_t` with baud rate, data bits, parity, stop bits, and flow control.
- Call `uart_param_config(uart_num, &config)`.
- Set TX/RX pins with `uart_set_pin(uart_num, tx_pin, rx_pin, RTS, CTS)`.
- Install the driver with `uart_driver_install()` and configure RX/TX buffers.

### Usage notes

- Encapsulate UART behavior in a module such as `uart_console.c` or `uart_link.c`.
- For debug logging, rely on ESP-IDF logging (`ESP_LOGI/W/E`) rather than custom serial prints.
- For binary protocols, design framing (length, checksum, start/end markers) and implement parsing in a dedicated task.

## 5. I2C

### Modern I2C driver (IDF v5+)

- Configure the I2C bus with `i2c_new_master_bus()` using `i2c_master_bus_config_t`.
- Add devices via `i2c_master_bus_add_device()` with `i2c_device_config_t`.
- Use `i2c_master_transmit()` / `i2c_master_receive()` for transactions.

### Legacy driver

- When constrained by older IDF versions, use the legacy driver (`i2c_param_config`, `i2c_driver_install`, command links, etc.).

For device drivers (sensors, expanders, etc.):
- Implement thin wrappers on top of the bus API.
- Handle NACKs and timeouts gracefully.

## 6. SPI

- Initialize the SPI bus via `spi_bus_initialize()` with appropriate pins and flags.
- Add devices via `spi_bus_add_device()` with `spi_device_interface_config_t`.
- Exchange data using `spi_device_transmit()` or queued transactions.
- Manage chip-select lines carefully; either let the driver handle CS or control it manually with GPIO when needed.

## 7. ADC

- Configure the ADC unit and channel (width, attenuation) using the ESP-IDF ADC driver.
- For single conversions, use the appropriate blocking read function.
- For periodic sampling, consider DMA or continuous-mode APIs where available.
- Apply scaling and calibration (voltage, current, temperature) based on reference voltage, attenuation, and external resistor networks.

## 8. Interrupts and concurrency

- Keep ISRs short: clear flags, capture minimal data, and notify tasks.
- Use queues, semaphores, or direct-to-task notifications to hand work from ISRs to tasks.
- Mark shared variables accessed from ISRs as `volatile` and protect multi-byte accesses.
- For critical sections in tasks, use FreeRTOS primitives (`taskENTER_CRITICAL` / `taskEXIT_CRITICAL`) or mutexes as appropriate.

## 9. Logging and diagnostics

- Use ESP-IDF logging macros (`ESP_LOGE/W/I/D/V`) with clear tags per module.
- Avoid printing from ISRs; instead, signal tasks that perform logging.
- Provide at least one command or serial console output that prints:
  - Project name and version.
  - Build date/time.
  - ESP-IDF version.
  - Target SoC.

## 10. Configuration and NVS

- Use `menuconfig` (`idf.py menuconfig`) and Kconfig options for tunable parameters.
- Access configuration options via `CONFIG_*` macros in code.
- For runtime-persistent configuration, use NVS (non-volatile storage) and define a clear migration strategy when structures change.

## 11. Firmware version management

### Application version source

- Use ESP-IDF’s application version mechanism instead of adhoc constants.
- The application version is obtained in the following order (conceptually):
  - If `CONFIG_APP_PROJECT_VER_FROM_CONFIG` is set, `CONFIG_APP_PROJECT_VER` from `sdkconfig` is used.
  - Else, if `PROJECT_VER` is set in `CMakeLists.txt`, its value is used.
  - Else, if `version.txt` exists at the project root, its contents are used.
  - Else, if the project is in a Git repo, `git describe` is used.
  - Otherwise, the version defaults to a fallback value (for example, "1").

### Setting the version

- In the root `CMakeLists.txt`, you can set:

```cmake
set(PROJECT_VER "1.2.3")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_app)
```

- Alternatively, enable `CONFIG_APP_PROJECT_VER_FROM_CONFIG` in `menuconfig` and set `CONFIG_APP_PROJECT_VER`.

### Accessing version in firmware

- Use `const esp_app_desc_t *desc = esp_app_get_description();` to obtain a pointer to the running app’s description.
- Use `desc->version`, `desc->project_name`, `desc->time`, `desc->date`, and `desc->idf_ver` to report firmware and build details.
- For logging the ESP-IDF version alone, use `esp_get_idf_version()`.

### Agent behavior for version handling

When working on an ESP-IDF project:
- Check for existing version mechanisms (CMake `PROJECT_VER`, `sdkconfig` options, `version.txt`, or Git-based versioning) before inventing new ones.
- Reuse and extend the existing mechanism rather than hardcoding new version strings in multiple C files.
- When adding version reporting (for example, via UART console or CLI), read from the app description (`esp_app_get_description()`) instead of duplicating literals.
- For OTA and upgrade-downgrade logic, compare firmware versions using the configured app version rather than manually maintained strings.

## 12. Build-first completion rule

When modifying ESP-IDF firmware:
- Treat a clean `idf.py build` as part of the definition of done.
- After changes to drivers, component dependencies, CMakeLists, sdkconfig, or core application code, always rebuild before considering the task finished.
- If repeated build errors suggest a structural mismatch (wrong IDF version, incompatible APIs, misconfigured components), pause feature development and resolve those issues first.
