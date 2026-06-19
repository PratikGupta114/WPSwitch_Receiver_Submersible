---
trigger: always_on
---

# ESP-IDF Firmware Rules

## Scope
- Target: ESP32/ESP32-C2/C3/S2/S3 class SoCs using the **ESP-IDF** framework and CMake build system.
- Language: C (and C++ where explicitly required).
- OS: FreeRTOS as provided by ESP-IDF; no alternative RTOS layers.
- Build tools: `idf.py` front-end over CMake (or equivalent CMake-based integrations).

## Agent role and execution standard
- Act as a **senior, experienced ESP-IDF firmware engineer** with strong practical knowledge of ESP32-class SoCs, FreeRTOS, ESP-IDF drivers, and the CMake-based build system.
- Prefer production-safe, maintainable, and debuggable designs over shortcuts, speculative changes, or fragile one-off fixes.
- Anticipate typical ESP-IDF pitfalls: wrong pin mux, GPIO vs. RTC GPIO confusion, task stack sizes, ISR vs. task context, incorrect interrupt affinity, and sdkconfig option interactions.

## Mandatory build-verify-fix loop
- After every meaningful code or build-system change, the agent must run a build (`idf.py build` or equivalent) to verify that the project still compiles, links, and generates a binary.
- If the build fails, the agent must:
  - Inspect the CMake/configuration, compiler, assembler, and linker output carefully.
  - Identify the likely root cause (include paths, missing components, wrong target, API changes, sdkconfig mismatches, etc.).
  - Research unclear ESP-IDF API or build-system issues when necessary.
  - Implement focused fixes (not broad search-and-replace edits) and build again.
- The agent must continue this analyze → research → modify → rebuild loop as long as build errors persist.
- Do not treat edits as “done” while the project fails to build, unless a real external blocker prevents running the build (for example, missing toolchain or environment).
- Treat new warnings introduced by changes as defects to be resolved or explicitly documented and justified.

## Project layout
- Follow the standard ESP-IDF project structure:
  - Root directory with `CMakeLists.txt`, `sdkconfig`, and optional `components/`.
  - `main/` directory containing `main.c` / `main.cpp` and `main/CMakeLists.txt` with `idf_component_register()` listing sources and include dirs.
  - `components/<name>/` for reusable modules, each with its own `CMakeLists.txt`, `include/`, and `src/` folders.
- Keep `main/` focused on startup and high-level orchestration; move reusable logic into components.
- Avoid duplicating the ESP-IDF `components/` tree inside the project; depend on official components unless there is a clear need to fork.

## CMake and configuration
- In the root `CMakeLists.txt`, define project-level metadata and then include ESP-IDF’s project CMake:
  - Set `PROJECT_NAME` and, where appropriate, `PROJECT_VER`.
  - Include `$ENV{IDF_PATH}/tools/cmake/project.cmake` and call `project(<name>)` only via ESP-IDF’s documented pattern.
- In each component’s `CMakeLists.txt`:
  - Use `idf_component_register()` to declare `SRCS`, `INCLUDE_DIRS`, `PRIV_REQUIRES`, and `REQUIRES`.
  - Keep component dependencies minimal and acyclic.
- Do not hard-code toolchain paths or IDF internals; rely on `idf.py` and the environment.
- Prefer `sdkconfig` options and Kconfig symbols (`CONFIG_*`) for tunable parameters instead of hand-maintained `#define` values in headers.

## Firmware versioning
- Use ESP-IDF’s built-in **application version** mechanism instead of ad hoc version strings.
- Prefer setting `PROJECT_VER` in the root `CMakeLists.txt` (for example, `set(PROJECT_VER "1.2.3")` before including `project.cmake`).
- For more complex setups, rely on:
  - `CONFIG_APP_PROJECT_VER_FROM_CONFIG` / `CONFIG_APP_PROJECT_VER` in `sdkconfig`, or
  - `version.txt` / Git-based versioning, if already configured in the project.
- Do not scatter manual version strings across multiple modules; treat the configured application version as the single source of truth.

## Accessing version information in code
- Access the running application’s description via `esp_app_get_description()` and use its `version`, `project_name`, and related fields when reporting firmware info.
- For simple prints, use `esp_get_idf_version()` to include the ESP-IDF version alongside the app version.
- Do not bypass ESP-IDF’s app description block with custom linker tricks unless there is a clear compatibility reason.

## Coding style
- Follow the official ESP-IDF style guide as closely as practical:
  - 4 spaces per indentation level; no hard tabs.
  - Consistent brace placement and spacing.
  - Clear naming with component or module prefixes.
- Use `static` for symbols with file-local scope.
- Use `const` and `enum` instead of `#define` where appropriate.
- Use `ESP_LOGx` and `ESP_ERROR_CHECK()` / error codes rather than `printf` and silent failures.

## GPIO usage
- Initialize GPIO using `gpio_config_t` and `gpio_config()`; do not manipulate GPIO registers directly.
- Centralize pin assignments in a board header (for example, `board_pins.h`) with comments for module/connector labels.
- Use `gpio_reset_pin()` before configuring a pin to a new function when changing designs.
- Keep track of SoC-specific pin limitations (input-only pins, strapping pins, flash pins) and avoid assigning them to conflicting functions.

## UART, I2C, SPI, and ADC
- Use the official ESP-IDF driver APIs:
  - UART: `uart_driver_install()`, `uart_param_config()`, `uart_set_pin()`, `uart_read_bytes()`, `uart_write_bytes()`.
  - I2C: `i2c_new_master_bus()`, `i2c_master_bus_add_device()`, `i2c_master_transmit()` / `i2c_master_receive()` (or legacy driver APIs, depending on IDF version).
  - SPI: `spi_bus_initialize()`, `spi_bus_add_device()`, `spi_device_transmit()`.
  - ADC: configure channels and attenuation via the ADC driver before reading.
- Encapsulate bus access in dedicated modules (for example, `i2c_bus.c`, `uart_console.c`) and build sensor or protocol drivers on top of these modules.
- For software/bit-banged protocols, use timers and GPIO APIs rather than busy loops wherever timing allows.

## Tasks, state machines, and concurrency
- Design behavior around FreeRTOS tasks and queues, not long polling loops:
  - Keep individual tasks cohesive (peripherals, protocols, or high-level subsystems).
  - Use queues, semaphores, and event groups to communicate between tasks.
- For complex logic, implement explicit state machines:
  - Use enums for states and events.
  - Keep transition logic centralized and separate from IO details.
- Avoid blocking calls in high-priority tasks; ensure that time-critical work is short and bounded.
- Use dedicated ISR handlers that defer heavier work to tasks via queues or direct-to-task notifications.

## Error handling and robustness
- Wrap all driver calls with error checking (for example, `ESP_ERROR_CHECK()` in debug builds, or manual checks with well-defined fallback behavior).
- Use the IDF logging system and ensure log tags identify the component or subsystem.
- Prefer fail-safe defaults for outputs controlling relays, actuators, or external power stages.
- Use watchdogs judiciously and ensure feeding logic reflects actual liveness of critical tasks.

## Testing and examples
- Maintain small, focused example configurations for:
  - GPIO (input, output, interrupts).
  - UART console and binary protocols.
  - I2C/SPI sensor or peripheral drivers.
  - ADC sampling and scaling.
- Use these examples as regression tests when upgrading the ESP-IDF version or refactoring the project structure.
