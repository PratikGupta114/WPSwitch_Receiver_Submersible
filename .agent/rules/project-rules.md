---
trigger: always_on
---

# WPSwitch Receiver Submersible — Agent Project Rules

## Scope
This rule file applies to the **entire** `WPSwitch_Receiver_Submersible` repository, which contains two firmware sub-projects:
- `ControlUnit/` — ESP32, ESP-IDF, C/C++
- `SwitchingUnit/` — CH32V003, PlatformIO, bare-metal C

## Rule 1 — Read Before You Write

Before making **any** code or configuration change in either sub-project, the agent **must** read and understand the following two documents located at the repository root:

1. **`ProjectOverview.md`** — High-level description of both projects, their relationship, architecture, and the inter-unit communication protocol.
2. **`ProjectStatus.md`** — Current state of the project: what is complete, what is in progress, and what open issues exist.

These documents provide the authoritative context needed to make informed changes. Skipping this step risks introducing contradictions or regressions.

## Rule 2 — Read Sub-Project Rules and Skills

Before working on a specific sub-project, the agent **must** read that sub-project's agent configuration files:

- **ControlUnit:**
  - `ControlUnit/.agent/rules/espidf-firmware-rules.md`
  - `ControlUnit/.agent/skills/esp-idf-esp32-firmware/` (all files within)
  - `ControlUnit/ProjectOverview.md`

- **SwitchingUnit:**
  - `SwitchingUnit/.agent/rules/ch32v003-noneos.md`
  - `SwitchingUnit/.agent/skills/ch32v003-noneos-mcu/` (all files within)
  - `SwitchingUnit/skills.md`
  - `SwitchingUnit/DEVELOPER_INSTRUCTIONS.md`
  - `SwitchingUnit/ProjectOverview.md`

These files define framework-specific coding standards, build practices, hardware constraints, and gotchas that must be followed.

## Rule 3 — Update Documentation After Major Architectural Changes

After making a **major architectural change**, the agent **must** update the following documentation to reflect the new state:

### What counts as a "major architectural change"?
- Changes to algorithms (e.g., stall detection, filtering, energy calculation)
- Changes to communication protocols (e.g., new commands, events, frame format modifications)
- Creation of new source files or modules
- Addition of a new feature (e.g., new MQTT commands, new sensor support, new relay states)
- Deletion or significant refactoring of existing modules
- Changes to hardware pin assignments or peripheral configurations
- Changes to build system configuration (new build environments, new dependencies)

### Documents to update:
1. **Repository root `ProjectOverview.md`** (`WPSwitch_Receiver_Submersible/ProjectOverview.md`) — Update the high-level architecture, protocol, or feature description as needed.
2. **Repository root `ProjectStatus.md`** (`WPSwitch_Receiver_Submersible/ProjectStatus.md`) — Update the project status, mark items as completed, add new in-progress items.
3. **Individual sub-project `ProjectOverview.md`** files:
   - `ControlUnit/ProjectOverview.md` — If the change affects the Control Unit.
   - `SwitchingUnit/ProjectOverview.md` — If the change affects the Switching Unit.

### Guidelines for documentation updates:
- Keep descriptions factual and verifiable against the source code.
- When updating protocol descriptions, ensure both sides (CU and SU) are cross-referenced.
- Include timestamps or version notes when significant milestones are reached.
- Do not remove historical information; instead, mark items as resolved or superseded.
