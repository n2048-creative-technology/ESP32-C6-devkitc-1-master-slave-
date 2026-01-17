# AGENTS.md

This repository contains firmware and tooling for a distributed system of
wirelessly connected ESP32-C6-DevKitC-1 devices built with PlatformIO.

This file defines the responsibilities, constraints, and interaction rules
for both human contributors and AI agents working in this codebase.

---

## 1. Hardware Target

Primary target:
- Board: ESP32-C6-DevKitC-1
- MCU: ESP32-C6 (RISC-V, 160 MHz)
- Wireless: Wi-Fi 6 (802.11ax), Bluetooth LE 5.0
- Frameworks: Arduino (default), ESP-IDF (optional)

Assume limited RAM and flash. Avoid dynamic allocation in real-time code.

---

## 2. Build System

- Toolchain: PlatformIO
- Environment files: `platformio.ini`
- Supported environments:
  - `esp32c6-arduino`
  - `esp32c6-idf` (if present)

Agents must:
- Never hardcode board-specific values outside `platformio.ini`
- Respect existing build flags and linker settings
- Keep firmware size below board limits

---

## 3. Network & Communication Model

Devices form a wireless network using one or more of:
- Wi-Fi (station / softAP)
- ESP-NOW
- Bluetooth LE (GATT or broadcast)

Design assumptions:
- Devices may join or leave at runtime
- Connectivity may be intermittent
- No device is guaranteed to be online

Agents must:
- Implement retry logic
- Avoid blocking calls in network handlers
- Handle message loss gracefully
- Prefer peer liveness with hysteresis (candidate/confirmed/stale) to avoid
  disconnect flapping in lossy environments

---

## 4. Firmware Architecture

Recommended structure:
- `src/`
  - `main.cpp`
  - `network/`
  - `devices/`
  - `protocol/`
- `lib/` for reusable modules

Rules:
- Hardware access isolated from application logic
- Communication protocol code must be transport-agnostic
- All timing-critical code must be non-blocking

---

## 5. Coding Standards

- Language: C++ (Arduino-compatible subset)
- Standard: C++17 where supported
- Formatting:
  - 2-space indentation
  - No dynamic memory in ISR context
  - Explicit includes (`#include <Arduino.h>` required)

Agents must:
- Comment hardware-dependent assumptions
- Avoid macros unless strictly necessary
- Prefer compile-time constants

---

## 6. Logging & Debugging

- Default output: Serial
- Logging levels:
  - ERROR
  - WARN
  - INFO
  - DEBUG (disabled in release builds)

Rules:
- Logs must be removable via build flags
- No logging inside ISR routines
- Avoid excessive serial output in production

---

## 7. Safety & Reliability

Agents must prioritize:
- Watchdog compatibility
- Brown-out safety
- Safe reboot behavior

Never:
- Block the main loop
- Assume persistent connectivity
- Assume synchronized clocks unless explicitly implemented

---

## 8. AI Agent Behavior

AI agents working in this repository must:
- Make minimal, incremental changes
- Explain architectural decisions in comments
- Never rewrite working subsystems without explicit instruction
- Preserve backwards compatibility unless requested

When uncertain:
- Leave TODO comments instead of guessing
- Prefer clarity over optimization

---

## 9. Documentation

Required:
- README.md with setup instructions
- Inline comments for non-obvious logic
- Diagrams for network topology if complexity increases

Optional:
- `docs/` folder for protocols and architecture notes

---

## 10. Scope Boundaries

This repository does NOT include:
- Mobile or desktop UI applications
- Cloud infrastructure
- Manufacturing or provisioning tools

Do not introduce external dependencies without justification.

Exception:
- A small, local-only utility GUI is allowed for monitoring/configuration,
  and must live under a top-level `gui/` folder with minimal dependencies.
