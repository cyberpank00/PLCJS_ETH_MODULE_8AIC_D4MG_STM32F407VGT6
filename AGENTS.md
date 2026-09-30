# AGENTS.md

Firmware for the PLCJS Ethernet 8AIC module (8x 4–20 mA / 0–20 mA current
inputs via two ADS1220, STM32F407VGT6, KSZ8863 switch, Modbus TCP). This file
is the orientation map for agents; user-facing documentation lives in
`README.md` / `README_EN.md`.

Derived from the 4RTD HW2.1 firmware (fw 2.0): the ADS1220 transport, the
write-once calibration store, the compact "grouped by quantity" register map
and every shared subsystem came from there; the acquisition module (`aic/`)
is new.

## Build (CMake)

Toolchain: STM32 Arm Clang (`starm-clang`) from STM32CubeCLT, generator Ninja.
Toolchain file: `cmake/starm-clang.cmake`. Presets in `CMakePresets.json`.

```
cmake --preset Debug
cmake --build --preset Debug
```

Release: substitute `Release`. Build output:
`build/<preset>/PLCJS_ETH_MODULE_8AIC_D4MG_STM32F407VGT6.{elf,hex,bin,map}`.

No host-side unit tests. "Verified" means it compiles, and where the change is
observable it was exercised against hardware.

### Tools required
- CMake >= 3.22, Ninja
- `starm-clang` (STM32CubeCLT) on PATH. Alternative GCC toolchain at
  `cmake/gcc-arm-none-eabi.cmake`.
- Node 18+ or Python for `tools/calibrate.mjs` / `tools/calibrate.py`.

## Repository layout

| Path | Owner | Notes |
|---|---|---|
| `Application/` | hand-written | All real logic. Edit here. |
| `Core/`, `Drivers/`, `Middlewares/`, `LWIP/`, `cmake/stm32cubemx/` | STM32CubeMX | Regenerated from the `.ioc`. |
| `startup_stm32f407xx.s`, `STM32F407XX_FLASH.ld` | hand-edited | Diverged from CubeMX output — see *Linker*. |
| `tools/` | hand-written | Calibration helpers (`calibrate.mjs`, `calibrate.py`, own `README.md`). |
| `DOC/` | assets | Schematic PDF, board photo (`MAIN_BOARD.png`, used by the READMEs). |

**CubeMX regeneration hazard.** Regenerating from the `.ioc` overwrites `Core/`,
`Drivers/`, `Middlewares/`, `LWIP/` and `cmake/stm32cubemx/CMakeLists.txt`, some
of which carry hand edits outside `USER CODE` guards (notably
`LWIP/Target/ethernetif.c`, `lwipopts.h`, and the pin block in `Core/Inc/main.h`
/ `Core/Src/gpio.c`, which was edited by hand and **not** regenerated for this
board). Diff carefully afterwards.

## Module map (`Application/`)

| Module | Responsibility |
|---|---|
| `app/` | Orchestrator: boot order, factory reset, network bring-up, housekeeping loop, acquisition task. Start here. |
| `spi/` | SPI1 transport (mode 1) shared by both ADS1220. |
| `ads1220/` | ADS1220 ×2 driver: reset/config, per-mux single-shot conversion synchronised on the DRDY pins, data-rate selection, liveness. Board wiring and register values are documented in its header. |
| `aic/` | Acquisition + conversion: code → mA, per-channel calibration, fixed NAMUR-style fault thresholds, EMA smoothing, int16 reading scaled to per-channel lo/hi thresholds, channel LEDs. |
| `calstore/` | **Write-once** per-channel calibration store in Flash (8 slots). Read this file before touching calibration. |
| `temp/` | On-chip MCU temperature sensor, exposed as IR126 / HR130. |
| `modbus/modbus_app.c` | Register-map adapter. **The map is documented in the header comment of `modbus_app.h`.** |
| `modbus/modbus_tcp_server.c` | Multi-client (4 slots) TCP server on LwIP netconn; when full, the longest-silent client is evicted (newest-wins). |
| `settings/` | Flash-backed settings, CRC32-protected. |
| `discovery/` | PDP responder, UDP/20556 broadcast. |
| `net_id/` | MAC and link-local IPv4 derived from the 96-bit MCU UID. |
| `ksz8863/` | SMI/MIIM driver for the Ethernet switch. |
| `led/`, `button/` | STAT_LED state machine; FACT_RES button. |
| `fw_header/` | Firmware image header consumed by the bootloader. Module identity. |
| `third_party/nanomodbus/` | Vendored protocol library, locally patched (FC15/FC16 hardening). |

### Register map shape
Multi-channel quantities are grouped **by quantity** (8 registers or 8 pairs =
channels 0..7). `float32` is two registers, **high word first**.

- Compact block (FC03/06/16) `0..47`: `0..7` int16 reading (RO, 0..32767 =
  scale_lo..scale_hi of the channel, negative below), `8..15` scale low
  threshold µA (default 4000), `16..23` scale high threshold µA (default
  20000), `24..31` enabled (default 1), `32..39` EMA level, `40..47` reserved.
  Disabled reads 0, fault reads −32768. Threshold writes are validated
  against each other (lo < hi ≤ 25000 µA) — move `hi` first when raising
  the whole span.
- Readings (FC04) `300..355`: current f32 ×8, raw current f32 ×8, flags ×8
  (fault code in bits 15..8: 1 open, 2 over-range, 3 ADC dead), ADC code
  int32 ×8.
- Calibration coefficients (FC03/06/16): `540 + ch*4` — gain, offset (mA).
- Nominals: `620..621` R_shunt Ω, `622..623` V_REF V.
- `HR133` ADC data rate (0 = 20 SPS + FIR, 1 = 90 SPS, 2 = 330 SPS).
- `IR127` = calibration lock bitmask, bit = channel.

## Invariants

### Calibration is irreversible — treat it as destructive

`calstore/` implements a **write-once** store: each of the 8 channel slots can
be committed exactly once, because internal Flash can only clear bits without
a full sector erase.

- Modbus writes to `540 + ch*4` are a **live preview only**, and are rejected
  once the slot is locked.
- Committing is `HR131 = 0xCA00 | ch`. **This is irreversible.**
- The only undo is `calstore_erase()`, which wipes the whole sector and reverts
  all 8 slots to neutral (gain 1.0, offset 0.0). It is gated behind two
  factors: arm with `HR132 = 0xC1A5`, then physically confirm with a button
  hold within 30 s (`CAL_ERASE_ARM_WINDOW_MS` / `CAL_ERASE_CONFIRM_MS`).
- Never issue a commit or an erase while testing, and never add a code path that
  can commit without explicit operator intent.

The calibration sector is **never** erased by settings save or factory reset.
Keep it that way.

### Single sources of truth
- **Module identity** — `Application/fw_header/fw_header.h`:
  `FW_PRODUCT_ID = 0x504C0804`, `FW_HW_REVISION = 0x0101`,
  `FW_VERSION_VALUE = 0x0105`.
- **Firmware version over Modbus** — IR120/IR121 derive from `FW_VERSION_VALUE`.
- **Register map** — the header comment of `modbus_app.h`, mirrored by the
  `MB_*` constants. Keep comment and constants in step.
- **Module ID** — `MODULE_ID_08AIC = 0x08AC`, reported in IR125.
- **Fault thresholds** — `AIC_OPEN_MA` (3.6) / `AIC_OVER_MA` (21.0) in
  `aic_module.h`; deliberately constants, not registers.

### Version policy — bump the minor on every change

**Mandatory.** Every change to firmware behaviour ships with `FW_VERSION_VALUE`
in `fw_header.h` incremented by one minor (`0x0105` → `0x0106`). The version is
the operator's only way to tell which build is running on a device in the field.

- Minor bump: any firmware-only change — fixes, features, register-map
  additions, timing or conversion-maths changes.
- Major bump: only together with a `FW_HW_REVISION` major change (MCU pinout).
  OTA requires `fw_version` major == `hw_revision` major.
- Pure documentation-only commits do not need a bump.

Bump checklist: `FW_VERSION_VALUE` in `fw_header.h`, the version rows in
`README.md` and `README_EN.md`.

### Persistence and Flash sectors

| Sector | Address | Content |
|---|---|---|
| 10 | `0x080C0000` | Settings (`settings.c`) |
| 11 | `0x080E0000` | Write-once calibration (`calstore.c`) |

- `settings_t` layout is frozen; reordering or resizing requires bumping
  `SETTINGS_VERSION` (currently 2, magic `0x08AC4A57`). A mismatch silently
  reverts deployed units to factory defaults.
- The field is named `use_dhcp` but holds a tri-state net mode (static / DHCP
  / link-local). Kept for parity with the sibling modules.
- **Sector 11 conflict:** the bootloader's `flash_map.h` nominally lists sector
  11 as a third staging sector (unused — staging is sectors 8–9 only). If the
  bootloader is ever extended to use it, 8AIC (and 4RTD) calibration is
  destroyed.

### Threading
- LwIP calls must run in the tcpip thread; the live network re-apply goes through
  `tcpip_callback()`.
- Flash writes and resets requested over Modbus/discovery are deferred to the
  housekeeping loop in `app_run()` via the `*_take_pending_*()` flags.
  Calibration commits and the sector erase must run from housekeeping, never
  from the tcpip thread.
- Any loop blocking longer than the IWDG period must call
  `HAL_IWDG_Refresh(&hiwdg)`.
- All ADS1220 SPI access stays on the `AIC` acquisition task. `aic_module_tick()`
  **blocks** for four conversions (~210 ms at 20 SPS) while polling DRDY with
  `osDelay(1)`; the task recomputes its sleep from the measured scan time, so
  a scan period shorter than the scan simply runs back-to-back.

### Boot order (`app_run()`)
Two ordering constraints inherited from 12DI, both load-bearing:
- The LED task starts **before** the FACT_RES button check, otherwise the
  factory-reset blink is silently dropped.
- Factory reset writes Flash **before** the visual confirmation: the sector erase
  blocks the CPU for ~1–2 s and would freeze the blink.

## Gotchas

- **Device name is 15 chars + NUL in a fixed 16-byte field**, and the PDP
  IDENTIFY response is a fixed 38 bytes. Must stay identical across every module
  variant and ModbusTool.
- Modbus TCP serves up to 4 clients from one task (round-robin, 2 ms
  first-byte poll per idle slot). Only when all 4 slots are busy does a new
  connection evict the longest-silent client; a silent client is dropped after
  30 s. Register callbacks are shared and sequential — last write wins. Needs
  `MEMP_NUM_NETCONN/NETBUF = 8`, `MEMP_NUM_TCP_PCB = 12` in `lwipopts.h`.
- **Slots are closed with `tcp_abort` (RST), never `netconn_close` (FIN).** A
  FIN to a vanished peer leaves the pcb in FIN_WAIT_1 retransmitting for
  minutes; a handful of cable pulls exhausted the pcb pool and LwIP silently
  dropped every new SYN (PDP still answered, Modbus looked dead). The abort
  must be **synchronous under `LOCK_TCPIP_CORE()`** — with core locking the
  netconn API runs in the caller, so a queued `tcpip_callback()` abort lands
  after `netconn_delete()` freed and the next accept reused the netconn.
- The server drops clients on `g_eth_link_stable` (debounced, follows the
  netif), not on the instantaneous `g_eth_any_link_up` (LED only): the KSZ8863
  port blips link-down for ~10 s after a cable is plugged back.
- **`nmbs_server_poll()` returns `NMBS_ERROR_NONE` for "nothing arrived" too.**
  A request is counted only when a response was produced (`txbuf_len != 0`).
  Counting the idle return as a request kept `last_activity` fresh forever
  (idle-drop never fired), lit POLLING for any silent client and fed the
  comms-loss timer from a silent connection.
- STAT_LED `POLLING` is driven by request recency only (one
  `LED_POLLING_PERIOD_MS` window), no "client connected" condition.
- `ethernet_link_thread` never calls `HAL_ETH_Stop_IT/Start_IT` after init:
  the MAC↔KSZ8863 port-3 MII link is internal and always up. Only the netif
  link flag is toggled.
- HR118 multiplexes distinct magics: `0xB00B` reboot, `0xB007` bootloader,
  `0x8863` KSZ8863 switch reset. HR117 = `0xA5A5` save, HR119 = `0xDEAD`
  factory reset, HR131 = calibration commit, HR132 = calibration-erase arm.
- HR130 (on-chip temperature) is read-only despite living in holding space.
- `float32` registers are high-word-first.
- ADS1220 has **no fault register**: open/over-range are inferred from the
  calibrated current and from a saturated code (`AIC_CODE_SATURATED`); a dead
  converter is one whose DRDY never asserts or whose config readback mismatches.
  The open check is skipped when the channel's low threshold is below 3.6 mA
  (e.g. a 0–20 mA scale), because 0 mA is a valid input there.
- The full scale is `V_REF / R_shunt` ≈ 22.8 mA at 89.9 Ω — the 4–20 mA
  span sits at ~88 % of the ADC range, leaving room for the 21 mA over-range
  limit. Do not raise the shunt above ~93 Ω or the over-range limit becomes
  undetectable.
- The int16 reading clamps at ±32767 so `0x8000` stays reserved for fault.
- Production pinout (matches DOC/ETH_MODULE_8AIC_HW1.1_FW1.3.SCH, a PDF):
  ADC0_CS PA9, ADC1_CS PD8, ADC0_DRDY PD10, ADC1_DRDY PD9, LEDs
  PB15/PB14/PB10/PE15..PE11. RMII/ETHINT unchanged.

## Linker / memory contract with the bootloader

`STM32F407XX_FLASH.ld` is **not** a stock CubeMX script: `FLASH` origin is
`0x08040000` (256 K application slot — the image only runs via the bootloader),
and `RAM` length is `0x1FFF0` so the top 16 bytes can hold the no-init
boot-request cell at `0x2001FFF0` (`BOOT_REQUEST_MAGIC = 0xB007CAFE`).
`.fw_header` is padded to offset `0x200`.

`fw_header_t` must stay byte-identical to `fw_header_t` in the bootloader's
`Application/validate/app_validate.h` (28 bytes, packed). CRC32 and image size
travel in OTA metadata, not the header.

OTA acceptance: `product_id` exact match **and** `hw_revision` major byte match.

## Multi-repo workspace

| Repo | Role |
|---|---|
| `PLCJS_ETH_MODULE_8AIC_D4MG_...` | This module — `0x504C0804` / IR125 `0x08AC`. |
| `PLCJS_ETH_MODULE_4RTD_D4MG_...` | 4x RTD, `0x504C0403` / `0x04D1`. Direct ancestor (ADS1220 driver, calstore, map shape). |
| `PLCJS_ETH_MODULE_12DI_D4MG_...` | 12 discrete inputs, `0x504C1201` / `0x12D1`. Origin of the shared subsystems. |
| `PLCJS_ETH_MODULE_12DQ_D4MG_...` | 12 discrete outputs, `0x504C1202` / `0x12D0`. |
| `BOOTLOADER_PLCJS_ETH_MODULE_STM32F407VGT6` | Shared bootloader. Owns `flash_map.h`, `app_validate.h`, `scripts/variants.csv` (`8aic,0x504C0804,0x010101`). |
| `PLCJS_Module_ModbusTool` | Qt6/C++17 desktop client. `build8AIC()` in `src/maps/ModuleMaps.cpp` mirrors this module's register map. |

**`Application/` subsystems are copy-pasted between firmware variants, not shared
via a submodule.** A fix here is not a fix elsewhere, and vice versa.

Cross-repo contracts that must change in lockstep:
- **Wire format** (PDP frame layout, 38-byte IDENTIFY, 16-byte name) — every
  firmware + `Pdp.cpp`.
- **`fw_header_t` layout, `FW_HEADER_OFFSET`, `BOOT_REQUEST_FLAG_ADDR`/`MAGIC`,
  flash map** — every firmware + bootloader + both linker scripts.
- **product_id** — `fw_header.h` here and `scripts/variants.csv` in the
  bootloader. `variants.csv` uses the 3-byte hw encoding `0x010101`; firmware
  headers use the 2-byte `0x0101`.
- **Register map changes** — `modbus_app.h` here and `build8AIC()` in
  `ModuleMaps.cpp`, or the tool shows stale registers.

## Maintaining this file

`AGENTS.md` is a living document, not a one-time write. Update it **in the same
commit** as the change it describes. Touch it when:

- an invariant, gotcha or threading rule is added or changes — especially the
  calibration write-once / sector-11 rules, which are safety-critical;
- a module is added, removed or repurposed (`Application/` map);
- the build procedure, toolchain or linker contract changes;
- a register-map change alters the header comment of `modbus_app.h`;
- `FW_VERSION_VALUE` is bumped and the version-policy text needs the new
  example value;
- a cross-repo contract changes (PDP wire format, `fw_header_t`, flash map,
  `product_id`) — update the *Multi-repo* section here **and** the corresponding
  section in the sibling repo(s).
