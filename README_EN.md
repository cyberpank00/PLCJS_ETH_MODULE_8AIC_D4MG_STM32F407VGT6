# PLCJS_ETH_MODULE_8AIC_D4MG_STM32F407VGT6

<p align="center">
  <img src="DOC/MAIN_BOARD.png" alt="PLCJS Ethernet 8AIC board (HW1.1)" width="600">
</p>

<p align="center"><strong>8AIC module board, HW1.1</strong></p>

Firmware for the PLCJS Ethernet module with 8 analog current inputs
(4–20 mA / 0–20 mA), based on `STM32F407VGT6`.

Eight channels are measured through two isolated 24-bit `ADS1220` ADCs (four
single-ended inputs each via the internal multiplexer), published over
`Modbus TCP`; settings live in internal Flash, calibration in a separate
write-once Flash sector. Main `STAT_LED` plus eight per-channel LEDs, factory
reset, OTA via the Ethernet bootloader. Infrastructure (network, Modbus,
settings, LED, bootloader, ADS1220 driver, write-once calibration) is
inherited from the `4RTD` HW2.1 variant.

## Hardware

- MCU `STM32F407VGT6`; Ethernet `KSZ8863` (RMII).
- 2× `ADS1220`, isolated (CA-IS3xxx), DRDY wired; `REF5020` 2.048 V reference
  on REFP0/REFN0.
- Input: 0.1 A fuse → shunt **91 Ω 1206** (firmware nominal 89.9 Ω,
  calibrated) → 1 kΩ + 100 nF → AINx; BAV99 clamps, SMAJ24CA TVS.

Pinout: SPI1 SCLK=PA5, MISO=PA6, MOSI=PB5. ADC0_CS=PA9, ADC1_CS=PD8,
ADC0_DRDY=PD10, ADC1_DRDY=PD9. Channel LEDs AI0..7 = PB15, PB14, PB10, PE15,
PE14, PE13, PE12, PE11. STAT_LED=PE9, FACT_RES=PE10, ETHRST=PD11, ETHINT=PB1.
ADC0 serves AI0..AI3, ADC1 AI4..AI7. SPI mode 1, ~2.6 MHz.

### ADS1220 configuration

| Reg | Value | Fields |
|---|---|---|
| 0 | `0x81 \| mux<<4` | MUX AINx/AVSS, gain 1, PGA bypass |
| 1 | `DR<<5` | 20/90/330 SPS, normal, single-shot |
| 2 | `0x50` / `0x40` | ext ref REFP0/REFN0, 50+60 Hz FIR (20 SPS only), IDAC off |
| 3 | `0x00` | — |

Per mux input: WREG MUX → START on both chips → wait DRDY → RDATA. A full
8-channel scan is four conversions (~210 ms at 20 SPS, ~50 ms at 90 SPS,
~16 ms at 330 SPS).

```
I_raw = code / 2^23 × V_REF / R_shunt        (20 mA ≈ 1.8 V ≈ 88 % FS)
I     = gain × I_raw + offset                 (per-channel calibration)
```

Channel LED: solid = active, off = disabled, fast blink (5 Hz) = fault (open
loop < 3.6 mA on live-zero scales, over-range > 21 mA / saturated ADC, ADC not
responding).

## Channel scale and fault detection

Each channel's scale is two thresholds in µA — low (`8 + ch`, default 4000)
and high (`16 + ch`, default 20000). The int16 reading (`0 + ch`) is scaled
automatically: low → `0`, high → `32767`, negative below low. For a 0–20 mA
scale write low = `0`.

| Condition | Fault |
|---|---|
| low threshold ≥ 3600 µA **and** `I < 3.6 mA` | open loop (code 1) |
| `I > 21 mA` or saturated ADC | over-range (code 2) |
| no DRDY / config readback mismatch | ADC not responding (code 3) |

Fault limits are constants (`AIC_OPEN_MA`, `AIC_OVER_MA` in `aic_module.h`).
Max measurable current at 89.9 Ω ≈ 22.8 mA.

## Calibration

Per-channel linear model `I = gain·I_raw + offset` fitted over ≥ 2 points from
a precision current source (tools in `tools/`). Accuracy budget over
−20…+70 °C after calibration at ~25 °C: shunt 25 ppm/°C 0.113 %, shunt
self-heating 0.023 %, REF5020 0.014 %, ADC/divider ~0.01 % → ≈ 0.16 % worst
case (RSS ≈ 0.12 %). The shunt must be thin-film, 0.1 %, ≤ 25 ppm/°C, 1206.

### Write-once

Flash sector 11, never erased in normal operation. Each of the 8 channel slots
commits **exactly once**: write `gain`/`offset` to `540 + ch×4` (live
preview) → write `0xCA00 | ch` to register `131` (**CAL COMMIT**). Lock
status: input register `127` (bit = channel). Emergency erase: `0xC1A5` to
register `132`, then hold `FACT_RES` ~3 s within 30 s.

## Modbus register map

Floats are float32, **high word first**. Multi-channel quantities are grouped
by quantity: 8 consecutive registers (or pairs) = channels 0..7.

### Compact block — Holding Registers (FC03/06/16), 0..47

| Reg | Description |
|---|---|
| 0..7 | **reading, int16 (RO)**: `0..32767` = `[low … high]` threshold of the channel, negative below low (clamped ±32767); disabled `0`; fault `−32768` |
| 8..15 | scale low threshold, µA (default `4000`; 0..25000, < high) |
| 16..23 | scale high threshold, µA (default `20000`; ≤ 25000, > low) |
| 24..31 | enabled (0/1, default `1`) |
| 32..39 | smoothing EMA (0 off, 1 = 1/4, 2 = 1/8, 3 = 1/16) |
| 40..47 | reserved |

### Readings — Input Registers (FC04), 300..355

| Reg | Type | Description |
|---|---|---|
| 300..315 | float32 ×8 | current, mA (calibrated, smoothed); NaN on fault/disabled |
| 316..331 | float32 ×8 | raw current, mA (calibration input) |
| 332..339 | u16 ×8 | flags: bit0 enabled, bit1 valid, bit2 fault; bits15..8 fault code (1 open, 2 over, 3 ADC) |
| 340..355 | int32 ×8 | 24-bit signed ADC code |

Global: `120` fw major, `121` fw minor, `122/123` uptime s, `125` module id
(`0x08AC`), `126` MCU temperature (signed 0.1 °C), `127` calibration lock mask.

### Global settings — Holding Registers

| Reg | Description |
|---|---|
| 100 | scan period, ms (50..5000) |
| 101 | LED mode (0/1/2) |
| 102 | Modbus slave id | 103 | Modbus TCP port |
| 104..107 | static IP | 108..111 | netmask | 112..115 | gateway |
| 116 | net mode: `0` static / `1` DHCP / `2` link-local (default `2`) |
| 117 | SAVE (`0xA5A5`) | 118 | REBOOT (`0xB00B`) / BOOTLOADER (`0xB007`) / KSZ8863 reset (`0x8863`) |
| 119 | FACTORY RESET (`0xDEAD`) | 130 | MCU temperature (RO) |
| 131 | CAL COMMIT (`0xCA00 \| ch`) | 132 | CAL ERASE ARM (`0xC1A5`) |
| 133 | **ADC rate**: `0` = 20 SPS + 50/60 Hz FIR (default), `1` = 90 SPS, `2` = 330 SPS |

Calibration coefficients (float32), base `540 + ch×4`: `+0..1` gain,
`+2..3` offset (mA). Nominals: `620..621` R_shunt Ω (89.9), `622..623`
V_REF V (2.048).

## Flash layout

| Region | Address | Size | Purpose |
|---|---:|---:|---|
| Bootloader | `0x08000000` | 128 KB | sectors 0-4 |
| Metadata | `0x08020000` | 128 KB | sector 5 |
| Application | `0x08040000` | 256 KB | sectors 6-7 (this firmware) |
| Staging | `0x08080000` | 256 KB | sectors 8-9 |
| Settings | `0x080C0000` | 128 KB | sector 10 |
| Calibration | `0x080E0000` | 128 KB | sector 11 (write-once) |

`SETTINGS_MAGIC = 0x08AC4A57`, `SETTINGS_VERSION = 2`. Calibration: 8 slots,
magic `0xCA11B08A`.

## Build

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Identity — `Application/fw_header/fw_header.h`: `FW_PRODUCT_ID=0x504C0804`,
`FW_HW_REVISION=0x0101`, `FW_VERSION_VALUE=0x0105`.

## Flashing

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/PLCJS_ETH_MODULE_8AIC_D4MG_STM32F407VGT6.elf -v -rst
```

## Network defaults

Net mode `2` = link-local (factory): `169.254.<mac[4]>.<mac[5]>` /16 from the
UID, discoverable by MAC (UDP broadcast port `20556`). Static fallback: IP
`192.168.1.10`, mask `255.255.255.0`, gateway `192.168.1.1`. Modbus TCP port
`502`, unit id `1`.

## Tools

`tools/calibrate.mjs` (Node.js 18+) / `tools/calibrate.py` (Python 3.8+):
status, configuration, calibration over Modbus TCP. See
[tools/README.md](tools/README.md).
