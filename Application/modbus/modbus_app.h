/**
  ******************************************************************************
  * @file    modbus_app.h
  * @brief   Modbus register-map adapter for the 8AIC analog current-input module.
  *
  *  Float values are IEEE-754 32-bit, transmitted as two 16-bit registers with
  *  the HIGH word first (big-endian word order: register[N] = bits 31..16).
  *  Multi-channel quantities are grouped by quantity: 8 consecutive registers
  *  (or 8 register pairs) are channels 0..7.
  *
  *  ---- Holding Registers (FC03/06/16) — compact per-channel block --------
  *      0..7    reading, int16 (RO): 0..32767 = scale_lo..scale_hi of the
  *                channel (auto-scaled to the thresholds below), negative
  *                below scale_lo, clamped to ±32767;
  *                disabled = 0,  fault / not yet valid = −32768 (0x8000)
  *      8..15   scale low threshold, µA  (default 4000;  0..25000, < high)
  *      16..23  scale high threshold, µA (default 20000; ..25000, > low)
  *      24..31  enabled (0/1, default 1)
  *      32..39  smoothing (EMA): 0 off, 1 weak (1/4), 2 medium (1/8), 3 (1/16)
  *      40..47  reserved (read 0, writes rejected)
  *    Open-loop fault (< 3.6 mA) is only raised on live-zero scales, i.e. when
  *    the low threshold is >= 3600 µA; over-range (> 21 mA) always applies.
  *
  *  ---- Input Registers (FC04, read-only) — grouped by quantity ----------
  *      300..315 current, float32 mA ×8           (NaN on fault / disabled)
  *      316..331 raw (uncalibrated) current, float32 mA ×8   (calibration input)
  *      332..339 status flags ×8: bit0 enabled, bit1 valid, bit2 fault,
  *                 bits15..8 fault code (1 open loop, 2 over-range, 3 ADC dead)
  *      340..355 ADC code, int32 (24-bit signed, high word first) ×8
  *    Global:
  *      120 fw major, 121 fw minor, 122/123 uptime s (lo/hi),
  *      125 module id (0x08AC), 126 on-chip temperature (signed 0.1 °C)
  *      129 settings saves since boot (Flash written + read-back), 130 last save
  *      result (0 none, 1 ok, 2 failed)
  *      127 calibration lock bitmask (bit = channel)
  *
  *  ---- Holding Registers (FC03/06/16) — global -----------------------------
  *      100 scan period ms (50..5000)          101 LED mode (0/1/2)
  *      102 Modbus slave id                    103 Modbus TCP port
  *      104..107 static IP octets              108..111 netmask octets
  *      112..115 gateway octets                116 net mode (0 static/1 DHCP/2 LL)
  *      117 SAVE trigger (0xA5A5)              118 REBOOT (0xB00B) / BOOT (0xB007)
  *                                                 / KSZ8863 reset (0x8863)
  *      119 FACTORY RESET trigger (0xDEAD)     130 on-chip temperature (RO)
  *      131 CAL COMMIT trigger (0xCA00|ch)     132 CAL ERASE ARM (0xC1A5)
  *      133 ADC data rate: 0 = 20 SPS + 50/60 Hz FIR, 1 = 90 SPS, 2 = 330 SPS
  *    Calibration coefficients (float32), base 540 + ch*4 (WRITE-ONCE):
  *      +0..1 gain,  +2..3 offset (mA)
  *      Writes are a live preview and are rejected once the channel slot is
  *      committed. Commit = write 0xCA00|ch to register 131.
  *    Nominal shunt (float32 Ω): 620..621     Nominal V_REF (float32 V): 622..623
  ******************************************************************************
  */
#ifndef APPLICATION_MODBUS_APP_H
#define APPLICATION_MODBUS_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "nanomodbus.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Magic write triggers (FC06/FC10 to specific holding registers). */
#define MODBUS_TRIG_SAVE            0xA5A5u
#define MODBUS_TRIG_REBOOT          0xB00Bu
#define MODBUS_TRIG_FACTORY_RESET   0xDEADu
#define MODBUS_TRIG_BOOTLOADER      0xB007u
/* Hardware-reset the KSZ8863 Ethernet switch (operator recovery command).
 * Written to MB_HR_TRIG_REBOOT (118). Interrupts pass-through traffic for
 * the duration of the chip reset + port auto-negotiation — use only when the
 * switch shows no signs of life. */
#define MODBUS_TRIG_SWITCH_RESET    0x8863u

/* Calibration commit: value = MB_CAL_COMMIT_BASE | ch, slot 0..7. */
#define MB_CAL_COMMIT_BASE          0xCA00u
#define MB_CAL_COMMIT_SLOT_MASK     0x00FFu

/* Emergency calibration-erase arming magic (two-factor: this + button). */
#define MODBUS_TRIG_CAL_ERASE_ARM   0xC1A5u

/* Window after arming during which a physical button confirm erases the
 * calibration sector, and the button hold required to confirm (ms). */
#define CAL_ERASE_ARM_WINDOW_MS     30000u
#define CAL_ERASE_CONFIRM_MS        3000u

/* No-init RAM cell shared with the bootloader. */
#define BOOT_REQUEST_FLAG_ADDR      0x2001FFF0u
#define BOOT_REQUEST_MAGIC          0xB007CAFEu

/* ---- Global holding registers ---- */
#define MB_HR_SCAN_MS               100u
#define MB_HR_LED_MODE              101u
#define MB_HR_SLAVE_ID              102u
#define MB_HR_TCP_PORT              103u
#define MB_HR_IP_BASE               104u
#define MB_HR_NETMASK_BASE          108u
#define MB_HR_GATEWAY_BASE          112u
#define MB_HR_USE_DHCP              116u
#define MB_HR_TRIG_SAVE             117u
#define MB_HR_TRIG_REBOOT           118u
#define MB_HR_TRIG_FACTORY_RESET    119u
#define MB_HR_TEMPERATURE           130u
#define MB_HR_CAL_COMMIT            131u
#define MB_HR_CAL_ERASE_ARM         132u
#define MB_HR_ADC_RATE              133u

/* ---- Global input registers ---- */
#define MB_IR_FW_VER_MAJOR          120u
#define MB_IR_FW_VER_MINOR          121u
#define MB_IR_UPTIME_LO             122u
#define MB_IR_UPTIME_HI             123u
#define MB_IR_SAVE_COUNT            129u  /* successful settings saves since boot */
#define MB_IR_SAVE_RESULT           130u  /* last save: 0 none, 1 ok, 2 failed    */
#define MB_IR_MODULE_ID             125u
#define MB_IR_TEMPERATURE           126u
#define MB_IR_CAL_LOCK              127u

#define MB_AI_CHANNELS              8u

/* ---- Compact per-channel block (holding): address = group*8 + ch ---- */
#define MB_HR_CH_BASE               0u
#define MB_HR_CH_GROUP_READING      0u    /* int16, read-only */
#define MB_HR_CH_GROUP_SCALE_LO     1u    /* µA */
#define MB_HR_CH_GROUP_SCALE_HI     2u    /* µA */
#define MB_HR_CH_GROUP_ENABLED      3u
#define MB_HR_CH_GROUP_SMOOTH       4u
#define MB_HR_CH_GROUP_RESERVED     5u
#define MB_HR_CH_GROUPS             6u    /* registers 0..47 */

/* ---- Readings (input registers), grouped by quantity ---- */
#define MB_IR_AI_BASE               300u
#define MB_IR_AI_CURRENT            300u  /* float32 ×8 -> 300..315 */
#define MB_IR_AI_RAW                316u  /* float32 ×8 -> 316..331 */
#define MB_IR_AI_FLAGS              332u  /* u16 ×8     -> 332..339 */
#define MB_IR_AI_CODE               340u  /* int32 ×8   -> 340..355 */
#define MB_IR_AI_END                356u  /* first address past the block */

/* Reading flag bits (registers 332..339). */
#define MB_AI_FLAG_ENABLED          0x0001u
#define MB_AI_FLAG_VALID            0x0002u
#define MB_AI_FLAG_FAULT            0x0004u

/* ---- Calibration coefficients (holding, float32) ---- */
#define MB_HR_AI_CAL_BASE           540u
#define MB_HR_AI_CAL_STRIDE         4u    /* gain, offset floats per channel */

/* ---- Nominal shunt / reference (holding, float32) ---- */
#define MB_HR_SHUNT_BASE            620u  /* 620..621 */
#define MB_HR_VREF_BASE             622u  /* 622..623 */

/* Module ID (input register 125). */
#define MODULE_ID_08AIC             0x08ACu

/** Initialise the modbus register adapter. Must be called after settings_init(). */
void modbus_app_init(void);

/** Mark a successful modbus transaction (used to drive STAT_LED state). */
void modbus_app_notify_request(void);

/** Get the populated nmbs_callbacks structure for nmbs_server_create(). */
const nmbs_callbacks* modbus_app_get_callbacks(void);

/**
 * Returns 1 if an emergency calibration erase has been armed over Modbus and
 * the arming window (CAL_ERASE_ARM_WINDOW_MS) has not yet expired. Used by the
 * application loop to gate the physical button confirmation. Non-destructive.
 */
uint8_t modbus_app_cal_erase_armed(void);

/** Clear the armed calibration-erase state (e.g. after the action completes). */
void modbus_app_clear_cal_erase_arm(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MODBUS_APP_H */
