/**
  ******************************************************************************
  * @file    modbus_app.c
  * @brief   Modbus register-map adapter for the 8AIC module (see modbus_app.h).
  ******************************************************************************
  */

#include "modbus_app.h"

#include <string.h>

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "calstore.h"
#include "fw_header.h"
#include "led_module.h"
#include "ads1220.h"
#include "aic_module.h"
#include "settings.h"
#include "temp_module.h"

/* ---------------------------------------------------------------------------
 * Firmware version — derived from FW_VERSION_VALUE (set via CMake FW_VERSION,
 * encoding (major << 8) | minor) so there is a single source of truth shared
 * with the firmware header. Bumping FW_VERSION in CMakeLists updates both the
 * fw_header and the Modbus-reported version (IR120/IR121).
 * ------------------------------------------------------------------------- */
#define FW_VER_MAJOR    ((FW_VERSION_VALUE >> 8u) & 0xFFu)
#define FW_VER_MINOR    (FW_VERSION_VALUE & 0xFFu)

/* ---------------------------------------------------------------------------
 * Pending action flags driven by special holding-register triggers.
 * ------------------------------------------------------------------------- */
static volatile uint8_t s_pending_save           = 0u;
static volatile uint8_t s_pending_reboot         = 0u;
static volatile uint8_t s_pending_factory_reset  = 0u;
static volatile uint8_t s_pending_bootloader     = 0u;
static volatile uint8_t s_pending_switch_reset   = 0u;
static volatile uint32_t s_last_request_tick     = 0u;

/* Emergency calibration-erase arming (two-factor with the physical button). */
static volatile uint8_t  s_cal_erase_armed    = 0u;
static volatile uint32_t s_cal_erase_arm_tick = 0u;

void modbus_app_init(void)
{
    s_pending_save          = 0u;
    s_pending_reboot        = 0u;
    s_pending_factory_reset = 0u;
    s_pending_bootloader    = 0u;
    s_pending_switch_reset  = 0u;
    s_last_request_tick     = 0u;
    s_cal_erase_armed       = 0u;
    s_cal_erase_arm_tick    = 0u;
}

uint8_t modbus_app_cal_erase_armed(void)
{
    if (s_cal_erase_armed == 0u) { return 0u; }
    if ((HAL_GetTick() - s_cal_erase_arm_tick) > CAL_ERASE_ARM_WINDOW_MS) {
        s_cal_erase_armed = 0u;      /* window expired */
        return 0u;
    }
    return 1u;
}

void modbus_app_clear_cal_erase_arm(void)
{
    s_cal_erase_armed = 0u;
}

void modbus_app_notify_request(void)      { s_last_request_tick = HAL_GetTick(); }
uint32_t modbus_app_last_request_tick(void) { return s_last_request_tick; }

uint8_t modbus_app_take_pending_save(void)
{
    uint8_t v = s_pending_save; s_pending_save = 0u; return v;
}
uint8_t modbus_app_take_pending_reboot(void)
{
    uint8_t v = s_pending_reboot; s_pending_reboot = 0u; return v;
}
uint8_t modbus_app_take_pending_factory_reset(void)
{
    uint8_t v = s_pending_factory_reset; s_pending_factory_reset = 0u; return v;
}
uint8_t modbus_app_take_pending_bootloader(void)
{
    uint8_t v = s_pending_bootloader; s_pending_bootloader = 0u; return v;
}
uint8_t modbus_app_take_pending_switch_reset(void)
{
    uint8_t v = s_pending_switch_reset; s_pending_switch_reset = 0u; return v;
}

/* ---------------------------------------------------------------------------
 * float32 <-> register-pair helpers (HIGH word first).
 * ------------------------------------------------------------------------- */
static uint16_t float_word(float f, uint8_t word)
{
    uint32_t b;
    memcpy(&b, &f, sizeof(b));
    return (word == 0u) ? (uint16_t)(b >> 16) : (uint16_t)(b & 0xFFFFu);
}

static float float_set_word(float f, uint8_t word, uint16_t v)
{
    uint32_t b;
    memcpy(&b, &f, sizeof(b));
    if (word == 0u) {
        b = (b & 0x0000FFFFu) | ((uint32_t)v << 16);
    } else {
        b = (b & 0xFFFF0000u) | (uint32_t)v;
    }
    float o;
    memcpy(&o, &b, sizeof(o));
    return o;
}

/* ---------------------------------------------------------------------------
 * Address-range helpers.
 * ------------------------------------------------------------------------- */
/* Compact block 0..47: address = group*8 + ch. */
static bool in_ch_block(uint16_t a, uint8_t* group, uint8_t* ch)
{
    if (a >= MB_HR_CH_BASE + MB_HR_CH_GROUPS * MB_AI_CHANNELS) { return false; }
    const uint16_t rel = (uint16_t)(a - MB_HR_CH_BASE);
    *group = (uint8_t)(rel / MB_AI_CHANNELS);
    *ch    = (uint8_t)(rel % MB_AI_CHANNELS);
    return true;
}

/* Calibration block 540 + ch*4 + {gain w0, gain w1, off w0, off w1}. */
static bool in_ai_cal(uint16_t a, uint8_t* ch, bool* is_offset, uint8_t* word)
{
    if (a < MB_HR_AI_CAL_BASE) { return false; }
    const uint16_t rel = (uint16_t)(a - MB_HR_AI_CAL_BASE);
    if (rel >= MB_AI_CHANNELS * MB_HR_AI_CAL_STRIDE) { return false; }
    *ch        = (uint8_t)(rel / MB_HR_AI_CAL_STRIDE);
    const uint8_t idx = (uint8_t)(rel % MB_HR_AI_CAL_STRIDE);
    *is_offset = (idx >= 2u);
    *word      = (uint8_t)(idx & 1u);
    return true;
}

/* Nominal shunt (620..621) / V_REF (622..623): sel 0 = shunt, 1 = vref. */
static bool in_nominal(uint16_t a, uint8_t* sel, uint8_t* word)
{
    if (a < MB_HR_SHUNT_BASE || a > (uint16_t)(MB_HR_VREF_BASE + 1u)) { return false; }
    const uint16_t rel = (uint16_t)(a - MB_HR_SHUNT_BASE);
    *sel  = (uint8_t)(rel / 2u);
    *word = (uint8_t)(rel & 1u);
    return true;
}

static float* nominal_ptr(settings_t* s, uint8_t sel)
{
    return (sel == 0u) ? &s->shunt_nominal : &s->vref_nominal;
}

/* ---------------------------------------------------------------------------
 * Reads
 * ------------------------------------------------------------------------- */
static uint16_t read_input(uint16_t address)
{
    if (address >= MB_IR_AI_BASE && address < MB_IR_AI_END) {
        const aic_channel_status_t* st;

        if (address < MB_IR_AI_FLAGS) {
            /* Two float groups, 2 registers per channel each. */
            const uint16_t rel  = (uint16_t)(address - MB_IR_AI_CURRENT);
            const uint16_t grp  = (uint16_t)(rel / (2u * MB_AI_CHANNELS));
            const uint8_t  ch   = (uint8_t)((rel % (2u * MB_AI_CHANNELS)) / 2u);
            const uint8_t  word = (uint8_t)(rel & 1u);
            st = aic_module_get_status(ch);
            if (st == NULL) { return 0u; }
            return float_word((grp == 0u) ? st->i_cal_ma : st->i_raw_ma, word);
        }
        if (address < MB_IR_AI_CODE) {
            st = aic_module_get_status((uint8_t)(address - MB_IR_AI_FLAGS));
            if (st == NULL) { return 0u; }
            uint16_t f = 0u;
            if (st->enabled) { f |= MB_AI_FLAG_ENABLED; }
            if (st->valid)   { f |= MB_AI_FLAG_VALID; }
            if (st->fault)   { f |= MB_AI_FLAG_FAULT; }
            f |= (uint16_t)((uint16_t)st->fault_code << 8);
            return f;
        }
        const uint16_t rel = (uint16_t)(address - MB_IR_AI_CODE);
        st = aic_module_get_status((uint8_t)(rel / 2u));
        if (st == NULL) { return 0u; }
        const uint32_t u = (uint32_t)st->adc_code;
        return ((rel & 1u) == 0u) ? (uint16_t)(u >> 16) : (uint16_t)(u & 0xFFFFu);
    }

    switch (address) {
    case MB_IR_FW_VER_MAJOR: return FW_VER_MAJOR;
    case MB_IR_FW_VER_MINOR: return FW_VER_MINOR;
    case MB_IR_UPTIME_LO:    return (uint16_t)((HAL_GetTick() / 1000u) & 0xFFFFu);
    case MB_IR_UPTIME_HI:    return (uint16_t)(((HAL_GetTick() / 1000u) >> 16u) & 0xFFFFu);
    case MB_IR_MODULE_ID:    return MODULE_ID_08AIC;
    case MB_IR_TEMPERATURE:  return (uint16_t)temp_module_read_decicelsius();
    case MB_IR_CAL_LOCK:     return (uint16_t)(calstore_lock_mask() & 0xFFFFu);
    case MB_IR_SAVE_COUNT:      return settings_save_count();
    case MB_IR_SAVE_RESULT:     return settings_save_result();
    default:                 return 0u;
    }
}

static bool input_address_valid(uint16_t address)
{
    if (address >= MB_IR_AI_BASE && address < MB_IR_AI_END) { return true; }
    switch (address) {
    case MB_IR_FW_VER_MAJOR:
    case MB_IR_FW_VER_MINOR:
    case MB_IR_UPTIME_LO:
    case MB_IR_UPTIME_HI:
    case MB_IR_MODULE_ID:
    case MB_IR_TEMPERATURE:
    case MB_IR_CAL_LOCK:
    case MB_IR_SAVE_COUNT:
    case MB_IR_SAVE_RESULT:
        return true;
    default:
        return false;
    }
}

static uint16_t read_holding(uint16_t address)
{
    settings_t* s = settings_get();

    uint8_t group, ch, sel, word;
    bool is_offset;

    if (in_ch_block(address, &group, &ch)) {
        switch (group) {
        case MB_HR_CH_GROUP_READING:  return (uint16_t)aic_module_int16_reading(ch);
        case MB_HR_CH_GROUP_SCALE_LO: return s->ch_scale_lo_ua[ch];
        case MB_HR_CH_GROUP_SCALE_HI: return s->ch_scale_hi_ua[ch];
        case MB_HR_CH_GROUP_ENABLED:  return s->ch_enabled[ch];
        case MB_HR_CH_GROUP_SMOOTH:   return s->ch_smooth[ch];
        default:                      return 0u;
        }
    }
    if (in_ai_cal(address, &ch, &is_offset, &word)) {
        const float v = is_offset ? calstore_offset(ch, 0u) : calstore_gain(ch, 0u);
        return float_word(v, word);
    }
    if (in_nominal(address, &sel, &word)) {
        return float_word(*nominal_ptr(s, sel), word);
    }

    switch (address) {
    case MB_HR_SCAN_MS:           return s->scan_ms;
    case MB_HR_LED_MODE:          return s->led_mode;
    case MB_HR_SLAVE_ID:          return s->modbus_slave_id;
    case MB_HR_TCP_PORT:          return s->modbus_tcp_port;
    case MB_HR_ADC_RATE:          return s->adc_rate;

    case MB_HR_IP_BASE + 0u:      return s->ip[0];
    case MB_HR_IP_BASE + 1u:      return s->ip[1];
    case MB_HR_IP_BASE + 2u:      return s->ip[2];
    case MB_HR_IP_BASE + 3u:      return s->ip[3];

    case MB_HR_NETMASK_BASE + 0u: return s->netmask[0];
    case MB_HR_NETMASK_BASE + 1u: return s->netmask[1];
    case MB_HR_NETMASK_BASE + 2u: return s->netmask[2];
    case MB_HR_NETMASK_BASE + 3u: return s->netmask[3];

    case MB_HR_GATEWAY_BASE + 0u: return s->gateway[0];
    case MB_HR_GATEWAY_BASE + 1u: return s->gateway[1];
    case MB_HR_GATEWAY_BASE + 2u: return s->gateway[2];
    case MB_HR_GATEWAY_BASE + 3u: return s->gateway[3];

    case MB_HR_USE_DHCP:          return s->use_dhcp;

    case MB_HR_TRIG_SAVE:
    case MB_HR_TRIG_REBOOT:
    case MB_HR_TRIG_FACTORY_RESET:
    case MB_HR_CAL_COMMIT:
    case MB_HR_CAL_ERASE_ARM:      return 0u;

    case MB_HR_TEMPERATURE:       return (uint16_t)temp_module_read_decicelsius();

    default:                      return 0u;
    }
}

/* ---------------------------------------------------------------------------
 * Writes
 * ------------------------------------------------------------------------- */
static nmbs_error apply_holding_write(uint16_t address, uint16_t value)
{
    settings_t* s = settings_get();

    uint8_t group, ch, sel, word;
    bool is_offset;

    if (in_ch_block(address, &group, &ch)) {
        switch (group) {
        case MB_HR_CH_GROUP_READING:
        case MB_HR_CH_GROUP_RESERVED:
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;   /* read-only */
        case MB_HR_CH_GROUP_SCALE_LO:
            /* Must stay below the current high threshold. Write the high one
             * first when moving the whole span upwards. */
            if (value > SETTINGS_SCALE_MAX_UA || value >= s->ch_scale_hi_ua[ch]) {
                return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
            }
            s->ch_scale_lo_ua[ch] = value;
            break;
        case MB_HR_CH_GROUP_SCALE_HI:
            if (value > SETTINGS_SCALE_MAX_UA || value <= s->ch_scale_lo_ua[ch]) {
                return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
            }
            s->ch_scale_hi_ua[ch] = value;
            break;
        case MB_HR_CH_GROUP_ENABLED:
            if (value > 1u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
            s->ch_enabled[ch] = (uint8_t)value;
            break;
        case MB_HR_CH_GROUP_SMOOTH:
            if (value > SETTINGS_SMOOTH_MAX) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
            s->ch_smooth[ch] = (uint8_t)value;
            break;
        default:
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        aic_module_apply_config();
        return NMBS_ERROR_NONE;
    }

    if (in_ai_cal(address, &ch, &is_offset, &word)) {
        /* Write-once: reject any change to an already committed slot. */
        if (calstore_is_locked(ch, 0u)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        const float cur     = is_offset ? calstore_offset(ch, 0u) : calstore_gain(ch, 0u);
        const float patched = float_set_word(cur, word, value);
        const bool  ok      = is_offset ? calstore_set_offset(ch, 0u, patched)
                                        : calstore_set_gain(ch, 0u, patched);
        return ok ? NMBS_ERROR_NONE : NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

    if (in_nominal(address, &sel, &word)) {
        float* p = nominal_ptr(s, sel);
        *p = float_set_word(*p, word, value);
        return NMBS_ERROR_NONE;
    }

    switch (address) {
    case MB_HR_SCAN_MS:
        if (value < SETTINGS_SCAN_MS_MIN || value > SETTINGS_SCAN_MS_MAX) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        s->scan_ms = value;
        aic_module_apply_config();
        break;

    case MB_HR_ADC_RATE:
        if (value >= ADS1220_RATE_COUNT) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->adc_rate = (uint8_t)value;
        aic_module_apply_config();
        break;

    case MB_HR_LED_MODE:
        if (value > (uint16_t)LED_MODE_STATE_MACHINE) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->led_mode = value;
        led_module_set_mode((uint8_t)value);
        break;

    case MB_HR_SLAVE_ID:
        if (value < 1u || value > 247u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->modbus_slave_id = (uint8_t)value;
        break;

    case MB_HR_TCP_PORT:
        if (value == 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->modbus_tcp_port = value;
        break;

    case MB_HR_IP_BASE + 0u: case MB_HR_IP_BASE + 1u:
    case MB_HR_IP_BASE + 2u: case MB_HR_IP_BASE + 3u:
        if (value > 255u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->ip[address - MB_HR_IP_BASE] = (uint8_t)value;
        break;

    case MB_HR_NETMASK_BASE + 0u: case MB_HR_NETMASK_BASE + 1u:
    case MB_HR_NETMASK_BASE + 2u: case MB_HR_NETMASK_BASE + 3u:
        if (value > 255u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->netmask[address - MB_HR_NETMASK_BASE] = (uint8_t)value;
        break;

    case MB_HR_GATEWAY_BASE + 0u: case MB_HR_GATEWAY_BASE + 1u:
    case MB_HR_GATEWAY_BASE + 2u: case MB_HR_GATEWAY_BASE + 3u:
        if (value > 255u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->gateway[address - MB_HR_GATEWAY_BASE] = (uint8_t)value;
        break;

    case MB_HR_USE_DHCP:
        /* Network mode: 0=static, 1=DHCP, 2=link-local (see NET_MODE_* in settings.h). */
        if (value > NET_MODE_LINKLOCAL) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->use_dhcp = (uint8_t)value;
        break;

    case MB_HR_TRIG_SAVE:
        if (value == MODBUS_TRIG_SAVE) { s_pending_save = 1u; }
        else if (value != 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        break;

    case MB_HR_TRIG_REBOOT:
        if (value == MODBUS_TRIG_REBOOT) { s_pending_reboot = 1u; }
        else if (value == MODBUS_TRIG_BOOTLOADER) { s_pending_bootloader = 1u; }
        else if (value == MODBUS_TRIG_SWITCH_RESET) { s_pending_switch_reset = 1u; }
        else if (value != 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        break;

    case MB_HR_TRIG_FACTORY_RESET:
        if (value == MODBUS_TRIG_FACTORY_RESET) { s_pending_factory_reset = 1u; }
        else if (value != 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        break;

    case MB_HR_CAL_COMMIT: {
        if ((value & ~MB_CAL_COMMIT_SLOT_MASK) != MB_CAL_COMMIT_BASE) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        const uint8_t cch = (uint8_t)(value & MB_CAL_COMMIT_SLOT_MASK);   /* = channel */
        if (cch >= MB_AI_CHANNELS) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        /* One-shot: fails if already locked or on Flash error. */
        if (!calstore_commit(cch, 0u)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        break;
    }

    case MB_HR_CAL_ERASE_ARM:
        if (value == MODBUS_TRIG_CAL_ERASE_ARM) {
            s_cal_erase_arm_tick = HAL_GetTick();
            s_cal_erase_armed    = 1u;
        } else if (value == 0u) {
            s_cal_erase_armed    = 0u;   /* explicit disarm */
        } else {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        break;

    default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }

    return NMBS_ERROR_NONE;
}

static bool holding_address_valid(uint16_t address)
{
    uint8_t a, b, d;
    bool c;
    if (in_ch_block(address, &a, &b))       { return true; }
    if (in_ai_cal(address, &a, &c, &d))     { return true; }
    if (in_nominal(address, &a, &d))        { return true; }

    if (address >= MB_HR_SCAN_MS && address <= MB_HR_USE_DHCP) { return true; }
    if (address == MB_HR_TRIG_SAVE || address == MB_HR_TRIG_REBOOT ||
        address == MB_HR_TRIG_FACTORY_RESET) { return true; }
    if (address == MB_HR_CAL_COMMIT || address == MB_HR_CAL_ERASE_ARM ||
        address == MB_HR_ADC_RATE) { return true; }
    if (address == MB_HR_TEMPERATURE) { return true; }
    return false;
}

/* ---------------------------------------------------------------------------
 * nanoMODBUS callbacks
 * ------------------------------------------------------------------------- */
static nmbs_error cb_read_input_registers(uint16_t address, uint16_t quantity,
                                          uint16_t* registers_out)
{
    for (uint16_t i = 0; i < quantity; i++) {
        const uint16_t a = (uint16_t)(address + i);
        if (!input_address_valid(a)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        registers_out[i] = read_input(a);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_read_holding_registers(uint16_t address, uint16_t quantity,
                                            uint16_t* registers_out)
{
    for (uint16_t i = 0; i < quantity; i++) {
        const uint16_t a = (uint16_t)(address + i);
        if (!holding_address_valid(a)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        registers_out[i] = read_holding(a);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_single_register(uint16_t address, uint16_t value)
{
    if (!holding_address_valid(address)) {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    return apply_holding_write(address, value);
}

static nmbs_error cb_write_multiple_registers(uint16_t address, uint16_t quantity,
                                              const uint16_t* registers)
{
    for (uint16_t i = 0; i < quantity; i++) {
        if (!holding_address_valid((uint16_t)(address + i))) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    for (uint16_t i = 0; i < quantity; i++) {
        const nmbs_error e = apply_holding_write((uint16_t)(address + i), registers[i]);
        if (e != NMBS_ERROR_NONE) {
            return e;
        }
    }
    return NMBS_ERROR_NONE;
}

static const nmbs_callbacks s_callbacks = {
    .read_coils                = NULL,
    .read_discrete_inputs      = NULL,
    .read_holding_registers    = cb_read_holding_registers,
    .read_input_registers      = cb_read_input_registers,
    .write_single_coil         = NULL,
    .write_single_register     = cb_write_single_register,
    .write_multiple_coils      = NULL,
    .write_multiple_registers  = cb_write_multiple_registers,
};

const nmbs_callbacks* modbus_app_get_callbacks(void)
{
    return &s_callbacks;
}
