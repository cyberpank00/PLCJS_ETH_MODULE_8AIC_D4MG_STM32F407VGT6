/**
  ******************************************************************************
  * @file    led_module.c
  * @brief   Time-triggered LED state machine. The driver consumes a fixed
  *          tick period (typically 10 ms) and emits a blink pattern that
  *          depends on the selected operating mode and reported device state.
  *
  *          LED_STATE_FACTORY_RESET uses its own independent periodic blink
  *          path with configurable T_ON and T_OFF (see
  *          led_module_set_factory_reset_timing). All other states share the
  *          existing burst-with-gap pattern engine driven by led_pattern_t.
  ******************************************************************************
  */

#include "led_module.h"

#include <stddef.h>

#include "main.h"
#include "settings.h"
#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Pattern timings, milliseconds (burst-style states: NO_POLLING, POLLING,
 * NO_LINK). LED_STATE_FACTORY_RESET timings are NOT defined here — see
 * s_freset_on_ms / s_freset_off_ms below and the public setter.
 * ------------------------------------------------------------------------- */
#define LED_PULSE_ON_MS         50u
#define LED_PULSE_GAP_MS        150u    /* off-time between pulses inside one burst */

#define LED_NOPOLL_PERIOD_MS    3000u
/* LED_POLLING_PERIOD_MS lives in led_module.h (shared with app.c). */

#define LED_NOPOLL_PULSES       1u
#define LED_POLLING_PULSES      2u
#define LED_NOLINK_PULSES       3u
#define LED_NOLINK_PERIOD_MS    3000u

/* CAL_ARMED: a dense 5-pulse train each period, visually distinct from the
 * 1/2/3-blink states — signals that an emergency cal-erase is armed. */
#define LED_CALARM_PULSES       5u
#define LED_CALARM_PERIOD_MS    1500u

/* ---------------------------------------------------------------------------
 * Internal state for the shared burst-pattern engine (NO_POLLING / POLLING /
 * NO_LINK). FACT_RESET has its own state below.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t pulses;            /* pulses in the current burst         */
    uint16_t period_ms;         /* total period including idle gap     */
    uint16_t pulse_idx;         /* currently emitted pulse, 0-based    */
    uint16_t timer_ms;          /* time elapsed in current sub-state   */
    uint8_t  on_phase;          /* 1 = ON, 0 = OFF                     */
    uint8_t  finished_burst;    /* when set, we are in the trailing gap*/
} led_pattern_t;

static uint8_t        s_mode  = LED_MODE_STATE_MACHINE;
static led_state_t    s_state = LED_STATE_NO_POLLING;
static led_pattern_t  s_pattern;

/* Dedicated state for LED_STATE_FACTORY_RESET — independent ON/OFF blink. */
static uint16_t s_freset_on_ms    = LED_FRESET_DEFAULT_ON_MS;
static uint16_t s_freset_off_ms   = LED_FRESET_DEFAULT_OFF_MS;
static uint16_t s_freset_timer_ms = 0u;
static uint8_t  s_freset_on_phase = 0u;

/* "Identify" (discovery flash-LED): rapid blink for a bounded duration. */
#define LED_IDENTIFY_HALF_MS   100u
static uint32_t s_identify_remaining_ms = 0u;
static uint16_t s_identify_timer_ms     = 0u;
static uint8_t  s_identify_on_phase     = 0u;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */
static inline void led_set(bool on)
{
    HAL_GPIO_WritePin(STAT_LED_GPIO_Port, STAT_LED_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void led_load_pattern(led_state_t st)
{
    s_pattern.pulse_idx      = 0u;
    s_pattern.timer_ms       = 0u;
    s_pattern.on_phase       = 0u;
    s_pattern.finished_burst = 0u;

    switch (st) {
    case LED_STATE_POLLING:
        s_pattern.pulses    = LED_POLLING_PULSES;
        s_pattern.period_ms = LED_POLLING_PERIOD_MS;
        break;
    case LED_STATE_NO_LINK:
        s_pattern.pulses    = LED_NOLINK_PULSES;
        s_pattern.period_ms = LED_NOLINK_PERIOD_MS;
        break;
    case LED_STATE_CAL_ARMED:
        s_pattern.pulses    = LED_CALARM_PULSES;
        s_pattern.period_ms = LED_CALARM_PERIOD_MS;
        break;
    case LED_STATE_FACTORY_RESET:
    case LED_STATE_NO_POLLING:
    default:
        s_pattern.pulses    = LED_NOPOLL_PULSES;
        s_pattern.period_ms = LED_NOPOLL_PERIOD_MS;
        break;
    }

    led_set(false);
}

/* ---------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */
void led_module_init(uint8_t mode)
{
    s_mode  = mode;
    s_state = LED_STATE_NO_POLLING;

    s_freset_timer_ms = 0u;
    s_freset_on_phase = 0u;

    led_load_pattern(s_state);

    if (mode == LED_MODE_ALW_ON) {
        led_set(true);
    } else {
        led_set(false);
    }
}

void led_module_set_mode(uint8_t mode)
{
    if (mode == s_mode) {
        return;
    }
    s_mode = mode;

    /* FACT_RESET is a critical indicator — it must remain visible until the
     * reboot regardless of the requested mode. */
    if (s_state == LED_STATE_FACTORY_RESET) {
        return;
    }

    if (mode == LED_MODE_ALW_ON) {
        led_set(true);
    } else if (mode == LED_MODE_ALW_OFF) {
        led_set(false);
    } else {
        led_load_pattern(s_state);
    }
}

uint8_t led_module_get_mode(void)
{
    return s_mode;
}

void led_module_set_state(led_state_t state)
{
    /* FACT_RESET is sticky — no transitions out until reboot. */
    if (s_state == LED_STATE_FACTORY_RESET) {
        return;
    }
    if (state == LED_STATE_FACTORY_RESET) {
        led_module_signal_factory_reset();
        return;
    }
    if (state == s_state) {
        return;
    }
    s_state = state;
    if (s_mode == LED_MODE_STATE_MACHINE) {
        led_load_pattern(s_state);
    }
}

void led_module_signal_factory_reset(void)
{
    if (s_state == LED_STATE_FACTORY_RESET) {
        return; /* already running */
    }
    s_state            = LED_STATE_FACTORY_RESET;
    s_freset_timer_ms  = 0u;
    s_freset_on_phase  = 1u;
    led_set(true);
}

void led_module_signal_cal_erase(void)
{
    /* Reuse the sticky factory-reset blink path with a faster cadence so the
     * pre-reboot confirmation is visually distinct. The device reboots right
     * after this, so overriding the factory-reset timing here is harmless. */
    led_module_set_factory_reset_timing(40u, 40u);
    led_module_signal_factory_reset();
}

void led_module_signal_identify(uint32_t ms)
{
    if (s_state == LED_STATE_FACTORY_RESET) {
        return; /* factory-reset indicator has priority */
    }
    s_identify_remaining_ms = ms;
    s_identify_timer_ms     = 0u;
    s_identify_on_phase     = 1u;
    led_set(true);
}

void led_module_set_factory_reset_timing(uint16_t on_ms, uint16_t off_ms)
{
    if (on_ms  < LED_FRESET_MIN_MS) on_ms  = LED_FRESET_MIN_MS;
    if (off_ms < LED_FRESET_MIN_MS) off_ms = LED_FRESET_MIN_MS;
    s_freset_on_ms  = on_ms;
    s_freset_off_ms = off_ms;
}

void led_module_get_factory_reset_timing(uint16_t *on_ms, uint16_t *off_ms)
{
    if (on_ms  != NULL) *on_ms  = s_freset_on_ms;
    if (off_ms != NULL) *off_ms = s_freset_off_ms;
}

/* ---------------------------------------------------------------------------
 * Tick
 * ------------------------------------------------------------------------- */
void led_module_tick(uint16_t tick_ms)
{
    /* FACT_RESET runs on its own independent periodic blink and overrides
     * the current mode (ALW_ON / ALW_OFF do not silence it). */
    if (s_state == LED_STATE_FACTORY_RESET) {
        s_freset_timer_ms = (uint16_t)(s_freset_timer_ms + tick_ms);
        const uint16_t threshold = s_freset_on_phase
                                       ? s_freset_on_ms
                                       : s_freset_off_ms;
        if (s_freset_timer_ms >= threshold) {
            s_freset_on_phase  = (uint8_t)(!s_freset_on_phase);
            s_freset_timer_ms  = 0u;
            led_set(s_freset_on_phase != 0u);
        }
        return;
    }

    /* Identify (discovery flash-LED) overrides the normal indication for its
     * bounded duration, then resumes whatever the mode/state dictates. */
    if (s_identify_remaining_ms > 0u) {
        if (s_identify_remaining_ms <= tick_ms) {
            s_identify_remaining_ms = 0u;
            if (s_mode == LED_MODE_ALW_ON)       { led_set(true); }
            else if (s_mode == LED_MODE_ALW_OFF) { led_set(false); }
            else                                 { led_load_pattern(s_state); }
            return;
        }
        s_identify_remaining_ms -= tick_ms;
        s_identify_timer_ms = (uint16_t)(s_identify_timer_ms + tick_ms);
        if (s_identify_timer_ms >= LED_IDENTIFY_HALF_MS) {
            s_identify_timer_ms = 0u;
            s_identify_on_phase = (uint8_t)(!s_identify_on_phase);
            led_set(s_identify_on_phase != 0u);
        }
        return;
    }

    if (s_mode == LED_MODE_ALW_ON) {
        led_set(true);
        return;
    }
    if (s_mode == LED_MODE_ALW_OFF) {
        led_set(false);
        return;
    }

    /* Burst-style pattern engine for NO_POLLING / POLLING / NO_LINK. */
    s_pattern.timer_ms = (uint16_t)(s_pattern.timer_ms + tick_ms);

    if (!s_pattern.finished_burst) {
        if (s_pattern.on_phase) {
            if (s_pattern.timer_ms >= LED_PULSE_ON_MS) {
                /* end of ON phase -> start gap */
                led_set(false);
                s_pattern.on_phase = 0u;
                s_pattern.timer_ms = 0u;
                s_pattern.pulse_idx++;
                if (s_pattern.pulse_idx >= s_pattern.pulses) {
                    s_pattern.finished_burst = 1u;
                }
            }
        } else {
            if (s_pattern.timer_ms >= LED_PULSE_GAP_MS) {
                /* start next ON pulse */
                led_set(true);
                s_pattern.on_phase = 1u;
                s_pattern.timer_ms = 0u;
            }
        }
        return;
    }

    /* In the trailing idle period, until period_ms has elapsed in total. */
    const uint32_t burst_duration =
        (uint32_t)s_pattern.pulses * (LED_PULSE_ON_MS + LED_PULSE_GAP_MS);
    const uint32_t elapsed_total =
        burst_duration + (uint32_t)s_pattern.timer_ms;

    if (elapsed_total >= s_pattern.period_ms) {
        /* Restart the same burst. */
        led_load_pattern(s_state);
    }
}
