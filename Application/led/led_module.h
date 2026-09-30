/**
  ******************************************************************************
  * @file    led_module.h
  * @brief   STAT_LED state machine driver.
  *
  *  Modes:
  *    - LED_MODE_ALW_OFF:        LED is permanently off (overridden by
  *                               LED_STATE_FACTORY_RESET while it is active).
  *    - LED_MODE_ALW_ON:         LED is permanently on (overridden by
  *                               LED_STATE_FACTORY_RESET while it is active).
  *    - LED_MODE_STATE_MACHINE:  LED follows the device state.
  *
  *  States in LED_MODE_STATE_MACHINE:
  *    - LED_STATE_NO_POLLING:     1 short blink every 3 s.
  *    - LED_STATE_POLLING:        2 short blinks every 1.5 s.
  *    - LED_STATE_NO_LINK:        3 short blinks every 3 s.
  *    - LED_STATE_FACTORY_RESET:  continuous periodic ON/OFF blink with
  *                                independently configurable T_ON and T_OFF
  *                                (see led_module_set_factory_reset_timing()).
  *                                Sticky: once entered, blinks until reboot
  *                                and overrides any subsequent state / mode
  *                                change.
  ******************************************************************************
  */
#ifndef APPLICATION_LED_MODULE_H
#define APPLICATION_LED_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default and clamp values for the LED_STATE_FACTORY_RESET blink pattern. */
/* Period of the POLLING double-blink. Also the window after the last Modbus
 * request during which the module still reports "being polled": one request
 * is one poll and earns exactly one burst. */
#define LED_POLLING_PERIOD_MS       1500u

#define LED_FRESET_DEFAULT_ON_MS    100u
#define LED_FRESET_DEFAULT_OFF_MS   100u
#define LED_FRESET_MIN_MS           10u

typedef enum {
    LED_STATE_NO_POLLING     = 0,
    LED_STATE_POLLING        = 1,
    LED_STATE_FACTORY_RESET  = 2,
    LED_STATE_NO_LINK        = 3,
    LED_STATE_CAL_ARMED      = 4,   /* emergency cal-erase armed: fast blink train */
} led_state_t;

/** Initialise the LED module with a given mode (see settings.h). */
void led_module_init(uint8_t mode);

/** Change the operating mode at runtime (see led_mode_t in settings.h). */
void led_module_set_mode(uint8_t mode);

/** Get the currently active operating mode. */
uint8_t led_module_get_mode(void);

/** Set the device state used by the state-machine mode. */
void led_module_set_state(led_state_t state);

/**
 * Enter LED_STATE_FACTORY_RESET and start the configurable periodic blink.
 * The state is sticky: subsequent set_state() / set_mode() calls are ignored
 * until the next boot. Override the timings with
 * led_module_set_factory_reset_timing() before calling this if needed.
 */
void led_module_signal_factory_reset(void);

/**
 * Enter a sticky fast blink used to confirm an emergency calibration erase
 * just before the reboot. Like the factory-reset indicator, it overrides the
 * current mode/state until the next boot.
 */
void led_module_signal_cal_erase(void);

/**
 * Configure the LED_STATE_FACTORY_RESET ON / OFF times, milliseconds.
 * Each value is clamped to at least LED_FRESET_MIN_MS to keep the FSM
 * from spinning. Takes effect on the next half-cycle.
 */
void led_module_set_factory_reset_timing(uint16_t on_ms, uint16_t off_ms);

/** Read the active LED_STATE_FACTORY_RESET ON / OFF times. NULL-safe. */
void led_module_get_factory_reset_timing(uint16_t *on_ms, uint16_t *off_ms);

/**
 * Blink the STAT_LED rapidly for @p ms milliseconds to physically locate the
 * device (discovery "flash LED", like TIA Portal). Overrides the normal state
 * pattern for the duration, then resumes. Ignored while FACTORY_RESET is active
 * (that indicator has priority).
 */
void led_module_signal_identify(uint32_t ms);

/**
 * Periodic tick — call from a single task with a fixed period. The default
 * tick used in the application is 10 ms; the module timing is expressed in
 * ticks so other periods are usable as long as `tick_ms` is passed.
 */
void led_module_tick(uint16_t tick_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LED_MODULE_H */
