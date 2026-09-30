/**
  ******************************************************************************
  * @file    app.c
  * @brief   Application orchestrator. Wires the modular drivers together,
  *          handles factory-reset, brings up the network interface in the
  *          configured mode and runs the housekeeping loop (LED ticks,
  *          watchdog, deferred Modbus actions).
  ******************************************************************************
  */

#include "app.h"
#include <string.h>

#include "cmsis_os.h"
#include "iwdg.h"
#include "lwip/api.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include "button_module.h"
#include "calstore.h"
#include "discovery.h"
#include "ksz8863.h"
#include "led_module.h"
#include "modbus_app.h"
#include "modbus_tcp_server.h"
#include "net_id.h"
#include "aic_module.h"
#include "settings.h"
#include "temp_module.h"

/* The LwIP MX_LWIP_Init() exposes its struct netif so that we can override
 * the addressing after MX_LWIP_Init() has run. */
extern struct netif gnetif;

/* Physical link state from ethernet_link_thread (ethernetif.c). */
extern volatile uint8_t g_eth_any_link_up;

/* Module accessors implemented in modbus_app.c. */
uint8_t modbus_app_take_pending_save(void);
uint8_t modbus_app_take_pending_reboot(void);
uint8_t modbus_app_take_pending_factory_reset(void);
uint8_t modbus_app_take_pending_bootloader(void);
uint8_t modbus_app_take_pending_switch_reset(void);
uint32_t modbus_app_last_request_tick(void);

/* ---------------------------------------------------------------------------
 * Sub-tasks
 * ------------------------------------------------------------------------- */

/* Acquisition task: scans all eight current inputs once per scan period
 * (configurable, default 250 ms). One scan is four single-shot ADS1220
 * conversions (~210 ms at 20 SPS), so with short periods the task simply
 * runs back-to-back. */
static void aic_task(void* arg)
{
    (void)arg;
    for (;;) {
        const uint32_t start = osKernelGetTickCount();
        aic_module_tick();
        const uint32_t spent  = osKernelGetTickCount() - start;
        const uint32_t period = aic_module_scan_period_ms();
        osDelay((spent < period) ? (period - spent) : 1u);
    }
}

/* LED task: 10 ms tick. Drives the main STAT_LED state machine and the four
 * per-channel status LEDs (solid = active, off = inactive, fast blink =
 * converter fault). */
static void led_task(void* arg)
{
    (void)arg;
    const uint16_t period_ms = 10u;
    uint32_t tick = osKernelGetTickCount();
    for (;;) {
        led_module_tick(period_ms);
        aic_module_led_tick(period_ms);
        tick += period_ms;
        osDelayUntil(tick);
    }
}

/* ---------------------------------------------------------------------------
 * Factory reset routine
 *
 * Order matters: commit defaults to Flash first (HAL_FLASH erase of sector 11
 * blocks the CPU and prevents the LED task from running for ~1-2 s), then
 * play the visual confirmation burst, then reboot. The LED task must already
 * be running when this routine is entered (see app_run()).
 * ------------------------------------------------------------------------- */
static void perform_factory_reset(void)
{
    /* Reset and persist defaults to Flash. */
    settings_reset_to_defaults();
    settings_save();

    /* Enter the sticky LED_STATE_FACTORY_RESET — the LED task drives the
     * configurable periodic ON/OFF blink on its 10 ms tick until the reboot
     * below. Default cadence is 100 ms ON / 100 ms OFF, configurable via
     * led_module_set_factory_reset_timing(). */
    led_module_signal_factory_reset();

    /* 3.5 s wait — long enough for the operator to see the indication at
     * the default 5 Hz cadence (and reasonable headroom at slower settings).
     * The LED keeps blinking the whole time. */
    const uint32_t deadline = HAL_GetTick() + 3500u;
    while (HAL_GetTick() < deadline) {
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(50);
    }
    NVIC_SystemReset();
}

/* ---------------------------------------------------------------------------
 * Emergency calibration erase (two-factor)
 *
 * Entered only after a Modbus arm (register 132 = 0xC1A5) followed by a
 * physical button confirmation. Erases the write-once calibration sector,
 * reverting every slot to identity (gain 1.0, offset 0.0) and re-enabling
 * writes, then reboots for a clean start. The sector erase blocks the CPU for
 * ~1-2 s, so the confirmation blink is started first.
 * ------------------------------------------------------------------------- */
static void perform_cal_erase(void)
{
    led_module_signal_cal_erase();
    HAL_IWDG_Refresh(&hiwdg);

    (void)calstore_erase();

    const uint32_t deadline = HAL_GetTick() + 1500u;
    while (HAL_GetTick() < deadline) {
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(50);
    }
    NVIC_SystemReset();
}

/* ---------------------------------------------------------------------------
 * Network bring-up
 * ------------------------------------------------------------------------- */
/* Bring the netif in line with the current settings. Idempotent: it only
 * touches the interface when the desired mode/address differs from the live
 * state, so it is safe to call both at boot and live after a settings change
 * (change IP / DHCP without a reboot). MUST run in the tcpip thread context
 * (called directly at boot before heavy traffic, and via tcpip_callback() for
 * the live path). */
static void apply_network_config(void)
{
    const settings_t* s = settings_get();
    const uint8_t mode = s->use_dhcp;   /* 0=static, 1=DHCP, 2=link-local */

    if (mode == NET_MODE_DHCP) {
        /* Already on DHCP (lease running or in progress) → nothing to do. */
        if (netif_dhcp_data(&gnetif) != NULL) {
            return;
        }
        /* Switch to DHCP: clear the address and start DHCP. */
        ip4_addr_t any;
        ip4_addr_set_zero(&any);
        netif_set_addr(&gnetif, &any, &any, &any);
        dhcp_start(&gnetif);
        return;
    }

    /* Static or link-local: compute the target address. */
    ip4_addr_t ip, mask, gw;
    if (mode == NET_MODE_LINKLOCAL) {
        /* Factory / unconfigured: deterministic 169.254.x.y (RFC 3927). */
        uint8_t ll[4];
        net_id_get_linklocal(ll);
        IP4_ADDR(&ip,   ll[0], ll[1], ll[2], ll[3]);
        IP4_ADDR(&mask, 255u, 255u, 0u, 0u);
        ip4_addr_set_zero(&gw);
    } else {
        IP4_ADDR(&ip,   s->ip[0],      s->ip[1],      s->ip[2],      s->ip[3]);
        IP4_ADDR(&mask, s->netmask[0], s->netmask[1], s->netmask[2], s->netmask[3]);
        IP4_ADDR(&gw,   s->gateway[0], s->gateway[1], s->gateway[2], s->gateway[3]);
    }

    /* Already at exactly this address (and not on DHCP) → nothing to do
     * (avoids needless churn when an unrelated setting is saved). */
    if (netif_dhcp_data(&gnetif) == NULL &&
        ip4_addr_cmp(netif_ip4_addr(&gnetif), &ip) &&
        ip4_addr_cmp(netif_ip4_netmask(&gnetif), &mask) &&
        ip4_addr_cmp(netif_ip4_gw(&gnetif), &gw)) {
        return;
    }

    /* Stop DHCP if it happens to be running, then load the address. */
    dhcp_release_and_stop(&gnetif);
    netif_set_addr(&gnetif, &ip, &mask, &gw);
}

/* tcpip_callback() trampoline so the live re-apply runs in the tcpip thread. */
static void apply_network_config_tcpip(void* arg)
{
    (void)arg;
    apply_network_config();
}

/* ---------------------------------------------------------------------------
 * Housekeeping
 * ------------------------------------------------------------------------- */
static void update_led_state_from_traffic(void)
{
    /* No physical link on either KSZ8863 port → distinct pattern. */
    if (!g_eth_any_link_up) {
        led_module_set_state(LED_STATE_NO_LINK);
        return;
    }

    const uint32_t now      = HAL_GetTick();
    const uint32_t last_req = modbus_app_last_request_tick();

    /* "Polling" for one double-blink period after every valid Modbus request.
     * A single request is a poll too, and clients that open a connection per
     * request must not be mistaken for silence, so the connection state is
     * deliberately not part of the condition. */
    const bool recent_traffic = (last_req != 0u) && ((now - last_req) <= LED_POLLING_PERIOD_MS);

    led_module_set_state(recent_traffic ? LED_STATE_POLLING : LED_STATE_NO_POLLING);
}

/* ---------------------------------------------------------------------------
 * Public entry point
 * ------------------------------------------------------------------------- */
void app_run(void)
{
    /* Load settings (or defaults) before anything that consumes them. */
    settings_init();
    settings_t* s = settings_get();

    /* Load write-once calibration coefficients from their dedicated Flash
     * sector. Un-committed (channel, range) pairs default to identity
     * (gain 1.0, offset 0.0). Survives factory reset. */
    calstore_init();

    /* Initialise the LED module and spawn its task BEFORE the button check.
     * The factory-reset burst (10 short blinks) is driven by led_module_tick()
     * which runs from led_task; if the task is not yet running, the burst is
     * silently dropped. */
    led_module_init((uint8_t)s->led_mode);
    const osThreadAttr_t led_attr_early = {
        .name = "LED", .stack_size = 256, .priority = osPriorityLow
    };
    osThreadNew(led_task, NULL, &led_attr_early);

    /* If the FACT_RES button is held at startup, blank-load defaults. We do
     * this synchronously here so that the rest of the boot uses defaults. */
    if (button_wait_held(BUTTON_HOLD_FOR_FACTORY_RESET_MS)) {
        perform_factory_reset();
        /* Not reached. */
    }

    /* Initialise the remaining hardware drivers. */
    aic_module_init();
    temp_module_init();
    modbus_app_init();

    /* Apply network configuration (static or DHCP). */
    apply_network_config();

    /* HAL_ETH_Init() has already run by the time we get here (LwIP init
     * called it from low_level_init()), so SMI/MIIM access is available.
     * Ensure KSZ8863 port 3 is in RMII mode (bit 6 of Global Control 4). */
    ksz8863_ensure_rmii_port3();

    /* Spawn the current-input acquisition task. The LED task was started earlier so that
     * the factory-reset burst is visible during the boot-time button-hold
     * path (see above). */
    const osThreadAttr_t aic_attr  = {
        .name = "AIC", .stack_size = 768, .priority = osPriorityHigh
    };
    osThreadNew(aic_task, NULL, &aic_attr);

    /* Spawn Modbus TCP server. */
    modbus_tcp_server_start();

    /* Open the discovery responder (UDP/20556) so the device can be found and
     * addressed by MAC even in the factory link-local state. */
    discovery_init();

    /* Housekeeping loop: feeds the watchdog, processes deferred actions and
     * keeps the LED state in sync with the Modbus traffic. */
    uint32_t tick = osKernelGetTickCount();
    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);

        /* Two-factor emergency calibration erase: a Modbus arm plus a physical
         * button confirmation within the arming window. While armed, the LED
         * shows a distinct pattern instead of the traffic state. */
        if (modbus_app_cal_erase_armed()) {
            led_module_set_state(LED_STATE_CAL_ARMED);
            if (button_is_pressed() && button_wait_held(CAL_ERASE_CONFIRM_MS)) {
                modbus_app_clear_cal_erase_arm();
                perform_cal_erase();
                /* Not reached. */
            }
        } else {
            update_led_state_from_traffic();
        }

        if (modbus_app_take_pending_save()) {
            HAL_IWDG_Refresh(&hiwdg);
            settings_save();
            /* Apply network settings live so an IP / DHCP change takes effect
             * immediately, without a reboot. Runs in the tcpip thread for
             * LwIP thread-safety; a no-op if the address is unchanged. */
            tcpip_callback(apply_network_config_tcpip, NULL);
        }

        /* Discovery-protocol deferred actions (Flash write / reset run here,
         * outside the tcpip thread that received the UDP request). */
        if (discovery_take_pending_save()) {
            HAL_IWDG_Refresh(&hiwdg);
            settings_save();
            tcpip_callback(apply_network_config_tcpip, NULL);
        }
        if (discovery_take_pending_factory()) {
            perform_factory_reset();
            /* Not reached. */
        }
        if (discovery_take_pending_reboot()) {
            NVIC_SystemReset();
            /* Not reached. */
        }
        if (modbus_app_take_pending_switch_reset()) {
            /* Operator-requested KSZ8863 recovery. Only flags the request:
             * the reset itself runs in the link-polling thread
             * (ksz8863_service) so all SMI access stays on one thread. */
            ksz8863_request_recovery();
        }
        if (modbus_app_take_pending_factory_reset()) {
            perform_factory_reset();
            /* Not reached. */
        }
        if (modbus_app_take_pending_bootloader()) {
            /* Ask the bootloader to stay active after the reset: write the
             * shared magic into the no-init RAM cell, then reset. The cell
             * survives a warm reset; the bootloader consumes it on entry. */
            *(volatile uint32_t *)BOOT_REQUEST_FLAG_ADDR = BOOT_REQUEST_MAGIC;
            const uint32_t bl_deadline = HAL_GetTick() + 200u;
            while (HAL_GetTick() < bl_deadline) {
                HAL_IWDG_Refresh(&hiwdg);
                osDelay(20);
            }
            NVIC_SystemReset();
        }
        if (modbus_app_take_pending_reboot()) {
            const uint32_t deadline = HAL_GetTick() + 200u;
            while (HAL_GetTick() < deadline) {
                HAL_IWDG_Refresh(&hiwdg);
                osDelay(20);
            }
            NVIC_SystemReset();
        }

        tick += 100u;
        osDelayUntil(tick);
    }
}
