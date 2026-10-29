/**
 * @file  addr_probe.h
 * @brief Non-blocking IPv4 address-in-use probe (RFC 5227 style ARP probe).
 *
 * Used by the PDP SET_NET handler before an address is applied. Three ARP probes
 * (sender 0.0.0.0, target = candidate) are sent from the lwIP timer context; any
 * ARP frame that shows another station owning the candidate - a reply or request
 * with that sender IP, or another host probing the same target - reports the
 * address as in use. This covers hosts that do not speak PDP (PC, PLC).
 *
 * The receive path must call addr_probe_input() for every received frame before
 * handing it to lwIP. The same file is mirrored in every application firmware.
 */
#ifndef ADDR_PROBE_H
#define ADDR_PROBE_H

#include <stdbool.h>
#include <stdint.h>

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADDR_PROBE_COUNT        3u      /* probes sent                         */
#define ADDR_PROBE_INTERVAL_MS  200u    /* between probes                      */
#define ADDR_PROBE_WAIT_MS      300u    /* wait for a late answer after last   */

typedef void (*addr_probe_done_cb)(bool in_use, void *arg);

/* Start probing `cand` on `netif`. Returns false if a probe is already running
 * or the frame cannot be built. `cb` is called once, from the lwIP timer
 * context, after ~ADDR_PROBE_COUNT*INTERVAL + WAIT ms (earlier on a conflict). */
bool addr_probe_start(struct netif *netif, const ip4_addr_t *cand,
                      addr_probe_done_cb cb, void *arg);

bool addr_probe_busy(void);

/* Receive hook: inspect a frame (does not consume or modify it). */
void addr_probe_input(const struct pbuf *p);

#ifdef __cplusplus
}
#endif

#endif /* ADDR_PROBE_H */
