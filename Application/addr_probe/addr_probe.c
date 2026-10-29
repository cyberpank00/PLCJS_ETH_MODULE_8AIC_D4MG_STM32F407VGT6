/**
 * @file  addr_probe.c
 * @brief Non-blocking IPv4 address-in-use probe (see addr_probe.h).
 */

#include "addr_probe.h"

#include <string.h>

#include "lwip/timeouts.h"

#define ETH_HDR_LEN     14u
#define ARP_PKT_LEN     28u
#define FRAME_LEN       (ETH_HDR_LEN + ARP_PKT_LEN)
#define ETHTYPE_ARP_HI  0x08u
#define ETHTYPE_ARP_LO  0x06u

/* Offsets inside the Ethernet frame. */
#define OFF_ETHTYPE     12u
#define OFF_SHA         22u
#define OFF_SPA         28u
#define OFF_TPA         38u

static struct {
    volatile bool    active;
    volatile bool    conflict;
    struct netif    *netif;
    uint8_t          cand[4];
    uint8_t          sent;
    addr_probe_done_cb cb;
    void            *arg;
} s;

static bool send_probe(void)
{
    struct pbuf *p = pbuf_alloc(PBUF_RAW, FRAME_LEN, PBUF_RAM);
    if (p == NULL) { return false; }

    uint8_t *f = (uint8_t *)p->payload;
    memset(f, 0, FRAME_LEN);
    memset(f, 0xFF, 6);                                   /* dst: broadcast */
    memcpy(f + 6, s.netif->hwaddr, 6);                    /* src            */
    f[OFF_ETHTYPE] = ETHTYPE_ARP_HI; f[OFF_ETHTYPE + 1] = ETHTYPE_ARP_LO;
    f[14] = 0x00; f[15] = 0x01;                           /* htype Ethernet */
    f[16] = 0x08; f[17] = 0x00;                           /* ptype IPv4     */
    f[18] = 6;    f[19] = 4;                              /* hlen, plen     */
    f[20] = 0x00; f[21] = 0x01;                           /* oper: request  */
    memcpy(f + OFF_SHA, s.netif->hwaddr, 6);              /* sha            */
    /* spa = 0.0.0.0 (probe), tha = 0 */
    memcpy(f + OFF_TPA, s.cand, 4);                       /* tpa = candidate */

    const err_t e = s.netif->linkoutput(s.netif, p);
    pbuf_free(p);
    return e == ERR_OK;
}

static void finish(void)
{
    if (!s.active) { return; }
    const bool in_use = s.conflict;
    addr_probe_done_cb cb = s.cb;
    void *arg = s.arg;
    s.active = false;
    if (cb != NULL) { cb(in_use, arg); }
}

static void probe_timer(void *unused)
{
    (void)unused;
    if (!s.active) { return; }
    if (s.conflict) { finish(); return; }

    if (s.sent < ADDR_PROBE_COUNT) {
        (void)send_probe();
        s.sent++;
        sys_timeout(s.sent < ADDR_PROBE_COUNT ? ADDR_PROBE_INTERVAL_MS : ADDR_PROBE_WAIT_MS,
                    probe_timer, NULL);
        return;
    }
    finish();
}

bool addr_probe_busy(void) { return s.active; }

bool addr_probe_start(struct netif *netif, const ip4_addr_t *cand,
                      addr_probe_done_cb cb, void *arg)
{
    if (s.active || netif == NULL || cand == NULL) { return false; }

    memset(&s, 0, sizeof(s));
    s.netif = netif;
    memcpy(s.cand, &cand->addr, 4);
    s.cb  = cb;
    s.arg = arg;
    s.active = true;
    sys_timeout(0u, probe_timer, NULL);
    return true;
}

void addr_probe_input(const struct pbuf *p)
{
    if (!s.active || p == NULL || p->len < FRAME_LEN) { return; }

    const uint8_t *f = (const uint8_t *)p->payload;
    if (f[OFF_ETHTYPE] != ETHTYPE_ARP_HI || f[OFF_ETHTYPE + 1] != ETHTYPE_ARP_LO) { return; }
    if (memcmp(f + OFF_SHA, s.netif->hwaddr, 6) == 0) { return; }   /* our own frame */

    static const uint8_t zero[4] = { 0, 0, 0, 0 };
    /* Another station already uses the candidate address ... */
    if (memcmp(f + OFF_SPA, s.cand, 4) == 0) { s.conflict = true; return; }
    /* ... or is probing for the very same address at the same time. */
    if (memcmp(f + OFF_SPA, zero, 4) == 0 && memcmp(f + OFF_TPA, s.cand, 4) == 0) {
        s.conflict = true;
    }
}
