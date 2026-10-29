/**
  ******************************************************************************
  * @file    discovery.c
  * @brief   PLCJS Discovery Protocol (PDP) responder — UDP/20556 broadcast.
  *
  * Frame (16-byte header, little payload):
  *   magic[4]="PLCD", version=1, opcode, txid(BE), target_mac[6], len(BE), payload
  * Device replies via UDP broadcast, echoing txid, with its own MAC in the
  * header and the response opcode (request opcode | 0x80).
  *
  * Opcodes (request / response):
  *   0x01/0x81 IDENTIFY  -> {product_id(BE32), hw(BE16), fw(BE16), net_mode,
  *                           in_bootloader, ip[4], mask[4], gw[4], name[16]}
  *   0x02/0x82 SET_NET   -> req {mode, ip[4], mask[4], gw[4]} ; resp {status}
  *   0x03/0x83 SET_NAME  -> req {name[..16]}                  ; resp {status}
  *   0x04/0x84 FLASH_LED -> req {seconds}                     ; resp {status}
  *   0x05/0x85 REBOOT    -> resp {status} then reset
  *   0x06/0x86 FACTORY   -> resp {status} then factory reset
  ******************************************************************************
  */
#include "discovery.h"

#include "lwip/opt.h"
#include "lwip/udp.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/def.h"
#include "lwip/pbuf.h"

#include <string.h>

#include "net_id.h"
#include "addr_probe.h"
#include "settings.h"
#include "led_module.h"
#include "fw_header.h"

/* Set to 1 in the bootloader build so IDENTIFY reports "in bootloader". */
#ifndef DISCOVERY_IN_BOOTLOADER
#define DISCOVERY_IN_BOOTLOADER 0
#endif

extern struct netif gnetif;

#define PDP_PORT            20556u
#define PDP_VERSION         1u
#define PDP_RESP_FLAG       0x80u

#define PDP_OP_IDENTIFY     0x01u
#define PDP_OP_SET_NET      0x02u
#define PDP_OP_SET_NAME     0x03u
#define PDP_OP_FLASH_LED    0x04u
#define PDP_OP_REBOOT       0x05u
#define PDP_OP_FACTORY      0x06u

#define PDP_HDR_LEN         16u
#define PDP_STATUS_OK       0u
#define PDP_STATUS_ERR      1u

static struct udp_pcb*  s_pcb;
static volatile uint8_t s_pending_save    = 0u;
static volatile uint8_t s_pending_reboot  = 0u;
static volatile uint8_t s_pending_factory = 0u;

bool discovery_take_pending_save(void)    { uint8_t v = s_pending_save;    s_pending_save    = 0u; return v != 0u; }
bool discovery_take_pending_reboot(void)  { uint8_t v = s_pending_reboot;  s_pending_reboot  = 0u; return v != 0u; }
bool discovery_take_pending_factory(void) { uint8_t v = s_pending_factory; s_pending_factory = 0u; return v != 0u; }

static void put16(uint8_t* p, uint16_t v) { uint16_t n = lwip_htons(v); memcpy(p, &n, 2); }
static void put32(uint8_t* p, uint32_t v) { uint32_t n = lwip_htonl(v); memcpy(p, &n, 4); }

/* Build and broadcast a PDP response frame. */
static void pdp_send(uint8_t opcode, const uint8_t txid[2],
                     const uint8_t* payload, uint16_t plen)
{
    const uint16_t total = (uint16_t)(PDP_HDR_LEN + plen);
    struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, total, PBUF_RAM);
    if (p == NULL) { return; }

    uint8_t* b = (uint8_t*)p->payload;
    memset(b, 0, total);
    b[0] = 'P'; b[1] = 'L'; b[2] = 'C'; b[3] = 'D';
    b[4] = PDP_VERSION;
    b[5] = opcode;
    b[6] = txid[0]; b[7] = txid[1];
    net_id_get_mac(&b[8]);            /* our MAC */
    put16(&b[14], plen);
    if (plen != 0u && payload != NULL) { memcpy(&b[16], payload, plen); }

    udp_sendto(s_pcb, p, IP_ADDR_BROADCAST, PDP_PORT);
    pbuf_free(p);
}

static void pdp_reply_status(uint8_t req_op, const uint8_t txid[2], uint8_t status)
{
    pdp_send((uint8_t)(req_op | PDP_RESP_FLAG), txid, &status, 1u);
}

/* SET_NET is applied only after the address passed the in-use probe. */
static bool    s_set_pending;
static uint8_t s_set_txid[2];
static uint8_t s_set_mode;
static uint8_t s_set_addr[12];   /* ip, netmask, gateway (static mode) */

static void set_net_commit(uint8_t mode, const uint8_t addr[12])
{
    settings_t* s = settings_get();
    s->use_dhcp = mode;
    if (mode == NET_MODE_STATIC) {
        memcpy(s->ip,      &addr[0], 4);
        memcpy(s->netmask, &addr[4], 4);
        memcpy(s->gateway, &addr[8], 4);
    }
    s_pending_save = 1u;
}

static void set_net_probe_done(bool in_use, void* arg)
{
    LWIP_UNUSED_ARG(arg);
    s_set_pending = false;
    if (in_use) {
        pdp_reply_status(PDP_OP_SET_NET, s_set_txid, PDP_STATUS_ERR);
        return;
    }
    set_net_commit(s_set_mode, s_set_addr);
    pdp_reply_status(PDP_OP_SET_NET, s_set_txid, PDP_STATUS_OK);
}

static void pdp_recv(void* arg, struct udp_pcb* pcb, struct pbuf* p,
                     const ip_addr_t* addr, u16_t port)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(pcb);
    LWIP_UNUSED_ARG(addr);
    LWIP_UNUSED_ARG(port);

    uint8_t buf[128];
    const u16_t len = pbuf_copy_partial(p, buf, sizeof(buf), 0);
    pbuf_free(p);

    if (len < PDP_HDR_LEN) { return; }
    if (!(buf[0] == 'P' && buf[1] == 'L' && buf[2] == 'C' && buf[3] == 'D')) { return; }
    if (buf[4] != PDP_VERSION) { return; }

    const uint8_t opcode = buf[5];
    if ((opcode & PDP_RESP_FLAG) != 0u) { return; }  /* ignore responses */

    const uint8_t* txid = &buf[6];

    /* Target-MAC filter: act only if broadcast (all-zero) or our MAC. */
    uint8_t mymac[6];
    net_id_get_mac(mymac);
    const uint8_t* tgt = &buf[8];
    const bool bcast = ((tgt[0] | tgt[1] | tgt[2] | tgt[3] | tgt[4] | tgt[5]) == 0u);
    if (!bcast && memcmp(tgt, mymac, 6) != 0) { return; }

    uint16_t plen;
    { uint16_t n; memcpy(&n, &buf[14], 2); plen = lwip_ntohs(n); }
    if ((uint32_t)PDP_HDR_LEN + plen > len) {
        plen = (len > PDP_HDR_LEN) ? (uint16_t)(len - PDP_HDR_LEN) : 0u;
    }
    const uint8_t* pl = &buf[PDP_HDR_LEN];

    switch (opcode) {
    case PDP_OP_IDENTIFY: {
        uint8_t r[64];
        memset(r, 0, sizeof(r));
        const settings_t* s = settings_get();
        put32(&r[0], (uint32_t)FW_PRODUCT_ID);
        put16(&r[4], (uint16_t)FW_HW_REVISION);
        put16(&r[6], (uint16_t)FW_VERSION_VALUE);
        r[8] = s->use_dhcp;                    /* net_mode 0/1/2 */
        r[9] = (uint8_t)DISCOVERY_IN_BOOTLOADER;
        const u32_t ip = ip4_addr_get_u32(netif_ip4_addr(&gnetif));
        const u32_t mk = ip4_addr_get_u32(netif_ip4_netmask(&gnetif));
        const u32_t gw = ip4_addr_get_u32(netif_ip4_gw(&gnetif));
        memcpy(&r[10], &ip, 4);
        memcpy(&r[14], &mk, 4);
        memcpy(&r[18], &gw, 4);
        /* Emit a fixed 16-byte name field on the wire (zero-padded) regardless
         * of the per-variant SETTINGS_NAME_LEN, so every module returns an
         * identical 38-byte IDENTIFY payload the tools parse uniformly. */
        memcpy(&r[22], s->name, (SETTINGS_NAME_LEN < 16u) ? SETTINGS_NAME_LEN : 16u);
        pdp_send((uint8_t)(PDP_OP_IDENTIFY | PDP_RESP_FLAG), txid, r, 22u + 16u);
        break;
    }
    case PDP_OP_SET_NET: {
        if (plen < 1u) { pdp_reply_status(opcode, txid, PDP_STATUS_ERR); break; }
        const uint8_t mode = pl[0];
        if (mode > NET_MODE_LINKLOCAL) { pdp_reply_status(opcode, txid, PDP_STATUS_ERR); break; }
        if (mode == NET_MODE_STATIC && plen < 13u) { pdp_reply_status(opcode, txid, PDP_STATUS_ERR); break; }

        uint8_t addr[12] = {0};
        ip4_addr_t cand;
        bool probe = true;
        if (mode == NET_MODE_STATIC) {
            memcpy(addr, &pl[1], 12);
            IP4_ADDR(&cand, pl[1], pl[2], pl[3], pl[4]);
        } else if (mode == NET_MODE_LINKLOCAL) {
            uint8_t ll[4];
            net_id_get_linklocal(ll);
            IP4_ADDR(&cand, ll[0], ll[1], ll[2], ll[3]);
        } else {
            probe = false;   /* DHCP assigns nothing here */
        }

        /* Re-applying the address the device already has needs no probe. */
        if (!probe || ip4_addr_cmp(&cand, netif_ip4_addr(&gnetif))) {
            set_net_commit(mode, addr);
            pdp_reply_status(opcode, txid, PDP_STATUS_OK);
            break;
        }
        /* A retransmit of the request being probed gets the single answer later. */
        if (s_set_pending) {
            if (memcmp(txid, s_set_txid, 2) != 0) { pdp_reply_status(opcode, txid, PDP_STATUS_ERR); }
            break;
        }
        s_set_mode = mode;
        memcpy(s_set_addr, addr, sizeof(s_set_addr));
        memcpy(s_set_txid, txid, 2);
        s_set_pending = true;
        if (!addr_probe_start(&gnetif, &cand, set_net_probe_done, NULL)) {
            s_set_pending = false;
            pdp_reply_status(opcode, txid, PDP_STATUS_ERR);
        }
        break;
    }
    case PDP_OP_SET_NAME: {
        settings_t* s = settings_get();
        memset(s->name, 0, SETTINGS_NAME_LEN);
        uint16_t n = (plen < (SETTINGS_NAME_LEN - 1u)) ? plen : (SETTINGS_NAME_LEN - 1u);
        memcpy(s->name, pl, n);
        s_pending_save = 1u;
        pdp_reply_status(opcode, txid, PDP_STATUS_OK);
        break;
    }
    case PDP_OP_FLASH_LED: {
        const uint8_t secs = (plen >= 1u) ? pl[0] : 10u;
        led_module_signal_identify((uint32_t)secs * 1000u);
        pdp_reply_status(opcode, txid, PDP_STATUS_OK);
        break;
    }
    case PDP_OP_REBOOT: {
        s_pending_reboot = 1u;
        pdp_reply_status(opcode, txid, PDP_STATUS_OK);
        break;
    }
    case PDP_OP_FACTORY: {
        s_pending_factory = 1u;
        pdp_reply_status(opcode, txid, PDP_STATUS_OK);
        break;
    }
    default:
        break;
    }
}

void discovery_init(void)
{
    if (s_pcb != NULL) { return; }
    s_pcb = udp_new();
    if (s_pcb == NULL) { return; }
    ip_set_option(s_pcb, SOF_BROADCAST);   /* allow send/recv of broadcast */
    if (udp_bind(s_pcb, IP_ANY_TYPE, PDP_PORT) != ERR_OK) {
        udp_remove(s_pcb);
        s_pcb = NULL;
        return;
    }
    udp_recv(s_pcb, pdp_recv, NULL);
}
