/**
  ******************************************************************************
  * @file    modbus_tcp_server.c
  * @brief   Multi-client Modbus TCP server task on top of LwIP netconn.
  *
  * Up to MB_MAX_CLIENTS TCP clients are served concurrently by one task that
  * round-robins over the slots. Each slot has its own nanoMODBUS server
  * context and RX/TX buffers; the register callbacks are shared and executed
  * strictly sequentially, so a write from one client is simply followed by
  * the next client's request ("last write wins", as with any Modbus device).
  *
  * Slot policy:
  *   - a new connection takes a free slot;
  *   - when every slot is busy the new connection evicts the client that has
  *     been silent the longest (newest-wins), so a master reconnecting after a
  *     cable pull / switch reboot never waits for a stale half-open connection
  *     to time out;
  *   - a connected-but-silent client is dropped after MB_IDLE_DROP_MS, TCP
  *     keep-alive catches dead peers at the stack level, link-down drops all.
  *
  * Idle slots are polled with a very short first-byte timeout so a request on
  * one connection is never delayed by the others waiting for data.
  ******************************************************************************
  */

#include "modbus_tcp_server.h"

#include <string.h>

#include "cmsis_os.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"

#include "modbus_app.h"
#include "nanomodbus.h"
#include "settings.h"

/* Debounced link state from ethernet_link_thread (follows the netif, 2 s to
 * drop). NOT the instantaneous g_eth_any_link_up: the KSZ8863 port blips
 * link-down several times while auto-negotiation settles after a cable is
 * plugged back, and dropping the clients on each blip kept killing freshly
 * accepted connections for ~10 s. */
extern volatile uint8_t g_eth_link_stable;

/* ---------------------------------------------------------------------------
 * Tunables
 * ------------------------------------------------------------------------- */
#define MB_MAX_CLIENTS          4u

#define MB_TX_BUF_SIZE          280u  /* max Modbus TCP frame: 7 MBAP + 253 PDU */

/* TCP keep-alive parameters (in milliseconds) */
#define MB_KEEPALIVE_IDLE_MS    10000u  /* 10 s idle before first probe  */
#define MB_KEEPALIVE_INTVL_MS    2000u  /*  2 s between probes           */
#define MB_KEEPALIVE_CNT            3u  /*  3 probes → dead after ~16 s  */

/* Poll timing: the read timeout bounds how long one poll of a slot waits for
 * a request to BEGIN. It is deliberately tiny so the task can cycle through
 * every slot (and the accept queue) with ~ms latency; a slot that has data
 * proceeds immediately. The byte timeout bounds the gap between bytes once a
 * frame has started (frames normally arrive in one segment anyway). */
#define MB_READ_TIMEOUT_MS          2u
#define MB_BYTE_TIMEOUT_MS       1000u

/* Drop a connected-but-silent client after this long with no valid request
 * (safety net alongside TCP keep-alive). */
#define MB_IDLE_DROP_MS         30000u

/* Sleep when no client is connected at all. */
#define MB_IDLE_SLEEP_MS            5u

/* ---------------------------------------------------------------------------
 * Per-client slot: netconn + nanoMODBUS context + IO buffers.
 * ------------------------------------------------------------------------- */
typedef struct {
    struct netconn* conn;           /* NULL = slot free                        */
    struct netbuf*  inbuf;
    char*           inbuf_data;
    u16_t           inbuf_len;
    u16_t           inbuf_pos;
    uint8_t         txbuf[MB_TX_BUF_SIZE];
    u16_t           txbuf_len;
    uint32_t        last_activity;  /* tick of the last valid request          */
    nmbs_t          mb;
} mb_client_t;

/* ---------------------------------------------------------------------------
 * Module state
 * ------------------------------------------------------------------------- */
static mb_client_t      s_clients[MB_MAX_CLIENTS];
static volatile uint8_t s_client_count = 0u;
static osThreadId_t     s_server_task  = NULL;

bool modbus_tcp_server_has_client(void)
{
    return s_client_count != 0u;
}

uint8_t modbus_tcp_server_client_count(void)
{
    return s_client_count;
}

/* ---------------------------------------------------------------------------
 * nanoMODBUS platform callbacks (arg = the slot)
 * ------------------------------------------------------------------------- */
static void inbuf_release(mb_client_t* c)
{
    if (c->inbuf != NULL) {
        netbuf_delete(c->inbuf);
        c->inbuf      = NULL;
        c->inbuf_data = NULL;
        c->inbuf_len  = 0;
        c->inbuf_pos  = 0;
    }
}

static int mb_read_byte(uint8_t* b, int32_t timeout_ms, void* arg)
{
    mb_client_t* c = (mb_client_t*)arg;

    if (c->inbuf == NULL || c->inbuf_pos >= c->inbuf_len) {
        inbuf_release(c);

        netconn_set_recvtimeout(c->conn, (timeout_ms < 0) ? 0 : (u32_t)timeout_ms);

        const err_t err = netconn_recv(c->conn, &c->inbuf);
        if (err == ERR_TIMEOUT) {
            return 0;
        }
        if (err != ERR_OK) {
            return -1;
        }

        netbuf_data(c->inbuf, (void**)&c->inbuf_data, &c->inbuf_len);
        c->inbuf_pos = 0;
    }

    *b = (uint8_t)c->inbuf_data[c->inbuf_pos++];
    return 1;
}

/* Buffer bytes instead of sending one-by-one. The complete response is
 * flushed to TCP after nmbs_server_poll() returns. */
static int mb_write_byte(uint8_t b, int32_t timeout_ms, void* arg)
{
    (void)timeout_ms;
    mb_client_t* c = (mb_client_t*)arg;
    if (c->txbuf_len >= MB_TX_BUF_SIZE) {
        return -1;  /* buffer overflow — should never happen */
    }
    c->txbuf[c->txbuf_len++] = b;
    return 1;
}

/* Flush the buffered TX data as a single TCP segment. */
static int mb_flush(mb_client_t* c)
{
    if (c->txbuf_len == 0u) {
        return 0;
    }
    const err_t err = netconn_write(c->conn, c->txbuf, c->txbuf_len, NETCONN_COPY);
    c->txbuf_len = 0u;
    return (err == ERR_OK) ? 0 : -1;
}

static void mb_sleep(uint32_t ms, void* arg)
{
    (void)arg;
    osDelay(ms);
}

/* ---------------------------------------------------------------------------
 * Slot lifecycle
 * ------------------------------------------------------------------------- */
/* Abort the TCP pcb behind a netconn (RST instead of FIN), synchronously,
 * under the LwIP core lock. The netconn's err callback detaches the pcb
 * (pcb.tcp = NULL), so the following netconn_delete() only frees the netconn.
 *
 * Must be synchronous: with LWIP_TCPIP_CORE_LOCKING the netconn API calls run
 * directly in the caller under the core lock, whereas tcpip_callback() is
 * queued to the tcpip thread. A queued abort would run AFTER netconn_delete()
 * has freed (and the next accept has reused) the netconn — and kill the new
 * connection instead. That was fw 2.8: every accepted client died at once. */
static void conn_abort(struct netconn* conn)
{
#if LWIP_TCPIP_CORE_LOCKING
    LOCK_TCPIP_CORE();
    if (conn->pcb.tcp != NULL) {
        tcp_abort(conn->pcb.tcp);
    }
    UNLOCK_TCPIP_CORE();
#else
#error "conn_abort() relies on LWIP_TCPIP_CORE_LOCKING; without it use a blocking tcpip_api_call()"
#endif
}

/* Every slot is closed with an ABORT, never a graceful FIN. The peers we
 * drop here are dead or being evicted: a FIN to a peer that vanished (cable
 * pull) leaves the pcb in FIN_WAIT_1 retransmitting for minutes, and a few
 * such pcbs exhaust MEMP_NUM_TCP_PCB — LwIP then silently drops every new
 * SYN and the module looks dead to Modbus while PDP still answers. Seen on
 * the bench after ~5 cable pulls. tcp_abort() frees the pcb at once; a live
 * Modbus client copes with RST exactly as with FIN. */
static void client_close(mb_client_t* c)
{
    inbuf_release(c);
    if (c->conn != NULL) {
        conn_abort(c->conn);
        netconn_delete(c->conn);
        c->conn = NULL;
        if (s_client_count > 0u) { s_client_count--; }
    }
}

static void close_all(void)
{
    for (uint8_t i = 0; i < MB_MAX_CLIENTS; i++) {
        client_close(&s_clients[i]);
    }
}

/* Configure keep-alive and build the nanoMODBUS server context for a freshly
 * accepted connection in slot @p c. Returns false (connection dropped) if the
 * server context could not be created. */
static bool client_setup(mb_client_t* c, struct netconn* conn)
{
    /* Enable TCP keep-alive so a cable-pull is also detected at the stack
     * level (~16 s) even if the peer never reconnects. */
    ip_set_option(conn->pcb.tcp, SOF_KEEPALIVE);
    conn->pcb.tcp->keep_idle  = MB_KEEPALIVE_IDLE_MS;
    conn->pcb.tcp->keep_intvl = MB_KEEPALIVE_INTVL_MS;
    conn->pcb.tcp->keep_cnt   = MB_KEEPALIVE_CNT;

    memset(c, 0, sizeof(*c));
    c->conn = conn;

    /* nmbs_server_create() copies the conf, so a local one is fine. */
    nmbs_platform_conf platform;
    platform.transport  = NMBS_TRANSPORT_TCP;
    platform.read_byte  = mb_read_byte;
    platform.write_byte = mb_write_byte;
    platform.sleep      = mb_sleep;
    platform.arg        = c;

    if (nmbs_server_create(&c->mb, settings_get()->modbus_slave_id,
                           &platform, modbus_app_get_callbacks()) != NMBS_ERROR_NONE) {
        netconn_close(conn);
        netconn_delete(conn);
        c->conn = NULL;
        return false;
    }
    nmbs_set_read_timeout(&c->mb, MB_READ_TIMEOUT_MS);
    nmbs_set_byte_timeout(&c->mb, MB_BYTE_TIMEOUT_MS);

    c->last_activity = osKernelGetTickCount();
    s_client_count++;
    return true;
}

/* Pick the slot for a new connection: a free one, else the least recently
 * active client is evicted (newest-wins when full). */
static mb_client_t* slot_for_new_connection(void)
{
    mb_client_t* victim = &s_clients[0];
    for (uint8_t i = 0; i < MB_MAX_CLIENTS; i++) {
        mb_client_t* c = &s_clients[i];
        if (c->conn == NULL) {
            return c;
        }
        if ((int32_t)(c->last_activity - victim->last_activity) < 0) {
            victim = c;
        }
    }
    client_close(victim);
    return victim;
}

/* ---------------------------------------------------------------------------
 * Server task entry point
 * ------------------------------------------------------------------------- */
static void modbus_tcp_server_thread(void* arg)
{
    (void)arg;

    struct netconn* listener = netconn_new(NETCONN_TCP);
    if (listener == NULL) {
        for (;;) { osDelay(1000); }
    }

    const uint16_t port = settings_get()->modbus_tcp_port;
    if (netconn_bind(listener, IP_ADDR_ANY, port) != ERR_OK) {
        netconn_delete(listener);
        for (;;) { osDelay(1000); }
    }

    netconn_listen(listener);
    /* Accept must never block the loop; we poll it every iteration. */
    netconn_set_nonblocking(listener, 1);

    memset(s_clients, 0, sizeof(s_clients));

    for (;;) {
        /* 1. Accept everything pending. Each connection takes a free slot or
         *    evicts the longest-silent client, so a reconnect storm ends with
         *    the newest connections in the slots and nothing left queued. */
        struct netconn* incoming = NULL;
        while (netconn_accept(listener, &incoming) == ERR_OK && incoming != NULL) {
            (void)client_setup(slot_for_new_connection(), incoming);
            incoming = NULL;
        }

        /* 2. Link down → drop everyone, nothing else to do. */
        if (!g_eth_link_stable) {
            if (s_client_count != 0u) { close_all(); }
            osDelay(MB_IDLE_SLEEP_MS);
            continue;
        }

        /* 3. Service every connected slot with one short, bounded poll. */
        if (s_client_count == 0u) {
            osDelay(MB_IDLE_SLEEP_MS);
            continue;
        }
        for (uint8_t i = 0; i < MB_MAX_CLIENTS; i++) {
            mb_client_t* c = &s_clients[i];
            if (c->conn == NULL) { continue; }

            c->txbuf_len = 0u;
            const nmbs_error e = nmbs_server_poll(&c->mb);
            if (e == NMBS_ERROR_NONE) {
                /* nanoMODBUS returns NONE both for "request handled" and for
                 * "nothing arrived within the read timeout" (first-byte
                 * timeout). Only a produced response proves a request: every
                 * valid unicast request gets one (data or exception). Treating
                 * the idle case as a request kept last_activity fresh forever
                 * (idle-drop never fired), lit the POLLING pattern for any
                 * connected-but-silent client and — worst — fed the comms-loss
                 * timer of the output modules from a silent connection. */
                if (c->txbuf_len != 0u) {
                    mb_flush(c);                   /* one TCP segment per response */
                    modbus_app_notify_request();
                    c->last_activity = osKernelGetTickCount();
                } else if ((osKernelGetTickCount() - c->last_activity) >= MB_IDLE_DROP_MS) {
                    client_close(c);               /* silent peer → free the slot */
                }
            } else if (e == NMBS_ERROR_TIMEOUT) {
                if ((osKernelGetTickCount() - c->last_activity) >= MB_IDLE_DROP_MS) {
                    client_close(c);               /* silent peer → free the slot */
                }
            } else {
                client_close(c);                   /* transport error → peer gone */
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * Public start
 * ------------------------------------------------------------------------- */
void modbus_tcp_server_start(void)
{
    if (s_server_task != NULL) {
        return;
    }
    const osThreadAttr_t attr = {
        .name       = "ModbusSrv",
        .stack_size = 2048,
        .priority   = osPriorityNormal,
    };
    s_server_task = osThreadNew(modbus_tcp_server_thread, NULL, &attr);
}
