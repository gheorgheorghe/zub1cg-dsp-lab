/*
 * ZUBoard 1CG Stage 0 — Cortex-A53 lwIP UDP <-> AXI-Lite & AXI-DMA Bridge
 * Build in Vitis 2024.2 Unified IDE (standalone + lwip220 on psu_cortexa53_0).
 *
 * UDP Protocol (Port 5000, little-endian uint32 words):
 *   CMD_REG_READ  (0x01): req = [cmd, byte_offset]          -> rsp = [0x81, value]
 *   CMD_REG_WRITE (0x02): req = [cmd, byte_offset, value]   -> rsp = [0x82, value]
 *   CMD_DMA_CAP   (0x03): req = [cmd, n_samples]            -> rsp = chunked UDP packets
 *                         each packet = [0x83, seq_idx, sample_0, ..., sample_K]
 */

#include <stdint.h>
#include <string.h>
#include "xil_printf.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "xparameters.h"

#include "lwip/init.h"
#include "lwip/udp.h"
#include "netif/xadapter.h"

/* Must match hw/scripts/create_bd.tcl address assignment */
#define DSP_SLOT_BASEADDR   0x80010000U
#define UDP_PORT            5000U
#define MAX_CAPTURE_SAMPLES 16384U
#define SAMPLES_PER_UDP_PKT 256U /* 1024 bytes payload + 8 byte header (< 1500 MTU) */

#define CMD_REG_READ        0x01U
#define CMD_REG_WRITE       0x02U
#define CMD_DMA_CAP         0x03U

static XAxiDma axi_dma;
static struct netif server_netif;

/* Cache-line aligned (64-byte) DDR4 DMA capture buffer */
static uint32_t dma_rx_buf[MAX_CAPTURE_SAMPLES] __attribute__((aligned(64)));

static int init_dma(void) {
    XAxiDma_Config *cfg = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_BASEADDR);
    if (!cfg) {
        xil_printf("ERROR: AXI-DMA config lookup failed\r\n");
        return XST_FAILURE;
    }
    if (XAxiDma_CfgInitialize(&axi_dma, cfg) != XST_SUCCESS) {
        xil_printf("ERROR: AXI-DMA init failed\r\n");
        return XST_FAILURE;
    }
    XAxiDma_IntrDisable(&axi_dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrDisable(&axi_dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    return XST_SUCCESS;
}

static void send_udp_words(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port,
                           const uint32_t *words, uint32_t n_words) {
    u16_t nbytes = (u16_t)(n_words * sizeof(uint32_t));
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, nbytes, PBUF_RAM);
    if (!p) return;
    memcpy(p->payload, words, nbytes);
    udp_sendto(pcb, p, addr, port);
    pbuf_free(p);
}

static void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port) {
    (void)arg;
    if (!p || p->len < 8) {
        if (p) pbuf_free(p);
        return;
    }

    uint32_t req[4] = {0};
    u16_t copy_len = (p->len < sizeof(req)) ? p->len : sizeof(req);
    pbuf_copy_partial(p, req, copy_len, 0);
    pbuf_free(p);

    uint32_t cmd = req[0];

    if (cmd == CMD_REG_READ) {
        uint32_t offset = req[1] & 0x3CU;
        uint32_t rsp[2] = {0x81U, Xil_In32(DSP_SLOT_BASEADDR + offset)};
        send_udp_words(pcb, addr, port, rsp, 2);

    } else if (cmd == CMD_REG_WRITE && copy_len >= 12) {
        uint32_t offset = req[1] & 0x3CU;
        uint32_t val    = req[2];
        Xil_Out32(DSP_SLOT_BASEADDR + offset, val);
        uint32_t rsp[2] = {0x82U, Xil_In32(DSP_SLOT_BASEADDR + offset)};
        send_udp_words(pcb, addr, port, rsp, 2);

    } else if (cmd == CMD_DMA_CAP) {
        uint32_t n_samples = req[1];
        if (n_samples == 0) n_samples = 1024;
        if (n_samples > MAX_CAPTURE_SAMPLES) n_samples = MAX_CAPTURE_SAMPLES;

        /* Program PKT_LEN register (0x08) to match capture length, pulse soft reset */
        uint32_t ctrl = Xil_In32(DSP_SLOT_BASEADDR + 0x00U);
        Xil_Out32(DSP_SLOT_BASEADDR + 0x08U, n_samples);
        Xil_Out32(DSP_SLOT_BASEADDR + 0x00U, (ctrl | 0x4U) & ~0x1U); /* soft_rst=1, enable=0 */
        Xil_Out32(DSP_SLOT_BASEADDR + 0x00U, ctrl & ~0x5U);          /* release rst */

        /* Flush/invalidate RX buffer before DMA writes to DDR4 */
        uint32_t nbytes = n_samples * sizeof(uint32_t);
        Xil_DCacheInvalidateRange((INTPTR)dma_rx_buf, nbytes);

        int st = XAxiDma_SimpleTransfer(&axi_dma, (UINTPTR)dma_rx_buf, nbytes,
                                        XAXIDMA_DEVICE_TO_DMA);
        if (st == XST_SUCCESS) {
            /* Enable streaming slot and wait for S2MM completion */
            Xil_Out32(DSP_SLOT_BASEADDR + 0x00U, ctrl | 0x1U);
            while (XAxiDma_Busy(&axi_dma, XAXIDMA_DEVICE_TO_DMA)) {
                /* Poll S2MM */
            }
            /* Mandatory ZynqMP cache invalidate before CPU reads DMA data */
            Xil_DCacheInvalidateRange((INTPTR)dma_rx_buf, nbytes);

            /* Stream captured buffer back over UDP in MTU-safe chunks */
            uint32_t pkt[2 + SAMPLES_PER_UDP_PKT];
            uint32_t sent = 0;
            uint32_t seq  = 0;
            while (sent < n_samples) {
                uint32_t chunk = n_samples - sent;
                if (chunk > SAMPLES_PER_UDP_PKT) chunk = SAMPLES_PER_UDP_PKT;
                pkt[0] = 0x83U;
                pkt[1] = seq++;
                memcpy(&pkt[2], &dma_rx_buf[sent], chunk * sizeof(uint32_t));
                send_udp_words(pcb, addr, port, pkt, 2 + chunk);
                sent += chunk;
            }
        }
    }
}

int main(void) {
    ip_addr_t ipaddr, netmask, gw;
    unsigned char mac_addr[] = {0x00, 0x0A, 0x35, 0x01, 0x02, 0x03};

    xil_printf("\r\n=== ZUBoard 1CG Stage 0 A53 UDP-DMA Bridge ===\r\n");
    xil_printf("DSP Slot Version: 0x%08X\r\n", Xil_In32(DSP_SLOT_BASEADDR + 0x38U));

    if (init_dma() != XST_SUCCESS) {
        return -1;
    }

    lwip_init();
    IP4_ADDR(&ipaddr,  192, 168, 1, 10);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw,      192, 168, 1, 1);

    if (!xemac_add(&server_netif, &ipaddr, &netmask, &gw, mac_addr,
                   XPAR_XEMACPS_0_BASEADDR)) {
        xil_printf("ERROR: xemac_add failed\r\n");
        return -1;
    }
    netif_set_default(&server_netif);
    netif_set_up(&server_netif);

    struct udp_pcb *pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);
    udp_recv(pcb, udp_recv_cb, NULL);

    xil_printf("UDP server listening on 192.168.1.10:%u\r\n", UDP_PORT);
    while (1) {
        xemacif_input(&server_netif);
    }
    return 0;
}
