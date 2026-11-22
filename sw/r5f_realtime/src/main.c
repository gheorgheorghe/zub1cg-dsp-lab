/*
 * ZUBoard 1CG — Cortex-R5F Hard Real-Time Supervisor Skeleton
 * Stage 0: Periodic 1 kHz AXI-Lite health check & heartbeat.
 * Stage 1: MEMS Gyro mode state machine (SWEEP -> PLL -> AGC -> QUAD_NULL -> FTR).
 * Stage 2: CPI Multi-Target Kalman / IMM Tracker.
 * Stage 3: 1 kHz Error-State EKF + LQR/PID Gimbal Controller.
 */

#include <stdint.h>
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"

#define DSP_SLOT_BASEADDR  0x80010000U
#define REG_STATUS_OFF     0x30U
#define REG_SAMPLE_CNT_OFF 0x34U
#define REG_VERSION_OFF    0x38U

int main(void) {
    uint32_t ver = Xil_In32(DSP_SLOT_BASEADDR + REG_VERSION_OFF);
    xil_printf("[R5F] Real-time supervisor started. Slot VER=0x%08X\r\n", ver);

    uint32_t tick = 0;
    while (1) {
        /* 1 kHz deterministic tick placeholder (replace with TTC hardware timer ISR in Stage 1) */
        usleep(1000);
        tick++;

        if ((tick % 1000U) == 0U) {
            uint32_t status = Xil_In32(DSP_SLOT_BASEADDR + REG_STATUS_OFF);
            uint32_t scnt   = Xil_In32(DSP_SLOT_BASEADDR + REG_SAMPLE_CNT_OFF);
            xil_printf("[R5F] t=%us status=0x%08X samples=%u\r\n",
                       tick / 1000U, status, scnt);
        }
    }
    return 0;
}
