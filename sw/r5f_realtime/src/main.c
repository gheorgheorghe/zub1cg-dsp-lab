/*
 * ZUBoard 1CG Stage 1.5 — Cortex-R5F 1 kHz Deterministic MEMS/CVG Supervisor
 * Build in Vitis 2024.2 Unified IDE (standalone on psu_cortexr5_0).
 *
 * Implements the deterministic 1 kHz state machine over AXI4-Lite (0x8001_0000):
 *   IDLE -> OPEN_LOOP_SWEEP -> PLL_ACQUIRE -> AGC_LOCK -> QUAD_NULL -> FTR_ACTIVE -> FAULT
 *
 * Also degrades gracefully to Stage 0 heartbeat mode if a Stage 0 bitstream is loaded.
 */

#include <stdint.h>
#include <stdlib.h>
#include "xil_io.h"
#include "xil_printf.h"
#include "xiltimer.h"

/* Must match hw/scripts/create_bd.tcl */
#define DSP_SLOT_BASEADDR   0x80010000U

/* Stage 0 RO registers (fallback) */
#define S0_REG_STATUS       0x30U
#define S0_REG_SAMPLE_CNT   0x34U
#define S0_REG_VERSION      0x38U

/* Stage 1 AXI4-Lite register map (matching rtl/mems_hil/mems_hil_axis_wrap.sv) */
#define S1_REG_CTRL         0x00U  /* [0]=run, [1]=pll, [2]=agc, [3]=clq, [4]=ftr, [5]=soft_rst */
#define S1_REG_BASE_FTW     0x0CU
#define S1_REG_DX_REF       0x10U
#define S1_REG_DX_RO        0xC0U  /* Q2.22 signed */
#define S1_REG_DY_RO        0xC4U  /* Q2.22 signed */
#define S1_REG_SX_RO        0xC8U  /* Q2.22 signed */
#define S1_REG_SY_RO        0xCCU  /* Q2.22 signed */
#define S1_REG_PLL_ADJ_RO   0xD0U  /* 24-bit signed, LSB = 16/2^32 * 1e6 Hz */
#define S1_REG_AGC_ADJ_RO   0xD4U  /* Q10.14 signed */
#define S1_REG_CLQ_ADJ_RO   0xD8U  /* Q10.14 signed */
#define S1_REG_FTR_ADJ_RO   0xDCU  /* Q10.14 signed */
#define S1_REG_STATUS       0xE4U  /* [0]=ready, [1]=step_done, [2]=run, [3]=ovf, [31:16]=stage */
#define S1_REG_FRAME_CNT    0xE8U
#define S1_REG_VERSION      0xECU  /* 0x5A55_0101 */

/* CTRL bit masks */
#define CTRL_RUN            (1U << 0)
#define CTRL_PLL            (1U << 1)
#define CTRL_AGC            (1U << 2)
#define CTRL_CLQ            (1U << 3)
#define CTRL_FTR            (1U << 4)
#define CTRL_SOFT_RST       (1U << 5)

/* Fixed-point thresholds in Q2.22 (1.0 = 4,194,304) */
#define Q22_ONE             4194304
#define TH_DY_LOCK          83886    /* |DY| < 0.020 -> PLL locked */
#define TH_DX_LOCK          83886    /* |DX - DX_ref| < 0.020 -> AGC locked */
#define TH_SY_LOCK          41943    /* |SY| < 0.010 -> CLQ nulled */
#define TH_DX_COLLAPSE      838860   /* DX < 0.20 -> amplitude collapse fault */
#define TH_DY_UNLOCK        1468006  /* |DY| > 0.35 -> PLL unlock fault */

/* Timing constants (1 kHz deterministic loop) */
#define LOOP_HZ             1000U
#define LOCK_QUAL_TICKS     20U      /* 20 ms consecutive in-band to qualify lock */
#define SWEEP_STEPS         41U      /* +-200 Hz in 10 Hz steps @ 1 MHz HIL tick */
#define SWEEP_DWELL_TICKS   5U       /* 5 ms per frequency bin */
#define SWEEP_STEP_FTW      42950U   /* 10 Hz / 1 MHz * 2^32 */
#define FAULT_COOLDOWN_MS   500U

/* Fault bitmask */
#define FLT_AMP_COLLAPSE    (1U << 0)
#define FLT_PLL_UNLOCK      (1U << 1)
#define FLT_STREAM_OVF      (1U << 2)
#define FLT_STATE_TIMEOUT   (1U << 3)

typedef enum {
    ST_IDLE = 0,
    ST_OPEN_LOOP_SWEEP,
    ST_PLL_ACQUIRE,
    ST_AGC_LOCK,
    ST_QUAD_NULL,
    ST_FTR_ACTIVE,
    ST_FAULT
} gyro_state_t;

static const char *state_name(gyro_state_t s) {
    switch (s) {
        case ST_IDLE:            return "IDLE";
        case ST_OPEN_LOOP_SWEEP: return "OPEN_LOOP_SWEEP";
        case ST_PLL_ACQUIRE:     return "PLL_ACQUIRE";
        case ST_AGC_LOCK:        return "AGC_LOCK";
        case ST_QUAD_NULL:       return "QUAD_NULL";
        case ST_FTR_ACTIVE:      return "FTR_ACTIVE";
        case ST_FAULT:           return "FAULT";
        default:                 return "UNKNOWN";
    }
}

static inline uint32_t reg_rd(uint32_t off) {
    return Xil_In32(DSP_SLOT_BASEADDR + off);
}

static inline void reg_wr(uint32_t off, uint32_t val) {
    Xil_Out32(DSP_SLOT_BASEADDR + off, val);
}

static inline int32_t reg_rd_s32(uint32_t off) {
    return (int32_t)Xil_In32(DSP_SLOT_BASEADDR + off);
}

static inline uint32_t iabs32(int32_t v) {
    return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

static void run_stage0_heartbeat(void) {
    const XTime tick_counts = (XTime)(COUNTS_PER_SECOND / LOOP_HZ);
    XTime next_deadline;
    XTime_GetTime(&next_deadline);

    uint32_t tick = 0;
    while (1) {
        next_deadline += tick_counts;
        XTime now;
        do {
            XTime_GetTime(&now);
        } while (now < next_deadline);

        tick++;
        if ((tick % 1000U) == 0U) {
            uint32_t status = reg_rd(S0_REG_STATUS);
            uint32_t scnt   = reg_rd(S0_REG_SAMPLE_CNT);
            xil_printf("[R5F-S0] t=%us status=0x%08X samples=%u\r\n",
                       tick / 1000U, status, scnt);
        }
    }
}

int main(void) {
    uint32_t ver_s1 = reg_rd(S1_REG_VERSION);
    if (((ver_s1 >> 16) & 0xFFFFU) != 0x5A55U) {
        uint32_t ver_s0 = reg_rd(S0_REG_VERSION);
        xil_printf("[R5F] Stage 0 slot detected (VER=0x%08X). Running heartbeat.\r\n", ver_s0);
        run_stage0_heartbeat();
        return 0;
    }

    xil_printf("\r\n[R5F] Stage 1.5 CVG 1 kHz Supervisor started (VER=0x%08X)\r\n", ver_s1);

    const XTime tick_counts = (XTime)(COUNTS_PER_SECOND / LOOP_HZ);
    XTime next_deadline;
    XTime_GetTime(&next_deadline);

    gyro_state_t state = ST_IDLE;
    gyro_state_t prev_state = ST_FAULT;
    uint32_t tick_ms = 0U;
    uint32_t state_ticks = 0U;
    uint32_t qual_cnt = 0U;
    uint32_t fault_mask = 0U;

    /* Open-loop frequency sweep state */
    uint32_t sweep_nom_ftw = 0U;
    uint32_t sweep_best_ftw = 0U;
    uint32_t sweep_best_amp = 0U;
    uint32_t sweep_idx = 0U;
    uint32_t sweep_dwell = 0U;

    uint32_t max_exec_us = 0U;

    while (1) {
        /* Deterministic 1 kHz deadline pacing (zero cumulative drift) */
        next_deadline += tick_counts;
        XTime t_start, t_end;
        do {
            XTime_GetTime(&t_start);
        } while (t_start < next_deadline);

        tick_ms++;
        state_ticks++;

        uint32_t ctrl   = reg_rd(S1_REG_CTRL);
        uint32_t status = reg_rd(S1_REG_STATUS);
        int32_t  dx     = reg_rd_s32(S1_REG_DX_RO);
        int32_t  dy     = reg_rd_s32(S1_REG_DY_RO);
        int32_t  sx     = reg_rd_s32(S1_REG_SX_RO);
        int32_t  sy     = reg_rd_s32(S1_REG_SY_RO);
        int32_t  dx_ref = reg_rd_s32(S1_REG_DX_REF);
        if (dx_ref <= 0) dx_ref = Q22_ONE;

        if (state != prev_state) {
            xil_printf("[R5F] t=%ums STATE -> %s (CTRL=0x%02X STATUS=0x%08X)\r\n",
                       tick_ms, state_name(state), ctrl & 0xFFU, status);
            prev_state = state;
            state_ticks = 0U;
            qual_cnt = 0U;
        }

        switch (state) {
            case ST_IDLE:
                /* Wait for host/preset to assert RUN, then either join active loop or sweep */
                if ((ctrl & CTRL_RUN) && !(ctrl & CTRL_SOFT_RST)) {
                    if ((ctrl & (CTRL_PLL | CTRL_AGC)) == (CTRL_PLL | CTRL_AGC)) {
                        /* Host already armed PLL+AGC (e.g., step/quad/adev test): monitor without overriding */
                        state = (ctrl & CTRL_FTR) ? ST_FTR_ACTIVE : ST_PLL_ACQUIRE;
                    } else {
                        sweep_nom_ftw  = reg_rd(S1_REG_BASE_FTW);
                        sweep_best_ftw = sweep_nom_ftw;
                        sweep_best_amp = 0U;
                        sweep_idx      = 0U;
                        sweep_dwell    = 0U;
                        reg_wr(S1_REG_CTRL, (ctrl & ~(CTRL_PLL | CTRL_CLQ | CTRL_FTR)) | CTRL_RUN | CTRL_AGC);
                        state = ST_OPEN_LOOP_SWEEP;
                    }
                }
                break;

            case ST_OPEN_LOOP_SWEEP: {
                if (sweep_dwell == 0U) {
                    int32_t step_off = (int32_t)sweep_idx - (int32_t)(SWEEP_STEPS / 2U);
                    uint32_t ftw = (uint32_t)((int32_t)sweep_nom_ftw + step_off * (int32_t)SWEEP_STEP_FTW);
                    reg_wr(S1_REG_BASE_FTW, ftw);
                }
                sweep_dwell++;
                if (sweep_dwell >= SWEEP_DWELL_TICKS) {
                    /* L1 amplitude proxy |DX| + |DY| (fast integer peak detector) */
                    uint32_t amp = iabs32(dx) + iabs32(dy);
                    if (amp > sweep_best_amp) {
                        sweep_best_amp = amp;
                        sweep_best_ftw = reg_rd(S1_REG_BASE_FTW);
                    }
                    sweep_dwell = 0U;
                    sweep_idx++;
                    if (sweep_idx >= SWEEP_STEPS) {
                        reg_wr(S1_REG_BASE_FTW, sweep_best_ftw);
                        reg_wr(S1_REG_CTRL, (ctrl & ~(CTRL_CLQ | CTRL_FTR)) | CTRL_RUN | CTRL_PLL | CTRL_AGC);
                        state = ST_PLL_ACQUIRE;
                    }
                }
                break;
            }

            case ST_PLL_ACQUIRE:
                if (iabs32(dy) < TH_DY_LOCK) {
                    qual_cnt++;
                    if (qual_cnt >= LOCK_QUAL_TICKS) {
                        state = ST_AGC_LOCK;
                    }
                } else {
                    qual_cnt = 0U;
                }
                if (state_ticks > 2000U) {
                    fault_mask |= FLT_STATE_TIMEOUT;
                    state = ST_FAULT;
                }
                break;

            case ST_AGC_LOCK:
                if (iabs32(dx - dx_ref) < TH_DX_LOCK) {
                    qual_cnt++;
                    if (qual_cnt >= LOCK_QUAL_TICKS) {
                        reg_wr(S1_REG_CTRL, ctrl | CTRL_RUN | CTRL_PLL | CTRL_AGC | CTRL_CLQ);
                        state = ST_QUAD_NULL;
                    }
                } else {
                    qual_cnt = 0U;
                }
                if (state_ticks > 2000U) {
                    fault_mask |= FLT_STATE_TIMEOUT;
                    state = ST_FAULT;
                }
                break;

            case ST_QUAD_NULL:
                if (iabs32(sy) < TH_SY_LOCK) {
                    qual_cnt++;
                    if (qual_cnt >= LOCK_QUAL_TICKS) {
                        reg_wr(S1_REG_CTRL, ctrl | CTRL_RUN | CTRL_PLL | CTRL_AGC | CTRL_CLQ | CTRL_FTR);
                        state = ST_FTR_ACTIVE;
                    }
                } else {
                    qual_cnt = 0U;
                }
                if (state_ticks > 2000U) {
                    fault_mask |= FLT_STATE_TIMEOUT;
                    state = ST_FAULT;
                }
                break;

            case ST_FTR_ACTIVE:
                /* Continuous real-time fault supervision */
                if (!(ctrl & CTRL_RUN) || (ctrl & CTRL_SOFT_RST)) {
                    state = ST_IDLE;
                    break;
                }
                if (dx < TH_DX_COLLAPSE) {
                    fault_mask |= FLT_AMP_COLLAPSE;
                }
                if (iabs32(dy) > TH_DY_UNLOCK) {
                    fault_mask |= FLT_PLL_UNLOCK;
                }
                if (fault_mask != 0U) {
                    state = ST_FAULT;
                }
                break;

            case ST_FAULT:
                if (state_ticks == 1U) {
                    xil_printf("[R5F] FAULT mask=0x%02X (DX=%d DY=%d SX=%d SY=%d)\r\n",
                               fault_mask, dx, dy, sx, sy);
                    /* Drop FTR/CLQ safely while keeping drive alive for recovery */
                    reg_wr(S1_REG_CTRL, ctrl & ~(CTRL_CLQ | CTRL_FTR));
                }
                if (state_ticks >= FAULT_COOLDOWN_MS) {
                    fault_mask = 0U;
                    state = ST_IDLE;
                }
                break;
        }

        XTime_GetTime(&t_end);
        uint32_t exec_us = (uint32_t)(((t_end - t_start) * 1000000ULL) / COUNTS_PER_SECOND);
        if (exec_us > max_exec_us) {
            max_exec_us = exec_us;
        }

        /* 1 Hz health & telemetry summary on UART */
        if ((tick_ms % 1000U) == 0U) {
            int32_t pll_adj = reg_rd_s32(S1_REG_PLL_ADJ_RO);
            int32_t ftr_adj = reg_rd_s32(S1_REG_FTR_ADJ_RO);
            uint32_t fcnt   = reg_rd(S1_REG_FRAME_CNT);
            xil_printf("[R5F] t=%us %s | DX=%d DY=%d SX=%d SY=%d | PLL=%d FTR=%d | frames=%u | max_us=%u\r\n",
                       tick_ms / 1000U, state_name(state),
                       dx, dy, sx, sy, pll_adj, ftr_adj, fcnt, max_exec_us);
            max_exec_us = 0U;
        }
    }

    return 0;
}
