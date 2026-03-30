/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CCID v13.7 — Continuous Early Check + Signed DC + Quad AC
  * @version        : 13.1.7 (the Most accurate post 6mA DC)
  ******************************************************************************
  *
  * Based on v13.6 (Signed DC + Quadratic AC + Dual-Stage).
  *
  * KEY CHANGE in v13.1.7:
  *   Stage 1 now checks EVERY SAMPLE from 20ms onward (not once at 40ms).
  *   The moment the running accumulator crosses the threshold, PB1 fires.
  *   Typical PB1 response: 20-40ms for strong signals.
  *
  *   The check is extremely lightweight per sample:
  *     AC: one divide + one subtract + one compare (all on squared values)
  *     DC: one divide + one fabsf + one compare
  *   No sqrt ever in ISR. Total ISR overhead: ~200ns per sample.
  *
  *   Stage 2 (precision at ~82ms) is unchanged from v13.6:
  *     DC: signed average, direction-aware
  *     AC: RMS + cycle trimming + quadratic offset
  *     PB1 corrected based on precise value.
  *
  *   Timeline:
  *     0ms        → Window opens
  *     0-20ms     → Samples stored + accumulated (no checks yet)
  *     20ms+      → EVERY sample: quick threshold check
  *     20-40ms    → PB1 fires as soon as threshold crossed (typical)
  *     80ms       → Window closes
  *     ~82ms      → Stage 2: precise current + PB1 correction
  *     ~82-100ms  → UART print
  *     ~100ms     → Next window
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---- Peripheral handles ---- */
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* ---- Constants ---- */
#define SAMPLE_COUNT        100
#define TICK_NS             15.625f
#define CAPTURE_WINDOW_MS   80
#define EARLY_START_MS      20         /* Start checking from 20ms */
#define EARLY_MIN_SAMPLES   30         /* Need at least 30 samples */
#define MAX_LIVE_SAMPLES    350
#define AC_DC_THRESHOLD     0.50f
#define EARLY_MARGIN        1.00f

/* ---- Alert thresholds (mA) ---- */
#define DC_ALERT_MA         6.0f
#define AC_ALERT_MA         30.0f

/* ============================================================
 *  PHASE 1 — Input Capture State Machine
 * ============================================================ */
volatile uint16_t cap_a1 = 0, cap_b = 0, cap_a2 = 0;
volatile uint8_t  cap_state = 0;

/* ============================================================
 *  PHASE 2 — Offset & Measured Gain
 * ============================================================ */
volatile float    del_t_sum_ns = 0.0f;
volatile float    del_t_sum_sq_ns = 0.0f;
volatile uint16_t sample_count = 0;

volatile float    offset_avg_ns = 0.0f;
volatile float    offset_rms_ns = 0.0f;
volatile float    mg_ns = 0.0f;

/* ============================================================
 *  PHASE 3 — Cg, Sensitivity
 * ============================================================ */
volatile float    Cg_ns = 0.0f;
volatile float    sensitivity_ns = 0.0f;

/* ============================================================
 *  PHASE 4-7 — Live Capture Window
 * ============================================================ */
volatile float    live_samples_ns[MAX_LIVE_SAMPLES];
volatile uint16_t live_index = 0;
volatile uint8_t  live_capturing = 0;
volatile uint8_t  live_window_done = 0;
volatile uint32_t live_start_tick = 0;

/* ============================================================
 *  Early Alert — ISR-side running accumulators
 * ============================================================ */
volatile float    early_sum_sq_ns = 0.0f;
volatile float    early_sum_ns = 0.0f;
volatile uint16_t early_count = 0;
volatile uint8_t  early_has_pos = 0;
volatile uint8_t  early_has_neg = 0;
volatile uint8_t  early_alert_fired = 0;    /* 1 = PB1 already set by early check */
volatile uint32_t early_fire_tick = 0;      /* timestamp when early alert fired */

/* ---- Pre-computed thresholds ---- */
volatile float    dc_early_current_thresh_ns = 0.0f;   /* |avg-offset| must exceed this */
volatile float    ac_early_sq_thresh_ns = 0.0f;         /* rms²-offset² must exceed this */
volatile float    offset_rms_sq = 0.0f;                  /* Pre-computed offset_rms² for ISR speed */

/* ============================================================
 *  State Machine & Flags
 * ============================================================ */
typedef enum {
    STATE_IDLE,
    STATE_OFFSET_SAMPLING,
    STATE_OFFSET_DONE,
    STATE_MG_SAMPLING,
    STATE_MG_DONE,
    STATE_LIVE_CAPTURE
} SystemState;

volatile SystemState sys_state = STATE_IDLE;
volatile uint8_t     data_ready = 0;
volatile uint8_t     result_type = 0;

/* ---- UART command buffer ---- */
char cmd_buf[16];
uint8_t cmd_idx = 0;

/* ---- Function prototypes ---- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

static void uart_print(const char *str);
static void start_offset_measurement(void);
static void start_mg_measurement(void);
static void start_live_capture(void);
static void start_capture_window(void);
static void compute_early_thresholds(void);
static void process_command(const char *cmd);
static void print_offset_result(void);
static void print_mg_result(void);
static void print_calibration_summary(void);
static void process_capture_window(void);

/* ============================================================
 *  UART Helper
 * ============================================================ */
static void uart_print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/* ============================================================
 *  Compute Early Alert Thresholds
 * ============================================================ */
static void compute_early_thresholds(void)
{
    /* DC: |quick_avg - offset_avg| > this value */
    dc_early_current_thresh_ns = DC_ALERT_MA * EARLY_MARGIN * fabsf(sensitivity_ns);

    /* AC: (quick_rms² - offset_rms²) > this value (squared, no sqrt in ISR) */
    float ac_signal_ns = AC_ALERT_MA * EARLY_MARGIN * fabsf(sensitivity_ns);
    ac_early_sq_thresh_ns = ac_signal_ns * ac_signal_ns;

    /* Pre-compute offset_rms² so ISR doesn't recompute it every sample */
    offset_rms_sq = offset_rms_ns * offset_rms_ns;
}

/* ============================================================
 *  PHASE 2 — Start Offset Measurement
 * ============================================================ */
static void start_offset_measurement(void)
{
    sample_count = 0;
    del_t_sum_ns = 0.0f;
    del_t_sum_sq_ns = 0.0f;
    cap_state = 0;
    data_ready = 0;
    sys_state = STATE_OFFSET_SAMPLING;
}

/* ============================================================
 *  PHASE 2 — Start MG Measurement
 * ============================================================ */
static void start_mg_measurement(void)
{
    sample_count = 0;
    del_t_sum_ns = 0.0f;
    del_t_sum_sq_ns = 0.0f;
    cap_state = 0;
    data_ready = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    sys_state = STATE_MG_SAMPLING;
}

/* ============================================================
 *  PHASE 4 — Start Live Capture Mode
 * ============================================================ */
static void start_live_capture(void)
{
    cap_state = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    sys_state = STATE_LIVE_CAPTURE;
    start_capture_window();
}

/* ============================================================
 *  Start a single 80ms capture window
 * ============================================================ */
static void start_capture_window(void)
{
    live_index = 0;
    live_capturing = 1;
    live_window_done = 0;
    cap_state = 0;

    early_sum_sq_ns = 0.0f;
    early_sum_ns = 0.0f;
    early_count = 0;
    early_has_pos = 0;
    early_has_neg = 0;
    early_alert_fired = 0;
    early_fire_tick = 0;

    live_start_tick = HAL_GetTick();
}

/* ============================================================
 *  Command Processing
 * ============================================================ */
static void process_command(const char *cmd)
{
    if (strcmp(cmd, "0x001") == 0)
    {
        uart_print("PHASE 2: Measuring Offset (100 samples)...\r\n");
        uart_print("  Computing BOTH average and RMS offset.\r\n");
        start_offset_measurement();
    }
    else if (strcmp(cmd, "0x002") == 0)
    {
        if (sys_state == STATE_IDLE && offset_avg_ns == 0.0f && offset_rms_ns == 0.0f)
        {
            uart_print("ERROR: Take offset first (0x001)\r\n");
            return;
        }
        uart_print("PHASE 2: PA12 HIGH - Measuring Gain (100 samples)...\r\n");
        start_mg_measurement();
    }
    else if (strcmp(cmd, "0x003") == 0)
    {
        if (sensitivity_ns == 0.0f)
        {
            uart_print("ERROR: Complete calibration first (0x001 then 0x002)\r\n");
            return;
        }
        uart_print("PHASE 4: Starting live capture (80ms windows)...\r\n");
        uart_print("  DC: signed avg | AC: quadratic RMS + trimming\r\n");
        uart_print("  Continuous early check from 20ms onward\r\n");
        uart_print("  Send 0x004 to stop.\r\n\r\n");
        start_live_capture();
    }
    else if (strcmp(cmd, "0x004") == 0)
    {
        sys_state = STATE_IDLE;
        cap_state = 0;
        live_capturing = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        uart_print("\r\nLive capture STOPPED. PB1 OFF.\r\n\r\n");
    }
    else
    {
        uart_print("Commands: 0x001 (offset), 0x002 (gain), 0x003 (live), 0x004 (stop)\r\n");
    }
}

/* ============================================================
 *  Print Functions
 * ============================================================ */
static void print_offset_result(void)
{
    char buf[128];
    uart_print("\r\n========== OFFSET (Phase 2) ==========\r\n");
    sprintf(buf, "  Samples:      %d\r\n", SAMPLE_COUNT);
    uart_print(buf);
    sprintf(buf, "  Offset(avg):  %.2f ns   [for DC]\r\n", offset_avg_ns);
    uart_print(buf);
    sprintf(buf, "  Offset(rms):  %.2f ns   [for AC]\r\n", offset_rms_ns);
    uart_print(buf);
    uart_print("  Now send 0x002 for Gain measurement.\r\n");
    uart_print("=======================================\r\n\r\n");
}

static void print_mg_result(void)
{
    char buf[128];
    uart_print("\r\n========== MEASURED GAIN (Phase 2) ==========\r\n");
    sprintf(buf, "  Samples: %d   MG: %.2f ns\r\n", SAMPLE_COUNT, mg_ns);
    uart_print(buf);
    uart_print("===============================================\r\n\r\n");
}

static void print_calibration_summary(void)
{
    char buf[128];
    uart_print("========== CALIBRATION COMPLETE (Phase 3) ==========\r\n");
    sprintf(buf, "  Offset(avg): %.2f ns  Offset(rms): %.2f ns\r\n", offset_avg_ns, offset_rms_ns);
    uart_print(buf);
    sprintf(buf, "  MG: %.2f ns  Cg: %.2f ns  Sens: %.6f ns/mA\r\n", mg_ns, Cg_ns, sensitivity_ns);
    uart_print(buf);
    sprintf(buf, "  DC early thresh: %.2f ns (%.1f mA)\r\n",
            dc_early_current_thresh_ns, DC_ALERT_MA * EARLY_MARGIN);
    uart_print(buf);
    sprintf(buf, "  AC early thresh: %.2f ns^2 (%.1f mA)\r\n",
            ac_early_sq_thresh_ns, AC_ALERT_MA * EARLY_MARGIN);
    uart_print(buf);
    uart_print("  Send 0x003 to start live capture.\r\n");
    uart_print("====================================================\r\n\r\n");
}

/* ============================================================
 *  PHASE 4-7: Process one completed 80ms capture window
 *
 *  Stage 2 — PRECISION (identical to v13.6):
 *    DC: signed average → direction-aware current
 *    AC: cycle trimming → RMS → quadratic offset
 *    PB1 CORRECTED based on precise value
 *    UART output with early alert timing info
 * ============================================================ */
static void process_capture_window(void)
{
    char buf[256];
    uint16_t n = live_index;

    if (n < 2)
    {
        uart_print("  Not enough samples.\r\n");
        return;
    }

    /* ================================================================
     *  PASS 1: Scan all samples
     * ================================================================ */
    float sum_dt_ns = 0.0f;
    float sum_sq_ns = 0.0f;
    float max_dt_ns = live_samples_ns[0];
    float min_dt_ns = live_samples_ns[0];
    uint8_t has_positive = 0;
    uint8_t has_negative = 0;

    for (uint16_t i = 0; i < n; i++)
    {
        float dt_ns = live_samples_ns[i];
        sum_dt_ns += dt_ns;
        sum_sq_ns += dt_ns * dt_ns;
        if (dt_ns > max_dt_ns) max_dt_ns = dt_ns;
        if (dt_ns < min_dt_ns) min_dt_ns = dt_ns;
        if (dt_ns > 0.0f) has_positive = 1;
        if (dt_ns < 0.0f) has_negative = 1;
    }

    float avg_cycle_ns = sum_dt_ns / (float)n;

    /* ---- AC/DC Classification ---- */
    uint8_t is_ac = 0;

    if (has_positive && has_negative)
    {
        is_ac = 1;
    }

    if (!is_ac)
    {
        float avg_abs = fabsf(avg_cycle_ns);
        if (avg_abs > 0.0f)
        {
            float variation = (max_dt_ns - min_dt_ns) / avg_abs;
            if (variation > AC_DC_THRESHOLD)
            {
                is_ac = 1;
            }
        }
    }

    /* ================================================================
     *  Current Calculation
     * ================================================================ */
    float current_mA = 0.0f;
    const char *sig_label;
    float offset_used_ns;
    uint16_t trimmed_cycles = 0;
    uint16_t rms_n = n;
    float del_t_rms_ns = 0.0f;

    if (is_ac)
    {
        sig_label = "AC/quad";
        offset_used_ns = offset_rms_ns;

        /* ---- Cycle trimming ---- */
        uint16_t rms_start = 0;
        uint16_t rms_end = n;

        uint16_t first_crossing = 0;
        uint16_t last_crossing = 0;
        uint8_t  found_first = 0;
        uint16_t crossing_count = 0;

        for (uint16_t i = 1; i < n; i++)
        {
            if (live_samples_ns[i - 1] <= 0.0f && live_samples_ns[i] > 0.0f)
            {
                crossing_count++;
                if (!found_first)
                {
                    first_crossing = i;
                    found_first = 1;
                }
                last_crossing = i;
            }
        }

        if (crossing_count >= 2)
        {
            rms_start = first_crossing;
            rms_end = last_crossing;
            trimmed_cycles = crossing_count - 1;
        }

        rms_n = rms_end - rms_start;
        if (rms_n < 2) { rms_start = 0; rms_end = n; rms_n = n; }

        float trim_sum_sq = 0.0f;
        for (uint16_t i = rms_start; i < rms_end; i++)
        {
            float dt_ns = live_samples_ns[i];
            trim_sum_sq += dt_ns * dt_ns;
        }

        del_t_rms_ns = sqrtf(trim_sum_sq / (float)rms_n);

        /* Quadratic AC offset */
        if (sensitivity_ns != 0.0f)
        {
            float rms_sq = del_t_rms_ns * del_t_rms_ns;
            float off_sq = offset_rms_ns * offset_rms_ns;

            if (rms_sq > off_sq)
                current_mA = sqrtf(rms_sq - off_sq) / sensitivity_ns;
            else
                current_mA = 0.0f;
        }
    }
    else
    {
        /* DC: signed average */
        sig_label = "DC/lin";
        offset_used_ns = offset_avg_ns;

        if (sensitivity_ns != 0.0f)
        {
            current_mA = (avg_cycle_ns - offset_avg_ns) / sensitivity_ns;
        }
    }

    float abs_current = fabsf(current_mA);

    /* ================================================================
     *  PB1 CORRECTION — Stage 2 overrides Stage 1
     * ================================================================ */
    uint8_t pb1_state = 0;

    if (is_ac)
    {
        if (abs_current >= AC_ALERT_MA)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
            pb1_state = 2;
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            pb1_state = 0;
        }
    }
    else
    {
        if (abs_current >= DC_ALERT_MA)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            pb1_state = 1;
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            pb1_state = 0;
        }
    }

    /* ================================================================
     *  UART Output
     * ================================================================ */
    uart_print("--------------------------------------------\r\n");

    if (is_ac && trimmed_cycles > 0)
    {
        sprintf(buf, "  Cap:%d  Trim:%d (%dcyc)  %s\r\n",
                n, rms_n, trimmed_cycles, sig_label);
    }
    else
    {
        sprintf(buf, "  Cap:%d  %s\r\n", n, sig_label);
    }
    uart_print(buf);

    sprintf(buf, "  Max:%.2f  Min:%.2f  Avg:%.2f ns\r\n",
            max_dt_ns, min_dt_ns, avg_cycle_ns);
    uart_print(buf);

    if (is_ac)
    {
        sprintf(buf, "  delTrms:%.2f  Offset:%.2f ns\r\n",
                del_t_rms_ns, offset_used_ns);
        uart_print(buf);
    }
    else
    {
        sprintf(buf, "  Offset:%.2f ns  Sens:%.4f ns/mA\r\n",
                offset_used_ns, sensitivity_ns);
        uart_print(buf);
    }

    /* Current line with early alert timing */
    if (is_ac)
    {
        sprintf(buf, "  I:%.4f mA", current_mA);
    }
    else
    {
        sprintf(buf, "  I:%.4f mA  |I|:%.4f mA", current_mA, abs_current);
    }
    uart_print(buf);

    if (early_alert_fired)
    {
        uint32_t fire_ms = early_fire_tick - live_start_tick;
        sprintf(buf, "  Early:%lums", (unsigned long)fire_ms);
    }
    else
    {
        sprintf(buf, "  Early:no");
    }
    uart_print(buf);

    if (pb1_state == 1)
        uart_print("  PB1:HIGH\r\n");
    else if (pb1_state == 2)
        uart_print("  PB1:BLINK\r\n");
    else
        uart_print("  PB1:LOW\r\n");

    uart_print("--------------------------------------------\r\n");
}

/* ============================================================
 *  Timer Input Capture ISR
 *
 *  v13.7 KEY CHANGE:
 *    Stage 1 checks EVERY SAMPLE from EARLY_START_MS (20ms) onward.
 *    As soon as running accumulator crosses threshold → PB1 fires.
 *    No single checkpoint — continuous monitoring.
 *
 *    Per-sample cost:
 *      AC: one divide + one subtract + one compare (~150ns)
 *      DC: one divide + one fabsf + one compare (~150ns)
 *      No sqrt ever. offset_rms² pre-computed.
 * ============================================================ */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (sys_state == STATE_IDLE) return;
    if (htim->Instance != TIM1) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint16_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (cap_state == 0)
        {
            cap_a1 = capture;
            cap_state = 1;
        }
        else if (cap_state == 2)
        {
            cap_a2 = capture;

            uint16_t t1_ticks = cap_b - cap_a1;
            uint16_t t2_ticks = cap_a2 - cap_b;

            float t1_ns = (float)t1_ticks * TICK_NS;
            float t2_ns = (float)t2_ticks * TICK_NS;

            float del_t_ns = t1_ns - t2_ns;

            /* ---- Offset sampling ---- */
            if (sys_state == STATE_OFFSET_SAMPLING)
            {
                del_t_sum_ns += del_t_ns;
                del_t_sum_sq_ns += del_t_ns * del_t_ns;
                sample_count++;

                if (sample_count >= SAMPLE_COUNT)
                {
                    offset_avg_ns = del_t_sum_ns / (float)SAMPLE_COUNT;
                    offset_rms_ns = sqrtf(del_t_sum_sq_ns / (float)SAMPLE_COUNT);

                    sys_state = STATE_OFFSET_DONE;
                    result_type = 0;
                    data_ready = 1;
                }
            }
            /* ---- MG sampling ---- */
            else if (sys_state == STATE_MG_SAMPLING)
            {
                del_t_sum_ns += del_t_ns;
                sample_count++;

                if (sample_count >= SAMPLE_COUNT)
                {
                    mg_ns = del_t_sum_ns / (float)SAMPLE_COUNT;

                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

                    Cg_ns = mg_ns - offset_avg_ns;

                    if (Cg_ns != 0.0f)
                        sensitivity_ns = Cg_ns / 24.0f;
                    else
                        sensitivity_ns = 0.0f;

                    compute_early_thresholds();

                    sys_state = STATE_MG_DONE;
                    result_type = 1;
                    data_ready = 1;
                }
            }
            /* ---- Live capture with CONTINUOUS early alert ---- */
            else if (sys_state == STATE_LIVE_CAPTURE && live_capturing)
            {
                uint32_t elapsed = HAL_GetTick() - live_start_tick;

                if (elapsed < CAPTURE_WINDOW_MS)
                {
                    /* Store sample in buffer */
                    if (live_index < MAX_LIVE_SAMPLES)
                    {
                        live_samples_ns[live_index++] = del_t_ns;
                    }

                    /* Accumulate for early check */
                    early_sum_sq_ns += del_t_ns * del_t_ns;
                    early_sum_ns += del_t_ns;
                    early_count++;
                    if (del_t_ns > 0.0f) early_has_pos = 1;
                    if (del_t_ns < 0.0f) early_has_neg = 1;

                    /* ============================================
                     *  ★ CONTINUOUS EARLY CHECK ★
                     *
                     *  Checks EVERY sample from 20ms onward
                     *  (with minimum 30 samples for stability).
                     *
                     *  Once fired, stops checking (early_alert_fired=1).
                     *  Stage 2 will correct if it was a false alarm.
                     *
                     *  Per-sample cost: ~150ns (no sqrt ever)
                     * ============================================ */
                    if (!early_alert_fired &&
                        elapsed >= EARLY_START_MS &&
                        early_count >= EARLY_MIN_SAMPLES)
                    {
                        /* Quick AC/DC: zero crossing only */
                        uint8_t quick_ac = (early_has_pos && early_has_neg) ? 1 : 0;

                        float inv_count = 1.0f / (float)early_count;

                        if (quick_ac)
                        {
                            /*
                             * AC CHECK (no sqrt):
                             *   quick_rms² = sum_sq / count
                             *   signal²    = quick_rms² - offset_rms²
                             *   fire if signal² > ac_early_sq_thresh
                             */
                            float quick_rms_sq = early_sum_sq_ns * inv_count;
                            float signal_sq = quick_rms_sq - offset_rms_sq;

                            if (signal_sq > ac_early_sq_thresh_ns)
                            {
                                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
                                early_alert_fired = 1;
                                early_fire_tick = HAL_GetTick();
                            }
                        }
                        else
                        {
                            /*
                             * DC CHECK (signed average, no sqrt):
                             *   quick_avg = sum / count
                             *   signal    = |quick_avg - offset_avg|
                             *   fire if signal > dc_early_current_thresh
                             */
                            float quick_avg = early_sum_ns * inv_count;
                            float dc_signal_ns = fabsf(quick_avg - offset_avg_ns);

                            if (dc_signal_ns > dc_early_current_thresh_ns)
                            {
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                                early_alert_fired = 1;
                                early_fire_tick = HAL_GetTick();
                            }
                        }
                    }
                }
                else
                {
                    /* 80ms elapsed — window complete */
                    live_capturing = 0;
                    live_window_done = 1;
                }
            }

            cap_state = 0;
        }
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (cap_state == 1)
        {
            cap_b = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            cap_state = 2;
        }
    }
}

/* ============================================================
 *  Main
 * ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

    uart_print("\r\n");
    uart_print("================================================\r\n");
    uart_print("  CCID v13.7 — Continuous Early + Signed DC\r\n");
    uart_print("  ALL values in NANOSECONDS\r\n");
    uart_print("================================================\r\n");
    uart_print("Commands:\r\n");
    uart_print("  0x001 - Measure Offset (avg + rms)\r\n");
    uart_print("  0x002 - PA12 HIGH + Gain + Cg\r\n");
    uart_print("  0x003 - Start Live Capture\r\n");
    uart_print("  0x004 - Stop Live Capture\r\n");
    uart_print("================================================\r\n");
    uart_print("  Window: 80ms | Early: continuous from 20ms\r\n");
    uart_print("  DC: I = (avg - offset_avg) / sens  [signed]\r\n");
    uart_print("  AC: I = sqrt(rms^2 - off^2) / sens [trimmed]\r\n");
    uart_print("================================================\r\n");
    uart_print("  PB1: HIGH if |DC| >= 6mA\r\n");
    uart_print("  PB1: BLINK if AC >= 30mA\r\n");
    uart_print("  PB1 response: 20-40ms typical\r\n");
    uart_print("================================================\r\n\r\n");

    while (1)
    {
        if (USART2->ISR & USART_ISR_RXNE)
        {
            uint8_t ch = (uint8_t)(USART2->RDR & 0xFF);

            while (!(USART2->ISR & USART_ISR_TXE)) {}
            USART2->TDR = ch;

            if (ch == '\r' || ch == '\n')
            {
                while (!(USART2->ISR & USART_ISR_TXE)) {}
                USART2->TDR = '\r';
                while (!(USART2->ISR & USART_ISR_TXE)) {}
                USART2->TDR = '\n';

                if (cmd_idx > 0)
                {
                    cmd_buf[cmd_idx] = '\0';
                    process_command(cmd_buf);
                    cmd_idx = 0;
                }
            }
            else
            {
                if (cmd_idx < sizeof(cmd_buf) - 1)
                    cmd_buf[cmd_idx++] = ch;
                else
                    cmd_idx = 0;
            }
        }

        if (data_ready)
        {
            data_ready = 0;

            if (result_type == 0)
            {
                print_offset_result();
                sys_state = STATE_IDLE;
            }
            else if (result_type == 1)
            {
                print_mg_result();
                print_calibration_summary();
                sys_state = STATE_IDLE;
            }
        }

        if (sys_state == STATE_LIVE_CAPTURE && live_window_done)
        {
            live_window_done = 0;
            process_capture_window();
            start_capture_window();
        }
    }
}

/* ============================================================
 *  Clock & Peripheral Init
 * ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
        Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
