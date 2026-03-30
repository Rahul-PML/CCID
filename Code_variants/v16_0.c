/* USER CODE BEGIN Header */
/*The version of the code is for testing purpose and seems to be valid
Zero + 16.0 
DC Accuracy check
DC Trigger check
AC accuracy Not verified
AC trigger Not verified
MODbus Check
AC/DC Clasification check*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CCID v15.1 — Modbus RTU + Fixed AC/DC Classifier + Dual-Stage Alert
  * @version        : 16.0
  ******************************************************************************
  *
  * CHANGES from v13.1.7:
  *
  * [1] MODBUS RTU over USART2 at 9600 baud (replacing TeraTerm debug UART)
  *     Register map (per MD0630T01A spec):
  *       0x0000  DC Current    R    0.01 mA/LSB
  *       0x0001  AC Current    R    0.01 mA/LSB
  *       0x0002  DC Threshold  R/W  0.1 mA/LSB  (default 60 = 6.0 mA)
  *       0x0003  AC Threshold  R/W  0.1 mA/LSB  (default 300 = 30.0 mA)
  *       0x0100  Slave Address R/W  (default 1)
  *     Function codes supported: 0x03 (Read), 0x10 (Write Multiple)
  *     Frame timeout: 5 ms inter-frame silence (T3.5 at 9600 baud)
  *     CRC: Modbus CRC-16
  *
  * [2] AC/DC CLASSIFIER FIX (resolves 2-6 mA DC misclassification as AC)
  *     Old bug: variation = (max-min)/|avg| > 0.50 wrongly reclassified
  *              small DC signals as AC because noise spread is fixed (~78 ns)
  *              while avg shrinks at low currents.
  *     Fix:     Inside has_positive&&has_negative branch, check avg deviation
  *              from offset_avg. If |avg - offset_avg| > offset_rms_ns,
  *              the mean shift is real → classify as DC.
  *              True AC has avg ≈ offset_avg (AC averages to zero).
  *
  * [3] AUTO-CALIBRATION on boot (1 sec settle + offset + gain = ~3 sec total)
  *     No manual commands needed. Modbus polling can start immediately after.
  *
  * [4] ALARM PINS
  *     PB4      = DC alarm  — HIGH when |DC| >= dc_alert_mA
  *     PB3      = AC alarm  — HIGH when AC  >= ac_alert_mA
  *     PB1      = AC+DC alarm — HIGH when either DC or AC alarm active
  *     PA12     = CAL pulse — used only during gain calibration
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================
 *  Peripheral handles
 * ============================================================ */
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart2;

/* ============================================================
 *  Firmware constants
 * ============================================================ */
#define SAMPLE_COUNT          100
#define TICK_NS               15.625f
#define CAPTURE_WINDOW_MS     80
#define EARLY_START_MS        10          /* Start early check at 10 ms */
#define EARLY_MIN_SAMPLES     15          /* Need >= 15 samples          */
#define MAX_LIVE_SAMPLES      350

/* Alarm thresholds (mA) — used for pin control only */
#define DC_ALERT_MA           6.0f
#define AC_ALERT_MA           30.0f

/* Hysteresis: alarm clears when current drops to 90% of threshold.
 *
 * The early check (ISR) threshold is derived as: alarm_thresh / HYST_FACTOR
 * = 111% of alarm threshold. This guarantees Stage 2 ALWAYS confirms when
 * early fires — true at ANY threshold value set via Modbus registers:
 *
 *   threshold    early fires at    Stage 2 arms at    Stage 2 clears at
 *    30 mA          33.3 mA           30.0 mA            27.0 mA   ← no overlap
 *    40 mA          44.4 mA           40.0 mA            36.0 mA   ← no overlap
 *   100 mA         111.1 mA          100.0 mA            90.0 mA   ← no overlap
 *
 * Jitter is impossible at any threshold: early only fires when Stage 2
 * will also arm, so they never fight each other. */
#define ALARM_HYST_FACTOR     0.90f

/* Signal-loss watchdog: clear alarm after this many ms of no samples */
#define SIGNAL_LOSS_MS        2000UL

/* ============================================================
 *  Modbus constants
 * ============================================================ */
#define MB_FRAME_BUF          64
#define MB_INTER_FRAME_MS     5           /* T3.5 silence timeout         */
#define MB_FC_READ            0x03
#define MB_FC_WRITE_MULTI     0x10
#define MB_EX_ILLEGAL_FUNC    0x01
#define MB_EX_ILLEGAL_ADDR    0x02
#define MB_EX_ILLEGAL_DATA    0x03

/* Register addresses */
#define MB_REG_DC_CURRENT     0x0000      /* R   — 0.01 mA/LSB            */
#define MB_REG_AC_CURRENT     0x0001      /* R   — 0.01 mA/LSB            */
#define MB_REG_DC_THRESH      0x0002      /* R/W — 0.1 mA/LSB, def=60     */
#define MB_REG_AC_THRESH      0x0003      /* R/W — 0.1 mA/LSB, def=300    */
#define MB_REG_SLAVE_ADDR     0x0100      /* R/W — slave address, def=1   */

/* ============================================================
 *  Input capture state machine
 * ============================================================ */
volatile uint16_t cap_a1 = 0, cap_b = 0, cap_a2 = 0;
volatile uint8_t  cap_state = 0;

/* ============================================================
 *  Calibration accumulators
 * ============================================================ */
volatile float    del_t_sum_ns    = 0.0f;
volatile float    del_t_sum_sq_ns = 0.0f;
volatile uint16_t sample_count    = 0;

volatile float    offset_avg_ns   = 0.0f;
volatile float    offset_rms_ns   = 0.0f;
volatile float    mg_ns           = 0.0f;
volatile float    Cg_ns           = 0.0f;
volatile float    sensitivity_ns  = 0.0f;

/* ============================================================
 *  Live capture window
 * ============================================================ */
volatile float    live_samples_ns[MAX_LIVE_SAMPLES];
volatile uint16_t live_index       = 0;
volatile uint8_t  live_capturing   = 0;
volatile uint8_t  live_window_done = 0;
volatile uint32_t live_start_tick  = 0;

/* ============================================================
 *  Early alert — ISR-side running accumulators
 * ============================================================ */
volatile float    early_sum_sq_ns  = 0.0f;
volatile float    early_sum_ns     = 0.0f;
volatile uint16_t early_count      = 0;
volatile uint8_t  early_alert_fired = 0;
volatile uint32_t early_fire_tick  = 0;

/* Pre-computed for ISR variance classifier */
volatile float    offset_rms_sq = 0.0f;

/* ============================================================
 *  Alarm state (persists across windows)
 * ============================================================ */
volatile uint8_t  alarm_dc_active  = 0;
volatile uint8_t  alarm_ac_active  = 0;
volatile uint32_t last_sample_tick = 0;   /* for signal-loss watchdog */

/* ============================================================
 *  Modbus registers (the "database")
 * ============================================================ */
volatile uint16_t mb_dc_current   = 0;     /* 0x0000 — written by process_capture_window */
volatile uint16_t mb_ac_current   = 0;     /* 0x0001 — written by process_capture_window */
volatile uint16_t mb_dc_thresh    = 60;    /* 0x0002 — 60 = 6.0 mA default              */
volatile uint16_t mb_ac_thresh    = 300;   /* 0x0003 — 300 = 30.0 mA default            */
volatile uint16_t mb_slave_addr   = 1;     /* 0x0100 — slave address                    */

/* Modbus frame buffers */
static uint8_t  mb_rx_buf[MB_FRAME_BUF];
static uint8_t  mb_tx_buf[MB_FRAME_BUF];
static uint8_t  mb_rx_idx      = 0;
static uint8_t  mb_rx_len      = 0;
static uint8_t  mb_frame_ready = 0;
static uint32_t mb_last_rx_tick = 0;

/* ============================================================
 *  System state machine
 * ============================================================ */
typedef enum {
    STATE_IDLE,
    STATE_OFFSET_SAMPLING,
    STATE_OFFSET_DONE,
    STATE_MG_SAMPLING,
    STATE_MG_DONE,
    STATE_LIVE_CAPTURE
} SystemState;

volatile SystemState sys_state  = STATE_IDLE;
volatile uint8_t     data_ready = 0;
volatile uint8_t     cal_status = 0;   /* 1 = calibration complete */

/* ============================================================
 *  Function prototypes
 * ============================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

static void auto_calibrate(void);
static void start_capture_window(void);
static void compute_early_thresholds(void);
static void process_capture_window(void);
static void sync_alarm_pins(void);

/* Modbus helpers */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
static uint16_t modbus_read_register(uint16_t addr);
static uint8_t  modbus_write_register(uint16_t addr, uint16_t value);
static void     modbus_poll_rx(void);
static void     modbus_process_frame(void);
static void     modbus_send(const uint8_t *buf, uint8_t len);

/* ============================================================
 *  CRC-16 (Modbus polynomial 0xA001)
 * ============================================================ */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ============================================================
 *  Modbus register read  (returns 0xFFFF for invalid addr)
 * ============================================================ */
static uint16_t modbus_read_register(uint16_t addr)
{
    switch (addr)
    {
        case MB_REG_DC_CURRENT:  return mb_dc_current;
        case MB_REG_AC_CURRENT:  return mb_ac_current;
        case MB_REG_DC_THRESH:   return mb_dc_thresh;
        case MB_REG_AC_THRESH:   return mb_ac_thresh;
        case MB_REG_SLAVE_ADDR:  return mb_slave_addr;
        default:                 return 0xFFFF;
    }
}

/* ============================================================
 *  Modbus register write  (returns 0=OK, 1=illegal addr/data)
 * ============================================================ */
static uint8_t modbus_write_register(uint16_t addr, uint16_t value)
{
    switch (addr)
    {
        case MB_REG_DC_THRESH:
            /* Accept any non-zero value (0 would disable alarm) */
            if (value == 0) return 1;
            mb_dc_thresh = value;
            return 0;

        case MB_REG_AC_THRESH:
            if (value == 0) return 1;
            mb_ac_thresh = value;
            return 0;

        case MB_REG_SLAVE_ADDR:
            /* Valid slave addresses: 1–247 */
            if (value < 1 || value > 247) return 1;
            mb_slave_addr = value;
            return 0;

        /* Current registers are read-only */
        case MB_REG_DC_CURRENT:
        case MB_REG_AC_CURRENT:
        default:
            return 1;
    }
}

/* ============================================================
 *  Transmit a Modbus response frame
 * ============================================================ */
static void modbus_send(const uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        while (!(USART2->ISR & USART_ISR_TXE)) {}
        USART2->TDR = buf[i];
    }
}

/* ============================================================
 *  Poll USART2 RX for incoming Modbus bytes.
 *  Call every main-loop iteration (~10 µs).
 *  Sets mb_frame_ready after MB_INTER_FRAME_MS silence.
 * ============================================================ */
static void modbus_poll_rx(void)
{
    /* Accumulate incoming bytes */
    while (USART2->ISR & USART_ISR_RXNE)
    {
        uint8_t ch = (uint8_t)(USART2->RDR & 0xFF);
        if (mb_rx_idx < MB_FRAME_BUF)
            mb_rx_buf[mb_rx_idx++] = ch;
        mb_last_rx_tick = HAL_GetTick();
        mb_frame_ready  = 0;   /* reset — still receiving */
    }

    /* Detect end-of-frame: 5 ms silence after last byte */
    if (mb_rx_idx > 0 && !mb_frame_ready)
    {
        if ((HAL_GetTick() - mb_last_rx_tick) >= MB_INTER_FRAME_MS)
        {
            mb_rx_len      = mb_rx_idx;
            mb_rx_idx      = 0;
            mb_frame_ready = 1;
        }
    }
}

/* ============================================================
 *  Process a complete Modbus frame.
 *  Supports FC 0x03 (Read Holding Registers)
 *  and      FC 0x10 (Write Multiple Registers).
 *  Responds to broadcast address 0x00 (no reply sent).
 * ============================================================ */
static void modbus_process_frame(void)
{
    if (!mb_frame_ready) return;
    mb_frame_ready = 0;

    /* Minimum frame length: 4 bytes (addr + func + 2 CRC) */
    if (mb_rx_len < 4) return;

    uint8_t rx_addr = mb_rx_buf[0];

    /* Accept our address or broadcast 0x00 */
    if (rx_addr != (uint8_t)mb_slave_addr && rx_addr != 0x00) return;

    /* Validate CRC */
    uint16_t rx_crc   = ((uint16_t)mb_rx_buf[mb_rx_len - 1] << 8) |
                         (uint16_t)mb_rx_buf[mb_rx_len - 2];
    uint16_t calc_crc = modbus_crc16(mb_rx_buf, mb_rx_len - 2);
    if (rx_crc != calc_crc) return;

    uint8_t func = mb_rx_buf[1];

    /* ---- FC 0x03: Read Holding Registers ---- */
    if (func == MB_FC_READ)
    {
        /* Request: [addr][0x03][reg_hi][reg_lo][qty_hi][qty_lo][CRC_lo][CRC_hi] */
        if (mb_rx_len < 8) return;

        uint16_t start_reg = ((uint16_t)mb_rx_buf[2] << 8) | mb_rx_buf[3];
        uint16_t num_regs  = ((uint16_t)mb_rx_buf[4] << 8) | mb_rx_buf[5];

        if (num_regs < 1 || num_regs > 10) /* sanity limit */
        {
            /* Exception: illegal data value */
            if (rx_addr == 0x00) return;
            mb_tx_buf[0] = (uint8_t)mb_slave_addr;
            mb_tx_buf[1] = 0x83;   /* 0x03 | 0x80 */
            mb_tx_buf[2] = MB_EX_ILLEGAL_DATA;
            uint16_t crc = modbus_crc16(mb_tx_buf, 3);
            mb_tx_buf[3] = (uint8_t)(crc & 0xFF);
            mb_tx_buf[4] = (uint8_t)(crc >> 8);
            modbus_send(mb_tx_buf, 5);
            return;
        }

        /* Check all requested registers are valid */
        for (uint16_t i = 0; i < num_regs; i++)
        {
            if (modbus_read_register(start_reg + i) == 0xFFFF &&
                start_reg + i != MB_REG_DC_CURRENT &&   /* 0x0000 could legitimately be 0 */
                start_reg + i != MB_REG_AC_CURRENT)
            {
                /* Attempt to read a register we don't know */
                uint16_t reg = start_reg + i;
                if (reg != MB_REG_DC_CURRENT && reg != MB_REG_AC_CURRENT &&
                    reg != MB_REG_DC_THRESH   && reg != MB_REG_AC_THRESH &&
                    reg != MB_REG_SLAVE_ADDR)
                {
                    if (rx_addr == 0x00) return;
                    mb_tx_buf[0] = (uint8_t)mb_slave_addr;
                    mb_tx_buf[1] = 0x83;
                    mb_tx_buf[2] = MB_EX_ILLEGAL_ADDR;
                    uint16_t crc2 = modbus_crc16(mb_tx_buf, 3);
                    mb_tx_buf[3] = (uint8_t)(crc2 & 0xFF);
                    mb_tx_buf[4] = (uint8_t)(crc2 >> 8);
                    modbus_send(mb_tx_buf, 5);
                    return;
                }
            }
        }

        /* Build response */
        uint8_t byte_count = (uint8_t)(num_regs * 2);
        mb_tx_buf[0] = (uint8_t)mb_slave_addr;
        mb_tx_buf[1] = MB_FC_READ;
        mb_tx_buf[2] = byte_count;

        for (uint16_t i = 0; i < num_regs; i++)
        {
            uint16_t val = modbus_read_register(start_reg + i);
            mb_tx_buf[3 + i * 2]     = (uint8_t)(val >> 8);    /* big-endian */
            mb_tx_buf[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
        }

        uint8_t  resp_len = 3 + byte_count;
        uint16_t crc3     = modbus_crc16(mb_tx_buf, resp_len);
        mb_tx_buf[resp_len]     = (uint8_t)(crc3 & 0xFF);
        mb_tx_buf[resp_len + 1] = (uint8_t)(crc3 >> 8);

        if (rx_addr != 0x00)   /* no reply to broadcast */
            modbus_send(mb_tx_buf, resp_len + 2);
        return;
    }

    /* ---- FC 0x10: Write Multiple Registers ---- */
    if (func == MB_FC_WRITE_MULTI)
    {
        /*
         * Request:
         * [addr][0x10][reg_hi][reg_lo][qty_hi][qty_lo][byte_cnt]
         * [data_hi_0][data_lo_0] ... [CRC_lo][CRC_hi]
         */
        if (mb_rx_len < 9) return;

        uint16_t start_reg  = ((uint16_t)mb_rx_buf[2] << 8) | mb_rx_buf[3];
        uint16_t num_regs   = ((uint16_t)mb_rx_buf[4] << 8) | mb_rx_buf[5];
        uint8_t  byte_count = mb_rx_buf[6];

        if (num_regs < 1 || byte_count != (uint8_t)(num_regs * 2) ||
            mb_rx_len < (uint8_t)(7 + byte_count + 2))
        {
            if (rx_addr == 0x00) return;
            mb_tx_buf[0] = (uint8_t)mb_slave_addr;
            mb_tx_buf[1] = 0x90;
            mb_tx_buf[2] = MB_EX_ILLEGAL_DATA;
            uint16_t crc = modbus_crc16(mb_tx_buf, 3);
            mb_tx_buf[3] = (uint8_t)(crc & 0xFF);
            mb_tx_buf[4] = (uint8_t)(crc >> 8);
            modbus_send(mb_tx_buf, 5);
            return;
        }

        /* Write each register */
        for (uint16_t i = 0; i < num_regs; i++)
        {
            uint16_t val = ((uint16_t)mb_rx_buf[7 + i * 2] << 8) |
                            (uint16_t)mb_rx_buf[8 + i * 2];
            if (modbus_write_register(start_reg + i, val) != 0)
            {
                if (rx_addr == 0x00) return;
                mb_tx_buf[0] = (uint8_t)mb_slave_addr;
                mb_tx_buf[1] = 0x90;
                mb_tx_buf[2] = MB_EX_ILLEGAL_DATA;
                uint16_t crc = modbus_crc16(mb_tx_buf, 3);
                mb_tx_buf[3] = (uint8_t)(crc & 0xFF);
                mb_tx_buf[4] = (uint8_t)(crc >> 8);
                modbus_send(mb_tx_buf, 5);
                return;
            }
        }

        /* Echo response: [addr][0x10][reg_hi][reg_lo][qty_hi][qty_lo][CRC] */
        mb_tx_buf[0] = (uint8_t)mb_slave_addr;
        mb_tx_buf[1] = MB_FC_WRITE_MULTI;
        mb_tx_buf[2] = mb_rx_buf[2];
        mb_tx_buf[3] = mb_rx_buf[3];
        mb_tx_buf[4] = mb_rx_buf[4];
        mb_tx_buf[5] = mb_rx_buf[5];
        uint16_t crc4 = modbus_crc16(mb_tx_buf, 6);
        mb_tx_buf[6] = (uint8_t)(crc4 & 0xFF);
        mb_tx_buf[7] = (uint8_t)(crc4 >> 8);

        if (rx_addr != 0x00)
            modbus_send(mb_tx_buf, 8);
        return;
    }

    /* ---- Unsupported function code ---- */
    if (rx_addr != 0x00)
    {
        mb_tx_buf[0] = (uint8_t)mb_slave_addr;
        mb_tx_buf[1] = func | 0x80;
        mb_tx_buf[2] = MB_EX_ILLEGAL_FUNC;
        uint16_t crc = modbus_crc16(mb_tx_buf, 3);
        mb_tx_buf[3] = (uint8_t)(crc & 0xFF);
        mb_tx_buf[4] = (uint8_t)(crc >> 8);
        modbus_send(mb_tx_buf, 5);
    }
}

/* ============================================================
 *  Compute early alert thresholds (called after calibration)
 * ============================================================ */
static void compute_early_thresholds(void)
{
    /* Only offset_rms² is pre-computed for ISR speed.
     * The actual early alarm thresholds are derived dynamically in the ISR
     * from the live mb_ac_thresh / mb_dc_thresh Modbus registers so that
     * any threshold change written via Modbus takes effect immediately. */
    offset_rms_sq = offset_rms_ns * offset_rms_ns;
}

/* ============================================================
 *  Sync alarm output pins from alarm state variables
 * ============================================================ */
static void sync_alarm_pins(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,
                      alarm_dc_active ? GPIO_PIN_SET : GPIO_PIN_RESET);  /* DC alarm */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,
                      alarm_ac_active ? GPIO_PIN_SET : GPIO_PIN_RESET);  /* AC alarm */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
                      (alarm_dc_active || alarm_ac_active) ? GPIO_PIN_SET : GPIO_PIN_RESET);  /* AC+DC combined */
}

/* ============================================================
 *  Start one 80 ms capture window
 * ============================================================ */
static void start_capture_window(void)
{
    live_index     = 0;
    live_capturing = 1;
    live_window_done = 0;
    cap_state      = 0;

    early_sum_sq_ns   = 0.0f;
    early_sum_ns      = 0.0f;
    early_count       = 0;
    early_alert_fired = 0;
    early_fire_tick   = 0;

    live_start_tick = HAL_GetTick();
}

/* ============================================================
 *  Auto-calibration: runs once on boot.
 *  Phase 1 — offset (no current)  ~400 ms + 1 s stabilise wait
 *  Phase 2 — gain  (24 mA ref)    ~400 ms + 500 ms settle
 * ============================================================ */
static void auto_calibrate(void)
{
    /* --- 1-second timer stabilisation --- */
    HAL_Delay(1000);

    /* --- Offset measurement --- */
    sample_count      = 0;
    del_t_sum_ns      = 0.0f;
    del_t_sum_sq_ns   = 0.0f;
    cap_state         = 0;
    data_ready        = 0;
    sys_state         = STATE_OFFSET_SAMPLING;

    while (sys_state == STATE_OFFSET_SAMPLING)
    {
        /* Spin — ISR will set sys_state = STATE_OFFSET_DONE */
    }

    offset_avg_ns = del_t_sum_ns      / (float)SAMPLE_COUNT;
    offset_rms_ns = sqrtf(del_t_sum_sq_ns / (float)SAMPLE_COUNT);
    sys_state     = STATE_IDLE;
    HAL_Delay(500);

    /* --- Gain measurement --- */
    sample_count    = 0;
    del_t_sum_ns    = 0.0f;
    del_t_sum_sq_ns = 0.0f;
    cap_state       = 0;
    data_ready      = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);   /* inject 24 mA */
    HAL_Delay(500);                                         /* settle        */
    sys_state = STATE_MG_SAMPLING;

    while (sys_state == STATE_MG_SAMPLING)
    {
        /* Spin — ISR will set sys_state = STATE_MG_DONE */
    }

    mg_ns  = del_t_sum_ns / (float)SAMPLE_COUNT;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); /* remove 24 mA */

    Cg_ns          = mg_ns - offset_avg_ns;
    sensitivity_ns = (Cg_ns != 0.0f) ? (Cg_ns / 24.0f) : 0.0f;

    compute_early_thresholds();

    cal_status = 1;
    sys_state  = STATE_IDLE;
}

/* ============================================================
 *  Process one completed 80 ms window (main loop — NOT ISR)
 *
 *  CLASSIFIER: variance-based (immune to window phase and timing jitter)
 *
 *    variance = rms² - avg²
 *    DC signal: variance ≈ offset_rms²            (noise floor only)
 *    AC signal: variance = (I_rms × sensitivity)² >> offset_rms²
 *
 *    Threshold = 4 × offset_rms² → detects AC ≥ ~3 mA RMS reliably.
 *    Immune to partial-cycle window alignment and HAL_GetTick ±0.5 ms jitter.
 *
 *  AC FORMULA: I = sqrt(variance) / sensitivity   (= sqrt(rms² - avg²) / sens)
 *    avg² removes the DC offset contribution; no cycle trimming needed.
 *    Works for any AC frequency; no zero-crossing dependency.
 * ============================================================ */
static void process_capture_window(void)
{
    uint16_t n = live_index;
    if (n < 2) return;

    /* ---- Pass 1: scan all samples ---- */
    float sum_dt_ns = 0.0f;
    float sum_sq_ns = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        float dt = live_samples_ns[i];
        sum_dt_ns += dt;
        sum_sq_ns += dt * dt;
    }

    float avg_cycle_ns = sum_dt_ns / (float)n;
    float rms_sq_ns    = sum_sq_ns / (float)n;

    /* ---- AC/DC classification: variance-based ---- */
    /*
     * variance = rms² - avg²
     * DC:  variance ≈ offset_rms²  (signal is flat; only noise contributes)
     * AC:  variance = (I_rms × sensitivity)² which is >> offset_rms²
     *      even at 9 mA AC: (9 × 37.62)² = 114,636 ns² vs threshold 12,996 ns²
     *
     * Previous avg_deviation check was broken: a 50 Hz AC signal captured
     * in a window with ±0.5 ms HAL_GetTick jitter produces avg_deviation
     * up to 416 ns (50 mA AC) >> offset_rms (57 ns) → wrongly classified DC.
     * Variance is immune because avg² cancels regardless of phase offset.
     */
    float variance_ns   = rms_sq_ns - (avg_cycle_ns * avg_cycle_ns);
    float var_threshold = 4.0f * offset_rms_ns * offset_rms_ns;  /* 4 × noise floor */

    uint8_t is_ac = (variance_ns > var_threshold) ? 1 : 0;

    /* ---- Current calculation ---- */
    float current_mA  = 0.0f;
    float abs_current = 0.0f;

    if (is_ac)
    {
        /*
         * I_ac = sqrt(variance) / sensitivity
         *      = sqrt(rms² - avg²) / sensitivity
         *
         * avg² removes the DC hardware offset contribution.
         * No cycle trimming needed: variance is phase-invariant.
         */
        if (sensitivity_ns != 0.0f && variance_ns > 0.0f)
            current_mA = sqrtf(variance_ns) / fabsf(sensitivity_ns);
        abs_current = current_mA;
    }
    else
    {
        /* DC: signed average */
        if (sensitivity_ns != 0.0f)
            current_mA = (avg_cycle_ns - offset_avg_ns) / sensitivity_ns;
        abs_current = fabsf(current_mA);
    }

    /* ---- Write Modbus registers ---- */
    if (is_ac)
    {
        mb_dc_current = 0;
        mb_ac_current = (uint16_t)(abs_current * 100.0f + 0.5f);
    }
    else
    {
        mb_dc_current = (uint16_t)(abs_current * 100.0f + 0.5f);
        mb_ac_current = 0;
    }

    /* ---- Evaluate alarms against Modbus thresholds ---- */
    /*
     * mb_dc_thresh is in 0.1 mA units → divide by 10.0 for mA comparison
     * mb_ac_thresh is in 0.1 mA units → divide by 10.0 for mA comparison
     */
    float dc_thresh_mA = (float)mb_dc_thresh / 10.0f;
    float ac_thresh_mA = (float)mb_ac_thresh / 10.0f;

    if (is_ac)
    {
        if (abs_current >= ac_thresh_mA)
            alarm_ac_active = 1;
        else if (abs_current < ac_thresh_mA * ALARM_HYST_FACTOR)
            alarm_ac_active = 0;

        alarm_dc_active = 0;
    }
    else
    {
        if (abs_current >= dc_thresh_mA)
            alarm_dc_active = 1;
        else if (abs_current < dc_thresh_mA * ALARM_HYST_FACTOR)
            alarm_dc_active = 0;

        alarm_ac_active = 0;
    }

    sync_alarm_pins();

    /* Update signal-loss watchdog */
    last_sample_tick = HAL_GetTick();
}

/* ============================================================
 *  Timer Input Capture ISR
 * ============================================================ */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (sys_state == STATE_IDLE) return;
    if (htim->Instance != TIM1) return;

    /* ---- CH1: A1 or A2 ---- */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint16_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (cap_state == 0)
        {
            cap_a1    = capture;
            cap_state = 1;
        }
        else if (cap_state == 2)
        {
            cap_a2 = capture;

            uint16_t t1_ticks = cap_b  - cap_a1;
            uint16_t t2_ticks = cap_a2 - cap_b;
            float    t1_ns    = (float)t1_ticks * TICK_NS;
            float    t2_ns    = (float)t2_ticks * TICK_NS;
            float    del_t_ns = t1_ns - t2_ns;

            /* ---- Offset sampling ---- */
            if (sys_state == STATE_OFFSET_SAMPLING)
            {
                del_t_sum_ns    += del_t_ns;
                del_t_sum_sq_ns += del_t_ns * del_t_ns;
                sample_count++;
                if (sample_count >= SAMPLE_COUNT)
                    sys_state = STATE_OFFSET_DONE;
            }
            /* ---- Gain sampling ---- */
            else if (sys_state == STATE_MG_SAMPLING)
            {
                del_t_sum_ns += del_t_ns;
                sample_count++;
                if (sample_count >= SAMPLE_COUNT)
                    sys_state = STATE_MG_DONE;
            }
            /* ---- Live capture with continuous early alert ---- */
            else if (sys_state == STATE_LIVE_CAPTURE && live_capturing)
            {
                uint32_t elapsed = HAL_GetTick() - live_start_tick;

                if (elapsed < CAPTURE_WINDOW_MS)
                {
                    /* Store sample */
                    if (live_index < MAX_LIVE_SAMPLES)
                        live_samples_ns[live_index++] = del_t_ns;

                    /* Accumulate for early check */
                    early_sum_sq_ns += del_t_ns * del_t_ns;
                    early_sum_ns    += del_t_ns;
                    early_count++;

                    /* Continuous early threshold check */
                    if (!early_alert_fired &&
                        elapsed >= EARLY_START_MS &&
                        early_count >= EARLY_MIN_SAMPLES)
                    {
                        /*
                         * Early check uses the same variance logic as Stage 2.
                         *
                         * variance = rms² - avg²
                         * AC: variance >> var_threshold → fire AC alarm (PB3 + PB1)
                         * DC: variance ≤ var_threshold → fire DC alarm if avg signal large (PB4 + PB1)
                         *
                         * Previous quick_ac = (has_pos && has_neg) was broken:
                         * with only 15 samples (~0.75 cycle at 50 Hz) the window
                         * may capture only the positive half → has_neg stays 0
                         * → treated as DC → fired on half-wave amplitude.
                         *
                         * No sqrt in ISR: compare signal² against threshold² directly.
                         */
                        float inv_n        = 1.0f / (float)early_count;
                        float quick_rms_sq = early_sum_sq_ns * inv_n;
                        float quick_avg    = early_sum_ns    * inv_n;
                        float quick_var    = quick_rms_sq - (quick_avg * quick_avg);

                        /* var_threshold = 4 × offset_rms² (pre-computed) */
                        float var_thr = 4.0f * offset_rms_sq;

                        /*
                         * Early thresholds derived dynamically from live Modbus registers.
                         * early = mb_thresh / HYST_FACTOR = mb_thresh / 0.90 = mb_thresh × 1.111
                         * This holds for ANY threshold value written via Modbus:
                         *   mb_ac_thresh in 0.1 mA units → /10 → mA → × sensitivity → ns → squared
                         */
                        float early_inv_hyst = 1.0f / ALARM_HYST_FACTOR;   /* 1.111 */
                        float ac_early_ns    = ((float)mb_ac_thresh * 0.1f) * early_inv_hyst * fabsf(sensitivity_ns);
                        float dc_early_ns    = ((float)mb_dc_thresh * 0.1f) * early_inv_hyst * fabsf(sensitivity_ns);

                        if (quick_var > var_thr)
                        {
                            /* AC signal — compare variance against dynamic early threshold² */
                            if (quick_var > ac_early_ns * ac_early_ns)
                            {
                                alarm_ac_active = 1;
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);   /* AC alarm */
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   /* combined */
                                early_alert_fired = 1;
                                early_fire_tick   = HAL_GetTick();
                            }
                        }
                        else
                        {
                            /* DC signal — compare avg signal against dynamic early threshold */
                            float dc_signal = fabsf(quick_avg - offset_avg_ns);
                            if (dc_signal > dc_early_ns)
                            {
                                alarm_dc_active = 1;
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   /* DC alarm */
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   /* combined */
                                early_alert_fired = 1;
                                early_fire_tick   = HAL_GetTick();
                            }
                        }
                    }
                }
                else
                {
                    /* 80 ms elapsed — close window */
                    live_capturing   = 0;
                    live_window_done = 1;
                }
            }

            cap_state = 0;
        }
    }

    /* ---- CH2: B ---- */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (cap_state == 1)
        {
            cap_b     = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
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
    MX_USART2_UART_Init();   /* 9600 baud for Modbus RTU */

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

    /* Run auto-calibration (~3 seconds) */
    auto_calibrate();

    /* Enter live capture */
    sys_state = STATE_LIVE_CAPTURE;
    start_capture_window();

    /* ================================================================
     *  Main loop
     *  Priority 1: close window → process → open next window
     *  Priority 2: poll Modbus RX + process frame
     *  Priority 3: signal-loss watchdog
     * ================================================================ */
    while (1)
    {
        /* --- Window complete: run precision engine --- */
        if (sys_state == STATE_LIVE_CAPTURE && live_window_done)
        {
            live_window_done = 0;
            process_capture_window();
            start_capture_window();
        }

        /* --- Modbus service --- */
        modbus_poll_rx();
        modbus_process_frame();

        /* --- Signal-loss watchdog ---
         * If no samples arrive for 2 seconds while alarms are active,
         * clear alarms to break the feedback deadlock
         * (alarm pin → electrical noise → signal lost → alarm never clears).
         */
        if ((alarm_dc_active || alarm_ac_active) &&
            last_sample_tick != 0 &&
            (HAL_GetTick() - last_sample_tick) > SIGNAL_LOSS_MS)
        {
            alarm_dc_active = 0;
            alarm_ac_active = 0;
            sync_alarm_pins();
        }
    }
}

/* ============================================================
 *  Clock & Peripheral Init
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct  = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct  = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit      = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_TIM1;
    PeriphClkInit.Tim1ClockSelection    = RCC_TIM1CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_IC_InitTypeDef      sConfigIC          = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 65535;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance            = USART2;
    huart2.Init.BaudRate       = 9600;       /* Modbus spec: 9600 baud */
    huart2.Init.WordLength     = UART_WORDLENGTH_8B;
    huart2.Init.StopBits       = UART_STOPBITS_1;
    huart2.Init.Parity         = UART_PARITY_NONE;
    huart2.Init.Mode           = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA12 = CAL (calibration pulse) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB4 = DC alarm output                   */
    /* PB3 = AC alarm output                   */
    /* PB1 = AC+DC combined alarm output       */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
