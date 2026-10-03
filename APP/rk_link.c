#include "rk_link.h"

#include "app_config.h"
#include "main.h"
#include "rk_protocol.h"
#include "usart.h"

#include <string.h>

#if MOTOR_SERIAL_TEST_ENABLE

#include "emm_v5.h"
#include <stdarg.h>
#include <stdio.h>

/* The console and binary RK protocol are mutually exclusive on USART1.
 * Only rk_link_poll() owns the parser, command state and reply queue. */
#define CONSOLE_RX_SIZE          256U
#define CONSOLE_LINE_SIZE         80U
#define CONSOLE_REPLY_SIZE       256U
#define CONSOLE_REPLY_COUNT        8U
#define CONSOLE_POLL_BYTE_BUDGET   64U
#define CONSOLE_TX_TIMEOUT_MS    100U
#define CONSOLE_MOTOR_COUNT        4U
#define CONSOLE_ALL_MOTORS        15U

#if MOTOR_TEST_MAX_RPM < 1 || MOTOR_TEST_MAX_RPM > 32767
#error MOTOR_TEST_MAX_RPM must be between 1 and 32767
#endif
#if MOTOR_TEST_MAX_DURATION_MS < 1 || MOTOR_TEST_MAX_DURATION_MS > 2147483647
#error MOTOR_TEST_MAX_DURATION_MS must be between 1 and 2147483647
#endif
#if MOTOR_TEST_MAX_ANGLE_DEG < 1 || MOTOR_TEST_MAX_ANGLE_DEG > 214748364
#error MOTOR_TEST_MAX_ANGLE_DEG must be between 1 and 214748364
#endif

typedef struct
{
    uint16_t length;
    char text[CONSOLE_REPLY_SIZE];
} console_reply_t;

static const uint8_t motor_addresses[CONSOLE_MOTOR_COUNT] = {
    CHASSIS_MOTOR_ADDR_0, CHASSIS_MOTOR_ADDR_1,
    CHASSIS_MOTOR_ADDR_2, CHASSIS_MOTOR_ADDR_3
};
static uint8_t rx_byte;
static uint8_t rx_ring[CONSOLE_RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overruns;
static volatile uint32_t uart_errors;
static volatile uint8_t rx_recover_pending;
static volatile uint8_t tx_complete_pending;
static uint8_t recovery_latched;
static uint32_t seen_overruns;
static uint32_t seen_uart_errors;

static char command_line[CONSOLE_LINE_SIZE];
static uint16_t line_used;
static uint8_t discard_line;
static uint8_t initialized_mask;
static uint8_t run_active;
static uint8_t run_index;
static int16_t run_rpm;
static uint32_t run_started_ms;
static uint32_t run_duration_ms;
static uint8_t stop_pending_mask;
static uint8_t stop_error_reported_mask;
static uint32_t seen_motor_failures;
static uint32_t seen_stale_motion;
static uint8_t seen_motor_fault;
static rk_link_stats_t link_stats;

static console_reply_t replies[CONSOLE_REPLY_COUNT];
static uint8_t reply_head;
static uint8_t reply_tail;
static uint8_t reply_count;
static uint32_t dropped_replies;
/* This buffer is retained until TC or successful AbortTransmit. */
static console_reply_t active_reply;
static uint8_t tx_active;
static uint8_t tx_started;
static uint32_t tx_started_ms;

static void reply(const char *format, ...)
{
    int length;
    va_list arguments;
    console_reply_t *item;

    if (reply_count == CONSOLE_REPLY_COUNT)
    {
        dropped_replies++;
        return;
    }
    item = &replies[reply_head];
    va_start(arguments, format);
    length = vsnprintf(item->text, sizeof(item->text), format, arguments);
    va_end(arguments);
    if (length < 0 || (size_t)length >= sizeof(item->text))
    {
        dropped_replies++;
        return;
    }
    item->length = (uint16_t)length;
    reply_head = (uint8_t)((reply_head + 1U) % CONSOLE_REPLY_COUNT);
    reply_count++;
}

static void poll_reply_tx(void)
{
    HAL_StatusTypeDef result;
    uint32_t now = HAL_GetTick();

    if (tx_active != 0U)
    {
        if (tx_started != 0U && tx_complete_pending != 0U)
        {
            tx_complete_pending = 0U;
            tx_active = 0U;
        }
        else if ((uint32_t)(now - tx_started_ms) >= CONSOLE_TX_TIMEOUT_MS)
        {
            /* Do not overwrite a buffer while HAL can still reference it. */
            if (HAL_UART_AbortTransmit(&huart1) != HAL_OK)
                return;
            tx_complete_pending = 0U;
            tx_active = 0U;
            link_stats.tx_errors++;
        }
    }

    if (tx_active == 0U && reply_count != 0U)
    {
        active_reply = replies[reply_tail];
        reply_tail = (uint8_t)((reply_tail + 1U) % CONSOLE_REPLY_COUNT);
        reply_count--;
        tx_complete_pending = 0U;
        tx_active = 1U;
        tx_started = 0U;
        tx_started_ms = now;
    }
    if (tx_active != 0U && tx_started == 0U)
    {
        result = HAL_UART_Transmit_IT(&huart1,
                    (uint8_t *)active_reply.text, active_reply.length);
        if (result == HAL_OK)
            tx_started = 1U;
        /* BUSY/ERROR retain the same buffer. Timeout recovery above aborts
         * any unfinished transfer before another reply can reuse it. */
    }
}

static void start_uart_receive(void)
{
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK)
        rx_recover_pending = 1U;
}

static void reset_input_after_fault(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rx_tail = rx_head;
    __set_PRIMASK(primask);
    line_used = 0U;
    discard_line = 1U; /* Require a fresh line boundary after lost bytes. */
    initialized_mask = 0U;
    stop_pending_mask |= CONSOLE_ALL_MOTORS;
}

static void recover_uart_receive(void)
{
    uint32_t overruns = rx_overruns;
    uint32_t errors = uart_errors;
    if (overruns != seen_overruns)
    {
        seen_overruns = overruns;
        reset_input_after_fault();
        reply("ERR RX OVERFLOW; STOP ALL pending; send newline then INIT\r\n");
    }
    if (errors != seen_uart_errors)
    {
        seen_uart_errors = errors;
        /* A new error must invalidate input even during an earlier recovery. */
        recovery_latched = 0U;
    }
    if (rx_recover_pending == 0U)
        return;
    if (recovery_latched == 0U)
    {
        recovery_latched = 1U;
        reset_input_after_fault();
        reply("ERR UART RX; STOP ALL pending; send newline then INIT\r\n");
    }
    if (HAL_UART_AbortReceive(&huart1) == HAL_OK)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        rx_recover_pending = 0U;
        start_uart_receive();
        if (rx_recover_pending == 0U)
            recovery_latched = 0U;
    }
}

static void service_motor_stop(void)
{
    uint8_t index;
    uint8_t bit;
    HAL_StatusTypeDef result;

    if (run_active != 0U &&
        (uint32_t)(HAL_GetTick() - run_started_ms) >= run_duration_ms)
        stop_pending_mask |= (uint8_t)(1U << run_index);

    for (index = 0U; index < CONSOLE_MOTOR_COUNT; index++)
    {
        bit = (uint8_t)(1U << index);
        if ((stop_pending_mask & bit) == 0U)
            continue;
        result = emm_v5_stop(motor_addresses[index], 0U);
        if (result != HAL_OK)
        {
            if ((stop_error_reported_mask & bit) == 0U)
                reply("ERR STOP %u status=%u; PENDING RETRY\r\n",
                    (unsigned)motor_addresses[index], (unsigned)result);
            stop_error_reported_mask |= bit;
            continue;
        }
        stop_pending_mask &= (uint8_t)~bit;
        stop_error_reported_mask &= (uint8_t)~bit;
        if (run_active != 0U && run_index == index)
            run_active = 0U;
        reply("OK STOP %u QUEUED (no motor feedback)\r\n",
            (unsigned)motor_addresses[index]);
    }
}

static void observe_motor_fault(void)
{
    emm_v5_stats_t stats;
    emm_v5_get_stats(&stats);
    if (stats.tx_failed != seen_motor_failures ||
        stats.stale_motion != seen_stale_motion ||
        (stats.faulted != 0U && seen_motor_fault == 0U))
    {
        initialized_mask = 0U;
        stop_pending_mask |= CONSOLE_ALL_MOTORS;
        reply("ERR motor transport failed/stale; STOP ALL pending; INIT required\r\n");
    }
    seen_motor_failures = stats.tx_failed;
    seen_stale_motion = stats.stale_motion;
    seen_motor_fault = stats.faulted;
}

static int motor_index(uint32_t address)
{
    uint8_t index;
    for (index = 0U; index < CONSOLE_MOTOR_COUNT; index++)
        if (address == motor_addresses[index])
            return (int)index;
    return -1;
}

static uint8_t parse_u32(const char *text, uint32_t *value)
{
    uint32_t number = 0U;
    uint32_t digit;
    if (*text == '\0')
        return 0U;
    while (*text != '\0')
    {
        if (*text < '0' || *text > '9')
            return 0U;
        digit = (uint32_t)(*text++ - '0');
        if (number > (UINT32_MAX - digit) / 10U)
            return 0U;
        number = number * 10U + digit;
    }
    *value = number;
    return 1U;
}

static uint8_t parse_i32(const char *text, int32_t *value)
{
    uint8_t negative = 0U;
    uint32_t magnitude;
    if (*text == '-' || *text == '+')
        negative = *text++ == '-' ? 1U : 0U;
    if (parse_u32(text, &magnitude) == 0U ||
        magnitude > (negative != 0U ? 2147483648U : 2147483647U))
        return 0U;
    if (negative != 0U && magnitude == 2147483648U)
        *value = INT32_MIN;
    else
        *value = negative != 0U ? -(int32_t)magnitude : (int32_t)magnitude;
    return 1U;
}

static uint8_t word_equals(const char *word, const char *expected)
{
    char ch;
    while (*word != '\0' && *expected != '\0')
    {
        ch = *word++;
        if (ch >= 'a' && ch <= 'z')
            ch = (char)(ch - 'a' + 'A');
        if (ch != *expected++)
            return 0U;
    }
    return *word == '\0' && *expected == '\0' ? 1U : 0U;
}

static void process_line(void)
{
    char *words[5];
    char *cursor = command_line;
    uint8_t count = 0U;
    uint32_t address;
    uint32_t duration;
    uint32_t angle_rpm;
    int32_t angle_deg;
    int32_t rpm;
    int index;
    HAL_StatusTypeDef result;
    emm_v5_stats_t motor_stats;

    while (*cursor != '\0')
    {
        while (*cursor == ' ' || *cursor == '\t')
            cursor++;
        if (*cursor == '\0')
            break;
        if (count == 5U)
        {
            reply("ERR too many arguments\r\n");
            return;
        }
        words[count++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
            cursor++;
        if (*cursor != '\0')
            *cursor++ = '\0';
    }
    if (count == 0U)
        return;
    observe_motor_fault();

    /* Refuse additional motion/init requests when replies cannot be queued.
     * STOP is always serviced, even under a flood of input. */
    if (reply_count == CONSOLE_REPLY_COUNT && word_equals(words[0], "STOP") == 0U)
    {
        dropped_replies++;
        return;
    }

    if (word_equals(words[0], "HELP") != 0U && count == 1U)
    {
        reply("HELP | INIT <addr> | RUN <addr> <signed_rpm> <ms> | ANGLE <addr> <signed_deg> <rpm> | STOP <addr|ALL> | STATUS\r\n"
              "Limits: run rpm=+/-%u, angle=+/-%u deg, angle rpm=1..%u; ms=1..%lu; INIT first; no motor feedback\r\n",
              (unsigned)MOTOR_TEST_MAX_RPM, (unsigned)MOTOR_TEST_MAX_ANGLE_DEG,
              (unsigned)MOTOR_TEST_MAX_RPM, (unsigned long)MOTOR_TEST_MAX_DURATION_MS);
        return;
    }
    if (word_equals(words[0], "STATUS") != 0U && count == 1U)
    {
        emm_v5_get_stats(&motor_stats);
        reply("STATUS init_mask=0x%02X run_queued=%u addr=%u rpm=%d stop_pending=0x%02X; no motor feedback\r\n",
              (unsigned)initialized_mask, (unsigned)run_active,
              run_active != 0U ? (unsigned)motor_addresses[run_index] : 0U,
              run_active != 0U ? (int)run_rpm : 0,
              (unsigned)stop_pending_mask);
        reply("UART rx_drop=%lu uart_err=%lu tx_err=%lu reply_drop=%lu\r\n",
              (unsigned long)rx_overruns,
              (unsigned long)uart_errors, (unsigned long)link_stats.tx_errors,
              (unsigned long)dropped_replies);
        reply("MOTOR tx_completed=%lu tx_failed=%lu queue_full=%lu stale_motion=%lu pending=%lu faulted=%u motion_blocked=%u\r\n",
              (unsigned long)motor_stats.tx_completed, (unsigned long)motor_stats.tx_failed,
              (unsigned long)motor_stats.queue_full, (unsigned long)motor_stats.stale_motion,
              (unsigned long)motor_stats.pending, (unsigned)motor_stats.faulted,
              (unsigned)motor_stats.motion_blocked);
        return;
    }
    if (word_equals(words[0], "STOP") != 0U && count == 2U)
    {
        if (word_equals(words[1], "ALL") != 0U)
            stop_pending_mask |= CONSOLE_ALL_MOTORS;
        else if (parse_u32(words[1], &address) != 0U &&
                 (index = motor_index(address)) >= 0)
            stop_pending_mask |= (uint8_t)(1U << index);
        else
        {
            reply("ERR invalid motor address\r\n");
            return;
        }
        link_stats.valid_frames++;
        service_motor_stop();
        return;
    }
    if (word_equals(words[0], "INIT") != 0U && count == 2U)
    {
        emm_v5_get_stats(&motor_stats);
        if (parse_u32(words[1], &address) == 0U ||
            (index = motor_index(address)) < 0)
        {
            reply("ERR invalid motor address\r\n");
            return;
        }
        if (run_active != 0U || stop_pending_mask != 0U || motor_stats.pending != 0U)
        {
            reply("ERR busy; wait for pending TX/STOP or RUN deadline\r\n");
            return;
        }
        initialized_mask &= (uint8_t)~(1U << index);
        result = emm_v5_init_motor((uint8_t)address);
        if (result == HAL_OK)
        {
            emm_v5_get_stats(&motor_stats);
            seen_motor_failures = motor_stats.tx_failed;
            seen_stale_motion = motor_stats.stale_motion;
            seen_motor_fault = motor_stats.faulted;
            initialized_mask |= (uint8_t)(1U << index);
            link_stats.valid_frames++;
            reply("OK INIT %u TX OK (no motor feedback)\r\n", (unsigned)address);
        }
        else
            reply("ERR INIT %u status=%u\r\n", (unsigned)address, (unsigned)result);
        return;
    }
    if (word_equals(words[0], "ANGLE") != 0U && count == 4U)
    {
        emm_v5_get_stats(&motor_stats);
        if (parse_u32(words[1], &address) == 0U ||
            (index = motor_index(address)) < 0)
        {
            reply("ERR invalid motor address\r\n");
            return;
        }
        if (parse_i32(words[2], &angle_deg) == 0U ||
            angle_deg < -(int32_t)MOTOR_TEST_MAX_ANGLE_DEG ||
            angle_deg > (int32_t)MOTOR_TEST_MAX_ANGLE_DEG ||
            parse_u32(words[3], &angle_rpm) == 0U ||
            angle_rpm == 0U || angle_rpm > MOTOR_TEST_MAX_RPM)
        {
            reply("ERR limits: angle=+/-%u deg; rpm=1..%u\r\n",
                  (unsigned)MOTOR_TEST_MAX_ANGLE_DEG,
                  (unsigned)MOTOR_TEST_MAX_RPM);
            return;
        }
        if (run_active != 0U || stop_pending_mask != 0U || motor_stats.pending != 0U)
        {
            reply("ERR busy; wait for pending TX/STOP or RUN deadline\r\n");
            return;
        }
        if ((initialized_mask & (1U << index)) == 0U)
        {
            reply("ERR INIT %u required first\r\n", (unsigned)address);
            return;
        }
        result = emm_v5_set_position((uint8_t)address, angle_deg * 10,
                                     (uint16_t)angle_rpm, CHASSIS_MOTOR_ACCEL,
                                     CHASSIS_MOTOR_DECEL, 2U, 0U);
        if (result != HAL_OK)
        {
            reply("ERR ANGLE %u status=%u; not queued\r\n",
                  (unsigned)address, (unsigned)result);
            return;
        }
        link_stats.valid_frames++;
        reply("OK ANGLE %u %lddeg %luRPM QUEUED; relative current position; no motor feedback\r\n",
              (unsigned)address, (long)angle_deg, (unsigned long)angle_rpm);
        return;
    }
    if (word_equals(words[0], "RUN") != 0U && count == 4U)
    {
        emm_v5_get_stats(&motor_stats);
        if (parse_u32(words[1], &address) == 0U ||
            (index = motor_index(address)) < 0)
        {
            reply("ERR invalid motor address\r\n");
            return;
        }
        if (parse_i32(words[2], &rpm) == 0U || rpm == 0 ||
            rpm < -(int32_t)MOTOR_TEST_MAX_RPM || rpm > (int32_t)MOTOR_TEST_MAX_RPM ||
            parse_u32(words[3], &duration) == 0U || duration == 0U ||
            duration > MOTOR_TEST_MAX_DURATION_MS)
        {
            reply("ERR limits: nonzero rpm within +/-%u; ms=1..%lu\r\n",
                  (unsigned)MOTOR_TEST_MAX_RPM, (unsigned long)MOTOR_TEST_MAX_DURATION_MS);
            return;
        }
        if (run_active != 0U || stop_pending_mask != 0U || motor_stats.pending != 0U)
        {
            reply("ERR busy; wait for pending TX/STOP or RUN deadline\r\n");
            return;
        }
        if ((initialized_mask & (1U << index)) == 0U)
        {
            reply("ERR INIT %u required first\r\n", (unsigned)address);
            return;
        }
        result = emm_v5_set_velocity((uint8_t)address, (int16_t)rpm,
                                    CHASSIS_MOTOR_ACCEL, 0U);
        if (result != HAL_OK)
        {
            reply("ERR RUN %u status=%u; not queued\r\n",
                  (unsigned)address, (unsigned)result);
            return;
        }
        run_index = (uint8_t)index;
        run_rpm = (int16_t)rpm;
        run_duration_ms = duration;
        run_started_ms = HAL_GetTick();
        run_active = 1U;
        link_stats.valid_frames++;
        reply("OK RUN %u %d %lu QUEUED; auto-stop timer started; no motor feedback\r\n",
              (unsigned)address, (int)rpm, (unsigned long)duration);
        return;
    }
    link_stats.bad_frames++;
    reply("ERR command/arguments; use HELP\r\n");
}

static void parser_feed(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (discard_line == 0U && line_used != 0U)
        {
            command_line[line_used] = '\0';
            process_line();
        }
        line_used = 0U;
        discard_line = 0U;
        return;
    }
    if (discard_line != 0U)
        return;
    if ((byte < 0x20U && byte != '\t') || byte > 0x7EU)
    {
        discard_line = 1U;
        line_used = 0U;
        link_stats.bad_frames++;
        reply("ERR ASCII text required; discard through newline\r\n");
        return;
    }
    if (line_used >= CONSOLE_LINE_SIZE - 1U)
    {
        discard_line = 1U;
        line_used = 0U;
        link_stats.bad_frames++;
        reply("ERR line too long; discard through newline\r\n");
        return;
    }
    command_line[line_used++] = (char)byte;
}

void rk_link_init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    rx_overruns = 0U;
    uart_errors = 0U;
    rx_recover_pending = 0U;
    recovery_latched = 0U;
    seen_overruns = 0U;
    seen_uart_errors = 0U;
    line_used = 0U;
    discard_line = 0U;
    initialized_mask = 0U;
    run_active = 0U;
    stop_pending_mask = 0U;
    stop_error_reported_mask = 0U;
    seen_motor_failures = 0U;
    seen_stale_motion = 0U;
    seen_motor_fault = 0U;
    reply_head = 0U;
    reply_tail = 0U;
    reply_count = 0U;
    dropped_replies = 0U;
    tx_complete_pending = 0U;
    tx_active = 0U;
    tx_started = 0U;
    memset(&link_stats, 0, sizeof(link_stats));
    start_uart_receive();
}

void rk_link_uart_rx_complete(void)
{
    uint16_t next = (uint16_t)((rx_head + 1U) % CONSOLE_RX_SIZE);
    if (next == rx_tail)
        rx_overruns++;
    else
    {
        rx_ring[rx_head] = rx_byte;
        rx_head = next;
    }
    start_uart_receive();
}

void rk_link_uart_tx_complete(void)
{
    tx_complete_pending = 1U;
}

void rk_link_uart_error(void)
{
    uart_errors++;
    rx_recover_pending = 1U;
}

void rk_link_poll(void)
{
    uint16_t consumed = 0U;
    recover_uart_receive();
    observe_motor_fault();
    service_motor_stop();
    poll_reply_tx();
    while (rx_recover_pending == 0U && rx_tail != rx_head &&
           consumed < CONSOLE_POLL_BYTE_BUDGET)
    {
        uint8_t byte;
        /* INIT may yield while waiting for motor UART completion. Never
         * execute following buffered commands after bytes were lost. */
        if (rx_overruns != seen_overruns || uart_errors != seen_uart_errors)
            break;
        byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % CONSOLE_RX_SIZE);
        parser_feed(byte);
        consumed++;
    }
    recover_uart_receive();
    observe_motor_fault();
    service_motor_stop();
    poll_reply_tx();
}

uint8_t rk_link_is_alive(uint32_t now_ms)
{
    (void)now_ms;
    return 0U; /* Text testing must never supply body-velocity commands. */
}

uint8_t rk_link_get_command(rk_velocity_command_t *command, uint32_t now_ms)
{
    (void)command;
    (void)now_ms;
    return 0U;
}

void rk_link_send_status(const rk_chassis_status_t *status)
{
    (void)status; /* STATUS is requested as text in console mode. */
}

void rk_link_get_stats(rk_link_stats_t *stats)
{
    if (stats == NULL)
        return;
    *stats = link_stats;
    stats->rx_overruns = rx_overruns;
    stats->uart_errors = uart_errors;
}

#else /* MOTOR_SERIAL_TEST_ENABLE == 0: existing RK binary protocol */

/* 接收环形缓冲大小：够容纳突发数据即可（本协议最快 460800 波特率，256B 余量充足） */
#define RK_RX_RING_SIZE 256U

/* ---------------- 接收路径（中断与任务共享） ---------------- */
static uint8_t rx_byte;                     /* 单字节中断接收的落点，每收 1 字节进一次中断 */
static uint8_t rx_ring[RK_RX_RING_SIZE];    /* 环形缓冲：中断写 head，任务读 tail */
static volatile uint16_t rx_head;           /* 写指针（中断上下文改，volatile 防优化） */
static volatile uint16_t rx_tail;           /* 读指针（任务上下文改） */
static volatile uint32_t rx_overruns;       /* 环形缓冲写满导致的丢字节次数 */
static volatile uint32_t uart_errors;       /* USART1 硬件错误次数（ORE/FE 等） */
static volatile uint8_t rx_recover_pending; /* 1 = 接收出错，等 poll 里恢复 */

/* ---------------- 组帧状态机（仅任务上下文） ---------------- */
static uint8_t frame[RK_PROTOCOL_MAX_FRAME_SIZE]; /* 正在拼装的帧缓存 */
static uint16_t frame_used;                       /* 当前已收字节数 */
static uint16_t frame_expected;                   /* 收满帧头后算出的整帧长度（0 = 未知） */
static uint32_t last_parser_byte_ms;              /* 上一字节到达时间，用于帧内 20ms 超时 */

/* ---------------- 指令与会话状态（仅任务上下文） ---------------- */
static rk_velocity_command_t latest_command; /* 最近一次校验通过的速度指令 */
static uint8_t command_valid;                /* 是否收到过有效指令（从未收到 -> 失联） */
static uint8_t sequence_valid;               /* 序号会话是否有效：超时/上位机重启后允许序号归零 */
static uint16_t last_rx_sequence;            /* 上一帧已接受序号 */
static uint16_t tx_sequence;                 /* 状态帧发送序号，每发一帧 +1 */
static uint32_t last_command_ms;             /* 最近有效指令的时间戳（存活判定用） */
static rk_link_stats_t link_stats;           /* 链路统计计数 */

/* 环形索引前进一步（到末尾回绕） */
static uint16_t ring_next(uint16_t index)
{
    return (uint16_t)((index + 1U) % RK_RX_RING_SIZE);
}

/* u32 饱和转换到 u16：统计值上报时防截断出错（封顶 0xFFFF） */
static uint16_t saturate_u16(uint32_t value)
{
    return value > 0xFFFFU ? 0xFFFFU : (uint16_t)value;
}

/* 武装/重新武装“单字节中断接收”；失败说明串口状态异常，交给 poll 恢复 */
static void start_uart_receive(void)
{
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK)
        rx_recover_pending = 1U;
}

/* 串口出错后的恢复流程：中止接收 -> 清溢出标志 -> 复位组帧 -> 重新武装接收 */
static void recover_uart_receive(void)
{
    if (rx_recover_pending == 0U)
        return;

    if (HAL_UART_AbortReceive(&huart1) == HAL_OK)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1); /* 清 ORE，否则后续接收一直失败 */
        rx_recover_pending = 0U;
        frame_used = 0U;
        frame_expected = 0U;
        start_uart_receive();
    }
}

/* 放弃当前正在拼装的帧（帧内超时/坏帧时调用） */
static void parser_reset(void)
{
    frame_used = 0U;
    frame_expected = 0U;
}

/* 一条校验通过的速度指令的业务处理：会话/序号检查 -> 解析字段 -> 记录时间戳 */
static void accept_velocity_command(const uint8_t *payload, uint16_t sequence)
{
    uint16_t delta;
    uint8_t flags;
    uint32_t now = HAL_GetTick();

    /* A restarted RK node begins at sequence zero. Once the previous command
     * stream has timed out, treat the next valid frame as a new session. */
    /* 上位机重启后序号会回到 0；超过 RK_LINK_TIMEOUT_MS 就允许序号重置 */
    if (sequence_valid != 0U &&
        (uint32_t)(now - last_command_ms) > RK_LINK_TIMEOUT_MS)
        sequence_valid = 0U;

    if (sequence_valid != 0U)
    {
        delta = (uint16_t)(sequence - last_rx_sequence);
        if (delta == 0U)
        {
            link_stats.duplicate_frames++; /* 完全相同的序号：重复帧，丢弃 */
            return;
        }
        if (delta >= 0x8000U)
        {
            link_stats.stale_frames++; /* 序号“倒退”（差值超过半圈）：过期帧，丢弃 */
            return;
        }
        link_stats.missing_frames += (uint32_t)delta - 1U; /* 中间跳过的序号计入丢帧 */
    }

    /* payload 布局：vx(i32) vy(i32) yaw(i32) flags(u8) + 3 字节保留 */
    flags = payload[12];
    latest_command.vx_mm_s = rk_protocol_read_i32(payload + 0U);
    latest_command.vy_mm_s = rk_protocol_read_i32(payload + 4U);
    latest_command.yaw_mdeg_s = rk_protocol_read_i32(payload + 8U);
    latest_command.enabled = (flags & RK_CMD_FLAG_ENABLE) != 0U ? 1U : 0U;
    latest_command.emergency_stop =
        (flags & RK_CMD_FLAG_EMERGENCY_STOP) != 0U ? 1U : 0U;
    latest_command.sequence = sequence;

    last_rx_sequence = sequence;
    sequence_valid = 1U;
    command_valid = 1U;
    last_command_ms = now; /* 刷新存活时间戳：chassis 任务据此判断链路超时 */
}

/* 收满一整帧后：验版本/CRC，按消息类型分发 */
static void process_frame(void)
{
    uint16_t payload_length = rk_protocol_read_u16(frame + 6U);
    uint16_t received_crc = rk_protocol_read_u16(frame + 8U + payload_length);
    uint16_t calculated_crc =
        rk_protocol_crc16(frame + 2U, (size_t)(6U + payload_length)); /* 校验范围 ver..payload */
    uint16_t sequence = rk_protocol_read_u16(frame + 4U);

    if (frame[2] != RK_PROTOCOL_VERSION || received_crc != calculated_crc)
    {
        link_stats.bad_frames++; /* 版本不符或 CRC 错误 */
        return;
    }

    link_stats.valid_frames++;
    if (frame[3] == RK_MSG_CMD_VELOCITY &&
        payload_length == RK_CMD_VELOCITY_PAYLOAD_SIZE)
    {
        accept_velocity_command(frame + RK_PROTOCOL_HEADER_SIZE, sequence); /* 唯一支持的下行消息 */
    }
    else
    {
        link_stats.bad_frames++; /* 未知消息类型或长度不符 */
    }
}

/* 逐字节组帧状态机：
 *  状态1：等 0xA5        状态2：等 0x5A（收到 0xA5 就继续等）
 *  之后原样收集，收满 8 字节帧头时解析 payload 长度并算出整帧长度
 *  收满整帧 -> process_frame 校验分发；帧内字节间隔超过 20ms 则弃帧重来 */
static void parser_feed(uint8_t byte, uint32_t now_ms)
{
    uint16_t payload_length;

    /* 帧内字节间隔超时：半截帧作废，防止错帧粘包 */
    if (frame_used != 0U &&
        (uint32_t)(now_ms - last_parser_byte_ms) > RK_FRAME_TIMEOUT_MS)
        parser_reset();
    last_parser_byte_ms = now_ms;

    if (frame_used == 0U)
    {
        if (byte == RK_PROTOCOL_SOF_0)
            frame[frame_used++] = byte; /* 看到第一个帧头字节，开始收 */
        return;
    }

    if (frame_used == 1U)
    {
        if (byte == RK_PROTOCOL_SOF_1)
            frame[frame_used++] = byte; /* 第二个帧头字节匹配 */
        else if (byte != RK_PROTOCOL_SOF_0)
            parser_reset(); /* 不是 5A，也不是新的 A5：重新找帧头 */
        return;
    }

    frame[frame_used++] = byte;
    if (frame_used == RK_PROTOCOL_HEADER_SIZE)
    {
        payload_length = rk_protocol_read_u16(frame + 6U);
        if (payload_length > RK_PROTOCOL_MAX_PAYLOAD)
        {
            link_stats.bad_frames++; /* 长度非法：直接判坏帧 */
            parser_reset();
            return;
        }
        frame_expected = (uint16_t)(RK_PROTOCOL_HEADER_SIZE + payload_length +
                                    RK_PROTOCOL_CRC_SIZE); /* 整帧 = 头 + payload + CRC */
    }

    if (frame_expected != 0U && frame_used == frame_expected)
    {
        process_frame(); /* 收满整帧：校验并分发 */
        parser_reset();
    }
}

/* 初始化：所有状态清零，立即启动 USART1 中断接收（此时调度器可能还没启动） */
void rk_link_init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    rx_overruns = 0U;
    uart_errors = 0U;
    rx_recover_pending = 0U;
    command_valid = 0U;
    sequence_valid = 0U;
    last_rx_sequence = 0U;
    tx_sequence = 0U;
    last_command_ms = 0U;
    last_parser_byte_ms = 0U;
    memset(&latest_command, 0, sizeof(latest_command));
    memset(&link_stats, 0, sizeof(link_stats));
    parser_reset();
    start_uart_receive();
}

/* 中断回调（每收 1 字节触发一次，见 main.c -> HAL_UART_RxCpltCallback）：
 * 把刚收到的字节放进环形缓冲，然后立刻重新武装下一次接收。
 * 注意：这里执行在中断上下文，只做搬移和计数，不做解析。 */
void rk_link_uart_rx_complete(void)
{
    uint16_t next = ring_next(rx_head);

    if (next == rx_tail)
        rx_overruns++; /* 缓冲满（任务消费不及时）：丢弃该字节 */
    else
    {
        rx_ring[rx_head] = rx_byte;
        rx_head = next;
    }
    start_uart_receive(); /* 关键：中断方式收完必须重新挂接收，否则不再进中断 */
}

/* 串口错误回调（ORE/FE/NE 等）：只记数并请求恢复，恢复动作留到任务上下文做 */
void rk_link_uart_error(void)
{
    uart_errors++;
    rx_recover_pending = 1U;
}

/* 任务侧轮询：先兜底恢复串口，再把环形缓冲里的字节逐個喂给组帧状态机 */
void rk_link_poll(void)
{
    recover_uart_receive();

    while (rx_tail != rx_head)
    {
        uint8_t byte = rx_ring[rx_tail];
        rx_tail = ring_next(rx_tail);
        parser_feed(byte, HAL_GetTick());
    }
}

/* 链路存活判定：从未收到过有效帧或超过 RK_LINK_TIMEOUT_MS -> 不存活 */
uint8_t rk_link_is_alive(uint32_t now_ms)
{
    if (command_valid == 0U)
        return 0U;
    return (uint32_t)(now_ms - last_command_ms) <= RK_LINK_TIMEOUT_MS ? 1U : 0U;
}

/* 取最新指令：只有链路存活时才返回 1（防止 chassis 使用过期速度） */
uint8_t rk_link_get_command(rk_velocity_command_t *command, uint32_t now_ms)
{
    if (command == NULL || rk_link_is_alive(now_ms) == 0U)
        return 0U;
    *command = latest_command;
    return 1U;
}

/* 组装并发送底盘状态帧（type=0x81，payload 38 字节）：
 *   +0  u32 uptime_ms          +4  4×i16 四轮 RPM
 *   +12 u16 flags              +14 u16 最近接受序号（无则 0xFFFF）
 *   +16 u32 valid_frames       +20 u32 bad_frames
 *   +24 u16 rx_overruns        +26 u16 uart_errors（两者饱和到 u16）
 *   +28 4×i16 实测 RPM×10    +36 u16 实测值有效掩码
 * 发送用阻塞方式（10ms 超时），失败只记 tx_errors，不重试。 */
void rk_link_send_status(const rk_chassis_status_t *status)
{
    uint8_t tx_frame[RK_PROTOCOL_HEADER_SIZE + RK_STATUS_PAYLOAD_SIZE +
                     RK_PROTOCOL_CRC_SIZE];
    uint8_t *payload;
    uint16_t crc;
    uint32_t i;

    if (status == NULL)
        return;

    tx_frame[0] = RK_PROTOCOL_SOF_0;
    tx_frame[1] = RK_PROTOCOL_SOF_1;
    tx_frame[2] = RK_PROTOCOL_VERSION;
    tx_frame[3] = RK_MSG_CHASSIS_STATUS;
    rk_protocol_write_u16(tx_frame + 4U, tx_sequence++);          /* 发送序号自增 */
    rk_protocol_write_u16(tx_frame + 6U, RK_STATUS_PAYLOAD_SIZE); /* payload 长度固定 38 */

    payload = tx_frame + RK_PROTOCOL_HEADER_SIZE;
    rk_protocol_write_u32(payload + 0U, status->uptime_ms);
    for (i = 0U; i < 4U; ++i)
        rk_protocol_write_i16(payload + 4U + i * 2U, status->wheel_rpm[i]);
    rk_protocol_write_u16(payload + 12U, status->flags);
    rk_protocol_write_u16(payload + 14U,
                          sequence_valid != 0U ? last_rx_sequence : 0xFFFFU); /* 回显已接受序号，供上位机对账 */
    rk_protocol_write_u32(payload + 16U, link_stats.valid_frames);
    rk_protocol_write_u32(payload + 20U, link_stats.bad_frames);
    rk_protocol_write_u16(payload + 24U, saturate_u16(rx_overruns));
    rk_protocol_write_u16(payload + 26U, saturate_u16(uart_errors));
    for (i = 0U; i < 4U; ++i)
        rk_protocol_write_i16(payload + 28U + i * 2U,
                              status->actual_rpm_x10[i]);
    rk_protocol_write_u16(payload + 36U, status->actual_valid_mask);

    crc = rk_protocol_crc16(tx_frame + 2U, 6U + RK_STATUS_PAYLOAD_SIZE); /* 与接收同一套 CRC 范围 */
    rk_protocol_write_u16(tx_frame + 8U + RK_STATUS_PAYLOAD_SIZE, crc);
    if (HAL_UART_Transmit(&huart1, tx_frame, (uint16_t)sizeof(tx_frame), 10U) != HAL_OK)
        link_stats.tx_errors++;
}

/* 拷贝统计：注意 rx_overruns / uart_errors 是 volatile 计数，需单独读出 */
void rk_link_get_stats(rk_link_stats_t *stats)
{
    if (stats == NULL)
        return;
    *stats = link_stats;
    stats->rx_overruns = rx_overruns;
    stats->uart_errors = uart_errors;
}

void rk_link_uart_tx_complete(void)
{
    /* Legacy binary status transmit remains synchronous. */
}

#endif /* MOTOR_SERIAL_TEST_ENABLE */
