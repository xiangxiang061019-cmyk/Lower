#ifndef RK_LINK_H
#define RK_LINK_H

#include <stdint.h>

/* ================= USART1 link (RK_UART_BAUDRATE, currently 115200 8N1) =================
 * MOTOR_SERIAL_TEST_ENABLE=1: newline-terminated ASCII single-motor test console.
 * Commands: HELP, INIT <addr>, RUN <addr> <signed_rpm> <ms>, ANGLE <addr> <signed_deg> <rpm>,
 * STOP <addr|ALL>, STATUS.
 * RUN requires INIT, permits only one timed run, and is limited by app_config.h.
 * ANGLE sends an X-firmware FD relative-to-current-position command.
 * Replies distinguish QUEUED / UART TX OK from the periodic actual-speed
 * feedback reported in the binary status frame.
 * MOTOR_SERIAL_TEST_ENABLE=0: preserve the binary RK protocol described below.
 * 职责：
 *  1. 中断逐字节接收 -> 256B 环形缓冲
 *  2. 任务上下文组帧、CRC16 校验、序号去重/丢帧统计
 *  3. 维护“最新速度指令”与链路存活判定（RK_LINK_TIMEOUT_MS 无有效帧 = 失联）
 *  4. 周期回发底盘状态帧（100ms 一次，由 chassis 任务触发）
 * ============================================================================== */

/* 上位机下发的速度指令（一帧校验通过后的解析结果） */
typedef struct
{
    int32_t vx_mm_s;        /* 车体前向速度，mm/s（正 = 向前） */
    int32_t vy_mm_s;        /* 车体左向速度，mm/s（正 = 向左） */
    int32_t yaw_mdeg_s;     /* 逆时针角速度，千分之一度/秒（÷1000 = deg/s） */
    uint16_t sequence;      /* 本帧序号，用于去重与丢帧统计 */
    uint8_t enabled;        /* 使能位：0 = 上位机要求停止 */
    uint8_t emergency_stop; /* 急停位：1 = 立即停机 */
} rk_velocity_command_t;

/* 上报给上位机的底盘状态 */
typedef struct
{
    uint32_t uptime_ms;     /* 下位机运行时间，ms */
    int16_t wheel_rpm[4];   /* 四轮目标/已下发 RPM（顺序 FL/FR/RR/RL） */
    int16_t actual_rpm_x10[4]; /* X firmware feedback, signed 0.1 RPM */
    uint16_t actual_valid_mask; /* bit i = actual_rpm_x10[i] is valid */
    uint16_t flags;         /* RK_STATUS_FLAG_* 位组合 */
} rk_chassis_status_t;

/* 链路统计（诊断用，部分字段随状态帧上报） */
typedef struct
{
    uint32_t valid_frames;      /* 校验通过的有效帧数 */
    uint32_t bad_frames;        /* CRC/版本错误或类型/长度非法的帧数 */
    uint32_t missing_frames;    /* 由序号差推断的丢失帧数 */
    uint32_t duplicate_frames;  /* 重复序号帧数 */
    uint32_t stale_frames;      /* 序号倒退（过期）帧数 */
    uint32_t rx_overruns;       /* 环形缓冲写满导致的丢字节次数 */
    uint32_t uart_errors;       /* USART1 硬件错误次数（ORE/FE 等） */
    uint32_t tx_errors;         /* 状态帧发送失败次数 */
} rk_link_stats_t;

/* 初始化链路状态并启动 USART1 单字节中断接收（在调度器启动前调用也可以） */
void rk_link_init(void);

/* Call from one task every 1 ms. Console mode checks automatic stop deadlines,
 * consumes at most 64 RX bytes per call, and services asynchronous USART1 replies. */
void rk_link_poll(void);

/* 取最新速度指令；链路失联（超过 RK_LINK_TIMEOUT_MS）时返回 0，表示数据不可信 */
uint8_t rk_link_get_command(rk_velocity_command_t *command, uint32_t now_ms);

/* 链路是否存活（从未收到过或超时都返回 0） */
uint8_t rk_link_is_alive(uint32_t now_ms);

/* Binary mode: status frame via blocking USART1 TX (10 ms timeout).
 * Console mode: no-op; use the text STATUS command instead. */
void rk_link_send_status(const rk_chassis_status_t *status);

/* 拷贝一份链路统计（留给调试/诊断接口用） */
void rk_link_get_stats(rk_link_stats_t *stats);

/* 由 USART1 的 HAL 中断回调调用（见 main.c 的 HAL_UART_RxCpltCallback / ErrorCallback） */
void rk_link_uart_rx_complete(void); /* 每收到 1 字节：存入环形缓冲并重新武装接收 */
void rk_link_uart_tx_complete(void); /* USART1 TX complete: flag only, consume in poll */
void rk_link_uart_error(void);       /* 串口错误：置恢复标志，由 poll 兜底恢复 */

#endif /* RK_LINK_H */
