#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ================ 全局标定/参数集中配置：改这里适配机械与通信 ================ */

/* Board communication settings. Both links use 8 data bits, no parity, 1 stop bit. */
/* 串口波特率：USART1 连 RK3588S 上位机；USART3 连 EMM V5 电机总线 */
#define RK_UART_BAUDRATE              115200U
#define EMM_UART_BAUDRATE             115200U

/* 电机总线发送：1=队列+USART3 DMA，0=阻塞发送 */
#define EMM_V5_USE_DMA                1U
#define EMM_V5_TX_QUEUE_LENGTH        32U
#define EMM_V5_TX_TIMEOUT_MS          50U
#define EMM_V5_FRAME_GAP_MS            2U
#define EMM_V5_MOTION_MAX_AGE_MS      100U
/* X firmware real-speed feedback: one motor query per period. Four motors
 * therefore complete a feedback round in about 100 ms with the defaults. */
#define EMM_V5_FEEDBACK_ENABLE         1U
#define EMM_V5_FEEDBACK_PERIOD_MS     25U
#define EMM_V5_FEEDBACK_TIMEOUT_MS     8U

/* Official ZDT X_V2 command layout. Position values use 0.1 degree units;
 * speed values use 0.1 RPM units. */
#define EMM_V5_X_V2_PROTOCOL             1U
#define EMM_V5_X_V2_MAX_RPM           3000U
#define EMM_V5_X_V2_CTRL_MODE            1U

/* 1: USART1 text motor test console. 0: RK3588S binary chassis protocol.
 * The console-only limits below are retained for a separate bench build. */
#define MOTOR_SERIAL_TEST_ENABLE      0U
#define MOTOR_TEST_MAX_RPM            60U
#define MOTOR_TEST_MAX_DURATION_MS  3000U
#define MOTOR_TEST_MAX_ANGLE_DEG     720U

/* Kept for legacy EMM V5 builds. X_V2 uses F6 for speed and FD for position. */
#define CHASSIS_VELOCITY_MODE_POSITION  0U
#define CHASSIS_MOTOR_DECEL            10U

/* RK3588S must continuously send a valid command while motion is enabled. */
/* 通信超时：LINK=200ms 没有有效指令就失联停机；FRAME=50ms 帧内字节间隔超时则弃帧 */
#define RK_LINK_TIMEOUT_MS               200U
#define RK_FRAME_TIMEOUT_MS              50U
/* 状态帧回发周期 */
#define RK_STATUS_PERIOD_MS              100U

/* Send one protocol status frame immediately after USART1 is initialized.
 * This makes a reset visible even if a later RTOS object fails to start. */
#define RK_BOOT_STATUS_ENABLE              1U

/* EMM V5 motor addresses on the shared USART3 bus. */
/* 四轴地址（共用一条总线）：0=FL 前左，1=FR 前右，2=RR 后右，3=RL 后左 */
#define CHASSIS_MOTOR_COUNT              4U
#define CHASSIS_MOTOR_ADDR_0             1U
#define CHASSIS_MOTOR_ADDR_1             2U
#define CHASSIS_MOTOR_ADDR_2             3U
#define CHASSIS_MOTOR_ADDR_3             4U

/* EMM V5 operating parameters. Confirm microstep setting on the motor. */
/* 电机参数：MAX_RPM=协议允许上限；SAFE_MAX_RPM=软件限幅（逆解后钳位）
 * ACCEL=速度模式加速度，单位 RPM/s（10 表示到 100 RPM 约需 10 s）；
 * PULSES_PER_REV=每圈脉冲数（当前仅作标定记录，未参与计算） */
#define CHASSIS_MOTOR_MAX_RPM         3000U
#define CHASSIS_SAFE_MAX_RPM          3000U
#define CHASSIS_MOTOR_ACCEL           200U
#define CHASSIS_MOTOR_PULSES_PER_REV  3200U

/* Mechanical values are intentionally centralized for calibration. */
/* 机械参数：轮半径、半轴距(L/2)、半轮距(W/2)，单位 mm（用于麦轮逆解） */
#define CHASSIS_WHEEL_RADIUS_MM       30.0f
#define CHASSIS_HALF_LENGTH_MM       150.0f
#define CHASSIS_HALF_WIDTH_MM        150.0f

/* Set to 1 for the default mecanum body-velocity conversion. */
/* 1=启用麦轮逆解；0=禁用（任何速度请求都会转成停机） */
#define CHASSIS_ENABLE_BODY_KINEMATICS  1U

/* Motor order is front-left, front-right, rear-right, rear-left.
 * Change signs after a one-wheel direction test on the assembled chassis. */
/* 方向符号：装车后逐轮测试，若该轮转向反了就把对应 SIGN 改成 -1 */
#define CHASSIS_DIR_SIGN_0               1
#define CHASSIS_DIR_SIGN_1               1
#define CHASSIS_DIR_SIGN_2               1
#define CHASSIS_DIR_SIGN_3               1

#endif /* APP_CONFIG_H */
