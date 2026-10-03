#ifndef RK_PROTOCOL_H
#define RK_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/* ==================== RK 私有协议（与 RK3588S 上位机约定） ====================
 * 帧结构（多字节字段一律小端）：
 *   偏移    0     1     2      3      4..5     6..7     8..       末尾2字节
 *   内容   0xA5  0x5A  ver    type   seq(L)   len(L)  payload(L)  crc16(L)
 * 说明：
 *   - ver  : 协议版本，收帧时必须等于 RK_PROTOCOL_VERSION，否则判坏帧
 *   - type : 消息类型，下行速度指令=0x01，上行底盘状态=0x81
 *   - seq  : 帧序号，用于去重（相同丢弃）与丢帧统计
 *   - len  : payload 字节数，超过 MAX_PAYLOAD 判错
 *   - crc16: 校验范围 = byte2(ver) 到 payload 最后一字节
 * ========================================================================== */

#define RK_PROTOCOL_SOF_0              0xA5U  /* 帧头第 1 字节 */
#define RK_PROTOCOL_SOF_1              0x5AU  /* 帧头第 2 字节 */
#define RK_PROTOCOL_VERSION            0x01U  /* 协议版本 */
#define RK_PROTOCOL_HEADER_SIZE           8U  /* 帧头长度：SOF(2)+ver+type+seq(2)+len(2) */
#define RK_PROTOCOL_CRC_SIZE              2U  /* 帧尾 CRC16 长度 */
#define RK_PROTOCOL_MAX_PAYLOAD           48U  /* 单帧 payload 上限 */
#define RK_PROTOCOL_MAX_FRAME_SIZE \
    (RK_PROTOCOL_HEADER_SIZE + RK_PROTOCOL_MAX_PAYLOAD + RK_PROTOCOL_CRC_SIZE)

#define RK_MSG_CMD_VELOCITY            0x01U  /* (下行) 速度指令帧 */
#define RK_MSG_CHASSIS_STATUS          0x81U  /* (上行) 底盘状态帧 */

#define RK_CMD_VELOCITY_PAYLOAD_SIZE     16U  /* 速度指令 payload 固定 16 字节 */
#define RK_STATUS_PAYLOAD_SIZE            38U  /* legacy 28B + 4x i16 actual RPMx10 + mask */

#define RK_CMD_FLAG_ENABLE             0x01U  /* 指令 flags bit0：允许底盘运动 */
#define RK_CMD_FLAG_EMERGENCY_STOP     0x02U  /* 指令 flags bit1：急停 */

/* 底盘状态 flags 位（回报给上位机，供显示/联锁判断） */
#define RK_STATUS_FLAG_LINK_ALIVE      0x0001U  /* 150ms 内收到过有效指令 */
#define RK_STATUS_FLAG_ENABLED         0x0002U  /* 上位机使能位为 1 */
#define RK_STATUS_FLAG_EMERGENCY_STOP  0x0004U  /* 收到急停指令 */
#define RK_STATUS_FLAG_SAFE_STOPPED    0x0008U  /* 已执行安全停机 */
#define RK_STATUS_FLAG_MOTOR_READY     0x0010U  /* 电机初始化成功 */

/* CRC16：多项式 0x1021、初值 0xFFFF（CRC-16/CCITT-FALSE，不反转、无末尾异或） */
uint16_t rk_protocol_crc16(const uint8_t *data, size_t length);

/* 以下读写函数均按“小端”解释/输出多字节字段 */
uint16_t rk_protocol_read_u16(const uint8_t *data);
int16_t rk_protocol_read_i16(const uint8_t *data);
uint32_t rk_protocol_read_u32(const uint8_t *data);
int32_t rk_protocol_read_i32(const uint8_t *data);
void rk_protocol_write_u16(uint8_t *data, uint16_t value);
void rk_protocol_write_i16(uint8_t *data, int16_t value);
void rk_protocol_write_u32(uint8_t *data, uint32_t value);
void rk_protocol_write_i32(uint8_t *data, int32_t value);

#endif /* RK_PROTOCOL_H */
