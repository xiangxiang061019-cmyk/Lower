#include "rk_protocol.h"

/* CRC-16/CCITT-FALSE：poly=0x1021、init=0xFFFF、输入不反转、输出不异或。
 * 逐位实现（不查表），上位机端必须用同一算法生成校验值才能互通。 */
uint16_t rk_protocol_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU; /* 初值 */
    size_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8U; /* 当前字节异或进高 8 位 */
        for (bit = 0U; bit < 8U; ++bit)
            crc = (uint16_t)((crc & 0x8000U) != 0U
                                 ? (crc << 1U) ^ 0x1021U /* 最高位为 1：左移后异或多项式 */
                                 : crc << 1U);           /* 最高位为 0：仅左移 */
    }
    return crc;
}

/* ---- 以下均为小端（低字节在前）读写，与帧格式约定一致 ---- */

uint16_t rk_protocol_read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

int16_t rk_protocol_read_i16(const uint8_t *data)
{
    return (int16_t)rk_protocol_read_u16(data);
}

uint32_t rk_protocol_read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

int32_t rk_protocol_read_i32(const uint8_t *data)
{
    return (int32_t)rk_protocol_read_u32(data);
}

void rk_protocol_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;        /* 低字节在前 */
    data[1] = (uint8_t)(value >> 8U);
}

void rk_protocol_write_i16(uint8_t *data, int16_t value)
{
    rk_protocol_write_u16(data, (uint16_t)value);
}

void rk_protocol_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;        /* 低字节在前 */
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

void rk_protocol_write_i32(uint8_t *data, int32_t value)
{
    rk_protocol_write_u32(data, (uint32_t)value);
}
