#include "rescue_protocol.h"

/*
 * CRC-16/CCITT-FALSE 参数：初值 0xFFFF、多项式 0x1021、无反射、异或值 0。
 * CRC 覆盖 version 到 payload，不包括帧头、CRC 自身和固定帧尾。
 */
static uint16_t Rescue_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                    ? (uint16_t)((crc << 1) ^ 0x1021U)
                    : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

int16_t Rescue_ReadI16(const uint8_t *data)
{
    uint16_t raw = Rescue_ReadU16(data);
    return raw < 0x8000U ? (int16_t)raw
                         : (int16_t)((int32_t)raw - 65536);
}

uint16_t Rescue_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void Rescue_WriteI16(uint8_t *data, int16_t value)
{
    Rescue_WriteU16(data, (uint16_t)value);
}

void Rescue_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

size_t Rescue_Encode(const RescueFrame *frame, uint8_t *output,
                     size_t capacity)
{
    size_t total;
    size_t i;
    uint16_t crc;
    uint8_t payload_length;

    if (frame == NULL || output == NULL ||
        frame->length > RESCUE_MAX_PAYLOAD) {
        return 0U;
    }

    payload_length = frame->length;
    total = RESCUE_FRAME_OVERHEAD + payload_length;
    if (capacity < total) {
        return 0U;
    }

    output[0] = RESCUE_FRAME_HEAD_0;
    output[1] = RESCUE_FRAME_HEAD_1;
    output[2] = RESCUE_PROTOCOL_VERSION;
    output[3] = frame->command;
    output[4] = frame->sequence;
    output[5] = payload_length;
    for (i = 0U; i < payload_length; ++i) {
        output[6U + i] = frame->payload[i];
    }

    crc = Rescue_Crc16(&output[2], 4U + payload_length);
    output[6U + payload_length] = (uint8_t)crc;
    output[7U + payload_length] = (uint8_t)(crc >> 8);
    output[8U + payload_length] = RESCUE_FRAME_TAIL_0;
    output[9U + payload_length] = RESCUE_FRAME_TAIL_1;
    return total;
}

void Rescue_ParserInit(RescueParser *parser)
{
    if (parser != NULL) {
        parser->count = 0U;
        parser->expected = 0U;
    }
}

/*
 * 错误后重新找帧头。若当前字节是 AA，就把它保留为新帧的第一个字节，
 * 这样相邻帧或损坏帧后的帧可以尽快重新同步。
 */
static void Rescue_ParserRestart(RescueParser *parser, uint8_t byte)
{
    parser->count = 0U;
    parser->expected = 0U;
    if (byte == RESCUE_FRAME_HEAD_0) {
        parser->bytes[0] = byte;
        parser->count = 1U;
    }
}

RescueParseResult Rescue_ParserFeed(RescueParser *parser, uint8_t byte,
                                    RescueFrame *out)
{
    uint8_t length;
    uint8_t i;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (parser == NULL || out == NULL) {
        return RESCUE_PARSE_ERROR;
    }

    if (parser->count == 0U) {
        if (byte == RESCUE_FRAME_HEAD_0) {
            parser->bytes[parser->count++] = byte;
        }
        return RESCUE_PARSE_WAIT;
    }
    if (parser->count == 1U) {
        if (byte == RESCUE_FRAME_HEAD_1) {
            parser->bytes[parser->count++] = byte;
        } else {
            Rescue_ParserRestart(parser, byte);
        }
        return RESCUE_PARSE_WAIT;
    }

    /* 最大帧长度由 length 检查保证，写入前不会超过固定缓冲区。 */
    parser->bytes[parser->count++] = byte;

    /* 收到 length 字段后，已经可以算出 CRC 与帧尾所在的精确位置。 */
    if (parser->count == 6U) {
        length = parser->bytes[5];
        if (parser->bytes[2] != RESCUE_PROTOCOL_VERSION ||
            length > RESCUE_MAX_PAYLOAD) {
            Rescue_ParserRestart(parser, byte);
            return RESCUE_PARSE_ERROR;
        }
        parser->expected = (uint8_t)(RESCUE_FRAME_OVERHEAD + length);
    }

    if (parser->expected == 0U || parser->count < parser->expected) {
        return RESCUE_PARSE_WAIT;
    }

    length = parser->bytes[5];
    received_crc = (uint16_t)parser->bytes[6U + length] |
                   ((uint16_t)parser->bytes[7U + length] << 8);
    calculated_crc = Rescue_Crc16(&parser->bytes[2], 4U + length);
    if (received_crc != calculated_crc ||
        parser->bytes[8U + length] != RESCUE_FRAME_TAIL_0 ||
        parser->bytes[9U + length] != RESCUE_FRAME_TAIL_1) {
        Rescue_ParserRestart(parser, byte);
        return RESCUE_PARSE_ERROR;
    }

    out->command = parser->bytes[3];
    out->sequence = parser->bytes[4];
    out->length = length;
    for (i = 0U; i < length; ++i) {
        out->payload[i] = parser->bytes[6U + i];
    }

    /* 完整帧已经交给调用者，最后一个帧尾字节不参与下一帧同步。 */
    Rescue_ParserRestart(parser, 0U);
    return RESCUE_PARSE_FRAME;
}
