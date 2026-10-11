/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : VT link monitor demo (USART6 RX + USART1 debug TX)
 ******************************************************************************
 * @attention
 *
 * 本测试用于隔离排查图传链路问题：
 * - USART6: 接收 VT03/VT13 21B 遥控帧（921600 8N1）
 * - USART1: 输出调试文本
 *
 * 输出格式对齐 scripts/vt_serial_monitor.py：
 * [LINK] / [STAT] / [DATA] ...
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp_init.h"
#include "bsp_usart.h"

/* Private define ------------------------------------------------------------*/
#define VT_FRAME_SIZE 21u
#define VT_HEADER_0 0xA9u
#define VT_HEADER_1 0x53u

#define VT_RX_CHUNK_SIZE 64u
#define VT_STREAM_BUFFER_SIZE 512u

#define LINK_TIMEOUT_MS 200u
#define REPORT_INTERVAL_MS 500u

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t total_frames;
    uint32_t valid_frames;
    uint32_t invalid_frames;
    uint32_t bytes_in;
    uint32_t resync_drop_bytes;
    uint32_t header_frames;
    uint32_t last_valid_tick;
    uint32_t first_valid_tick;
} LinkStats_t;

typedef struct
{
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t dial;

    int16_t ch0_c;
    int16_t ch1_c;
    int16_t ch2_c;
    int16_t ch3_c;
    int16_t dial_c;

    uint8_t gear;
    uint8_t pause;
    uint8_t custom_l;
    uint8_t custom_r;
    uint8_t trigger;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint8_t mouse_m;

    uint16_t keyboard;
} ParsedFrame_t;

typedef struct
{
    uint8_t captured;
    uint8_t frame[VT_FRAME_SIZE];
    uint16_t calc_with_header;
    uint16_t calc_reflected;
    uint16_t calc_without_header;
    uint16_t rx_le;
    uint16_t rx_be;
} CRCProbe_t;

/* Private variables ---------------------------------------------------------*/
static USARTInstance *vt_rx_usart = NULL;

static uint8_t stream_buf[VT_STREAM_BUFFER_SIZE];
static uint16_t stream_len = 0u;

static volatile LinkStats_t link_stats;
static volatile uint8_t link_online = 0u;
static volatile uint8_t link_online_rise_event = 0u;
static volatile uint8_t has_last_parsed = 0u;
static volatile ParsedFrame_t last_parsed;
static volatile CRCProbe_t crc_probe;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void Debug_DisableWatchdogs(void);
static void ReinitUartForVT(UART_HandleTypeDef *uart_handle);

static void Uart1SendBuffer(const uint8_t *buffer, uint16_t len);
static void Uart1SendString(const char *str);
static void AppendFormat(char *buffer, size_t buffer_size, size_t *offset, const char *fmt, ...);

static uint16_t CRC16_CCITT_FALSE(const uint8_t *data, uint16_t len);
static uint16_t CRC16_CCITT_REFLECTED(const uint8_t *data, uint16_t len);
static uint8_t VerifyFrame(const uint8_t *frame);
static uint16_t GetBitsU16(const uint8_t *buf, uint16_t start_bit, uint8_t bit_len);

static void StreamPush(const uint8_t *data, uint16_t len);
static void StreamConsume(uint16_t n);
static void ParseFrame(const uint8_t *frame, ParsedFrame_t *out);
static void ProcessRxStream(void);
static void VTRxCallback(void);

static const char *GearName(uint8_t gear);
static void BuildKeyList(uint16_t keyboard, char *out, size_t out_size);
static int32_t ChannelPercentX100(int32_t raw);
static void AppendSignedFixed2(char *buffer, size_t buffer_size, size_t *offset, int32_t value_x100);
static void AppendUnsignedFixed1(char *buffer, size_t buffer_size, size_t *offset, uint32_t value_x10);
static void AppendMsAsSec3(char *buffer, size_t buffer_size, size_t *offset, uint32_t ms);

static void PrintLinkOnline(uint32_t now_tick);
static void PrintLinkOffline(void);
static void PrintStatAndData(uint32_t now_tick);
static void PrintCRCProbeOnce(void);

static void SnapshotState(LinkStats_t *stats, ParsedFrame_t *parsed, uint8_t *parsed_valid, uint8_t *online);

/* Private user code ---------------------------------------------------------*/
static const char *kKeyNames[16] = {
    "W", "S", "A", "D", "SHIFT", "CTRL", "Q", "E",
    "R", "F", "G", "Z", "X", "C", "V", "B"};

static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static void ReinitUartForVT(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL)
        return;

    (void)HAL_UART_DeInit(uart_handle);
    uart_handle->Init.BaudRate = 921600;
    uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle->Init.StopBits = UART_STOPBITS_1;
    uart_handle->Init.Parity = UART_PARITY_NONE;
    uart_handle->Init.Mode = UART_MODE_TX_RX;
    uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart_handle) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Uart1SendBuffer(const uint8_t *buffer, uint16_t len)
{
    if (buffer == NULL || len == 0u)
        return;

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, 100);
}

static void Uart1SendString(const char *str)
{
    if (str == NULL)
        return;
    Uart1SendBuffer((const uint8_t *)str, (uint16_t)strlen(str));
}

static void AppendFormat(char *buffer, size_t buffer_size, size_t *offset, const char *fmt, ...)
{
    if (*offset >= buffer_size)
        return;

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(buffer + *offset, buffer_size - *offset, fmt, args);
    va_end(args);

    if (written < 0)
        return;

    size_t written_sz = (size_t)written;
    if (written_sz >= buffer_size - *offset)
    {
        *offset = buffer_size - 1u;
        buffer[*offset] = '\0';
    }
    else
    {
        *offset += written_sz;
    }
}

static uint16_t CRC16_CCITT_FALSE(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0u; bit < 8u; ++bit)
        {
            if (crc & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else
                crc <<= 1;
        }
    }
    return crc;
}

static uint16_t CRC16_CCITT_REFLECTED(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < len; ++i)
    {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit)
        {
            if (crc & 0x0001u)
                crc = (uint16_t)((crc >> 1) ^ 0x8408u);
            else
                crc >>= 1;
        }
    }
    return crc;
}

static uint8_t VerifyFrame(const uint8_t *frame)
{
    if (frame[0] != VT_HEADER_0 || frame[1] != VT_HEADER_1)
        return 0u;

    uint16_t calc_false = CRC16_CCITT_FALSE(frame, VT_FRAME_SIZE - 2u);
    uint16_t calc_reflected = CRC16_CCITT_REFLECTED(frame, VT_FRAME_SIZE - 2u);
    uint16_t rx_le = (uint16_t)frame[VT_FRAME_SIZE - 2u] | ((uint16_t)frame[VT_FRAME_SIZE - 1u] << 8);
    uint16_t rx_be = (uint16_t)frame[VT_FRAME_SIZE - 1u] | ((uint16_t)frame[VT_FRAME_SIZE - 2u] << 8);
    if (calc_false == rx_le || calc_false == rx_be)
        return 1u;
    if (calc_reflected == rx_le || calc_reflected == rx_be)
        return 1u;
    return 0u;
}

static uint16_t GetBitsU16(const uint8_t *buf, uint16_t start_bit, uint8_t bit_len)
{
    uint16_t out = 0u;
    for (uint8_t i = 0u; i < bit_len; ++i)
    {
        uint16_t bit_index = (uint16_t)(start_bit + i);
        uint16_t byte_index = (uint16_t)(bit_index >> 3);
        uint8_t bit_in_byte = (uint8_t)(bit_index & 0x07u);
        uint8_t bit = (uint8_t)((buf[byte_index] >> bit_in_byte) & 0x01u);
        out |= (uint16_t)bit << i;
    }
    return out;
}

static void StreamPush(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
        return;

    if (len >= VT_STREAM_BUFFER_SIZE)
    {
        data += (len - VT_STREAM_BUFFER_SIZE);
        len = VT_STREAM_BUFFER_SIZE;
        stream_len = 0u;
    }

    if ((uint16_t)(stream_len + len) > VT_STREAM_BUFFER_SIZE)
    {
        uint16_t overflow = (uint16_t)(stream_len + len - VT_STREAM_BUFFER_SIZE);
        if (overflow >= stream_len)
        {
            stream_len = 0u;
        }
        else
        {
            memmove(stream_buf, stream_buf + overflow, (size_t)(stream_len - overflow));
            stream_len = (uint16_t)(stream_len - overflow);
        }
    }

    memcpy(stream_buf + stream_len, data, len);
    stream_len = (uint16_t)(stream_len + len);
}

static void StreamConsume(uint16_t n)
{
    if (n == 0u || stream_len == 0u)
        return;

    if (n >= stream_len)
    {
        stream_len = 0u;
        return;
    }

    memmove(stream_buf, stream_buf + n, (size_t)(stream_len - n));
    stream_len = (uint16_t)(stream_len - n);
}

static void ParseFrame(const uint8_t *frame, ParsedFrame_t *out)
{
    if (out == NULL)
        return;

    out->ch0 = GetBitsU16(frame, 16u, 11u);
    out->ch1 = GetBitsU16(frame, 27u, 11u);
    out->ch2 = GetBitsU16(frame, 38u, 11u);
    out->ch3 = GetBitsU16(frame, 49u, 11u);
    out->dial = GetBitsU16(frame, 65u, 11u);

    out->ch0_c = (int16_t)out->ch0 - 1024;
    out->ch1_c = (int16_t)out->ch1 - 1024;
    out->ch2_c = (int16_t)out->ch2 - 1024;
    out->ch3_c = (int16_t)out->ch3 - 1024;
    out->dial_c = (int16_t)out->dial - 1024;

    out->gear = (uint8_t)GetBitsU16(frame, 60u, 2u);
    out->pause = (uint8_t)GetBitsU16(frame, 62u, 1u);
    out->custom_l = (uint8_t)GetBitsU16(frame, 63u, 1u);
    out->custom_r = (uint8_t)GetBitsU16(frame, 64u, 1u);
    out->trigger = (uint8_t)GetBitsU16(frame, 76u, 1u);

    out->mouse_x = (int16_t)GetBitsU16(frame, 80u, 16u);
    out->mouse_y = (int16_t)GetBitsU16(frame, 96u, 16u);
    out->mouse_z = (int16_t)GetBitsU16(frame, 112u, 16u);

    out->mouse_l = (GetBitsU16(frame, 128u, 2u) == 1u) ? 1u : 0u;
    out->mouse_r = (GetBitsU16(frame, 130u, 2u) == 1u) ? 1u : 0u;
    out->mouse_m = (GetBitsU16(frame, 132u, 2u) == 1u) ? 1u : 0u;

    out->keyboard = GetBitsU16(frame, 136u, 16u);
}

static void ProcessRxStream(void)
{
    while (stream_len >= 2u)
    {
        if (stream_buf[0] != VT_HEADER_0 || stream_buf[1] != VT_HEADER_1)
        {
            uint16_t header_pos = stream_len;
            for (uint16_t i = 1u; i + 1u < stream_len; ++i)
            {
                if (stream_buf[i] == VT_HEADER_0 && stream_buf[i + 1u] == VT_HEADER_1)
                {
                    header_pos = i;
                    break;
                }
            }

            if (header_pos >= stream_len)
            {
                uint16_t drop = (uint16_t)(stream_len - 1u);
                link_stats.resync_drop_bytes += drop;
                StreamConsume(drop);
            }
            else if (header_pos > 0u)
            {
                link_stats.resync_drop_bytes += header_pos;
                StreamConsume(header_pos);
            }
            continue;
        }

        if (stream_len < VT_FRAME_SIZE)
            break;

        link_stats.header_frames++;

        if (VerifyFrame(stream_buf))
        {
            ParsedFrame_t parsed;
            ParseFrame(stream_buf, &parsed);

            link_stats.total_frames++;
            link_stats.valid_frames++;
            link_stats.last_valid_tick = HAL_GetTick();
            if (link_stats.first_valid_tick == 0u)
                link_stats.first_valid_tick = link_stats.last_valid_tick;

            last_parsed = parsed;
            has_last_parsed = 1u;

            if (!link_online)
            {
                link_online = 1u;
                link_online_rise_event = 1u;
            }

            StreamConsume(VT_FRAME_SIZE);
        }
        else
        {
            link_stats.total_frames++;
            link_stats.invalid_frames++;
            link_stats.resync_drop_bytes++;

            memcpy((void *)crc_probe.frame, stream_buf, VT_FRAME_SIZE);
            crc_probe.calc_with_header = CRC16_CCITT_FALSE(stream_buf, VT_FRAME_SIZE - 2u);
            crc_probe.calc_reflected = CRC16_CCITT_REFLECTED(stream_buf, VT_FRAME_SIZE - 2u);
            crc_probe.calc_without_header = CRC16_CCITT_FALSE(stream_buf + 2u, VT_FRAME_SIZE - 4u);
            crc_probe.rx_le = (uint16_t)stream_buf[VT_FRAME_SIZE - 2u] | ((uint16_t)stream_buf[VT_FRAME_SIZE - 1u] << 8);
            crc_probe.rx_be = (uint16_t)stream_buf[VT_FRAME_SIZE - 1u] | ((uint16_t)stream_buf[VT_FRAME_SIZE - 2u] << 8);
            crc_probe.captured = 1u;
            StreamConsume(1u);
        }
    }
}

static void VTRxCallback(void)
{
    if (vt_rx_usart == NULL)
        return;

    uint16_t len = vt_rx_usart->recv_len;
    if (len == 0u)
        return;

    link_stats.bytes_in += len;
    StreamPush(vt_rx_usart->recv_buff, len);
    ProcessRxStream();
}

static const char *GearName(uint8_t gear)
{
    switch (gear)
    {
    case 0u:
        return "C";
    case 1u:
        return "N";
    case 2u:
        return "S";
    default:
        return "UNK";
    }
}

static void BuildKeyList(uint16_t keyboard, char *out, size_t out_size)
{
    if (out == NULL || out_size == 0u)
        return;

    size_t off = 0u;
    bool any = false;
    for (uint8_t i = 0u; i < 16u; ++i)
    {
        if ((keyboard >> i) & 0x01u)
        {
            const char *name = kKeyNames[i];
            if (any && off < out_size - 1u)
                out[off++] = ',';

            size_t nlen = strlen(name);
            if (off + nlen >= out_size)
                break;
            memcpy(out + off, name, nlen);
            off += nlen;
            any = true;
        }
    }

    if (!any)
    {
        if (out_size >= 2u)
        {
            out[0] = '-';
            out[1] = '\0';
            return;
        }
    }

    if (off < out_size)
        out[off] = '\0';
    else
        out[out_size - 1u] = '\0';
}

static int32_t ChannelPercentX100(int32_t raw)
{
    return (raw - 1024) * 10000 / 660;
}

static void AppendSignedFixed2(char *buffer, size_t buffer_size, size_t *offset, int32_t value_x100)
{
    char sign = '+';
    uint32_t abs_v;

    if (value_x100 < 0)
    {
        sign = '-';
        abs_v = (uint32_t)(-value_x100);
    }
    else
    {
        abs_v = (uint32_t)value_x100;
    }

    AppendFormat(buffer, buffer_size, offset, "%c%lu.%02lu", sign,
                 (unsigned long)(abs_v / 100u),
                 (unsigned long)(abs_v % 100u));
}

static void AppendUnsignedFixed1(char *buffer, size_t buffer_size, size_t *offset, uint32_t value_x10)
{
    AppendFormat(buffer, buffer_size, offset, "%lu.%01lu",
                 (unsigned long)(value_x10 / 10u),
                 (unsigned long)(value_x10 % 10u));
}

static void AppendMsAsSec3(char *buffer, size_t buffer_size, size_t *offset, uint32_t ms)
{
    AppendFormat(buffer, buffer_size, offset, "%lu.%03lu",
                 (unsigned long)(ms / 1000u),
                 (unsigned long)(ms % 1000u));
}

static void PrintLinkOnline(uint32_t now_tick)
{
    char line[96];
    size_t off = 0u;
    AppendFormat(line, sizeof(line), &off, "[LINK] ONLINE (t=");
    AppendMsAsSec3(line, sizeof(line), &off, now_tick);
    AppendFormat(line, sizeof(line), &off, ")\r\n");
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)off);
}

static void PrintLinkOffline(void)
{
    char line[96];
    size_t off = 0u;
    AppendFormat(line, sizeof(line), &off, "[LINK] OFFLINE (>");
    AppendMsAsSec3(line, sizeof(line), &off, LINK_TIMEOUT_MS);
    AppendFormat(line, sizeof(line), &off, "s 无有效帧)\r\n");
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)off);
}

static void SnapshotState(LinkStats_t *stats, ParsedFrame_t *parsed, uint8_t *parsed_valid, uint8_t *online)
{
    __disable_irq();
    *stats = link_stats;
    *parsed_valid = has_last_parsed;
    if (*parsed_valid)
        *parsed = last_parsed;
    *online = link_online;
    __enable_irq();
}

static void PrintStatAndData(uint32_t now_tick)
{
    LinkStats_t stats;
    ParsedFrame_t parsed;
    uint8_t parsed_valid = 0u;
    uint8_t online = 0u;
    SnapshotState(&stats, &parsed, &parsed_valid, &online);

    uint32_t crc_ok_x10 = 0u;
    uint32_t fps_x10 = 0u;
    if (stats.total_frames > 0u)
        crc_ok_x10 = (stats.valid_frames * 1000u) / stats.total_frames;

    if (stats.first_valid_tick > 0u && now_tick > stats.first_valid_tick)
    {
        uint32_t dt_ms = now_tick - stats.first_valid_tick;
        fps_x10 = (stats.valid_frames * 10000u) / dt_ms;
    }

    char stat_line[320];
    size_t off = 0u;
    AppendFormat(stat_line, sizeof(stat_line), &off,
                 "[STAT] online=%u valid=%lu invalid=%lu header=%lu crc_ok=",
                 (unsigned int)(online ? 1u : 0u),
                 (unsigned long)stats.valid_frames,
                 (unsigned long)stats.invalid_frames,
                 (unsigned long)stats.header_frames);
    AppendUnsignedFixed1(stat_line, sizeof(stat_line), &off, crc_ok_x10);
    AppendFormat(stat_line, sizeof(stat_line), &off, "%% fps=");
    AppendUnsignedFixed1(stat_line, sizeof(stat_line), &off, fps_x10);
    AppendFormat(stat_line, sizeof(stat_line), &off, " last_valid_age=");

    if (stats.last_valid_tick > 0u)
    {
        uint32_t age_ms = now_tick - stats.last_valid_tick;
        AppendMsAsSec3(stat_line, sizeof(stat_line), &off, age_ms);
        AppendFormat(stat_line, sizeof(stat_line), &off, "s");
    }
    else
    {
        AppendFormat(stat_line, sizeof(stat_line), &off, "inf");
    }

    AppendFormat(stat_line, sizeof(stat_line), &off,
                 " bytes=%lu drop=%lu\r\n",
                 (unsigned long)stats.bytes_in,
                 (unsigned long)stats.resync_drop_bytes);

    Uart1SendBuffer((const uint8_t *)stat_line, (uint16_t)off);

    if (!parsed_valid)
        return;

    char key_list[128];
    BuildKeyList(parsed.keyboard, key_list, sizeof(key_list));

    char line[256];
    size_t l;

    l = 0u;
    AppendFormat(line, sizeof(line), &l,
                 "[DATA] gear=%s(%u) pause=%u customL=%u customR=%u trigger=%u\r\n",
                 GearName(parsed.gear),
                 (unsigned int)parsed.gear,
                 (unsigned int)parsed.pause,
                 (unsigned int)parsed.custom_l,
                 (unsigned int)parsed.custom_r,
                 (unsigned int)parsed.trigger);
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)l);

    l = 0u;
    AppendFormat(line, sizeof(line), &l,
                 "[DATA] CH0(右X): raw=%4u centered=%+4d (",
                 parsed.ch0, parsed.ch0_c);
    AppendSignedFixed2(line, sizeof(line), &l, ChannelPercentX100(parsed.ch0));
    AppendFormat(line, sizeof(line), &l,
                 "%%) | CH1(右Y): raw=%4u centered=%+4d (",
                 parsed.ch1, parsed.ch1_c);
    AppendSignedFixed2(line, sizeof(line), &l, ChannelPercentX100(parsed.ch1));
    AppendFormat(line, sizeof(line), &l, "%%)\r\n");
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)l);

    l = 0u;
    AppendFormat(line, sizeof(line), &l,
                 "[DATA] CH2(左Y): raw=%4u centered=%+4d (",
                 parsed.ch2, parsed.ch2_c);
    AppendSignedFixed2(line, sizeof(line), &l, ChannelPercentX100(parsed.ch2));
    AppendFormat(line, sizeof(line), &l,
                 "%%) | CH3(左X): raw=%4u centered=%+4d (",
                 parsed.ch3, parsed.ch3_c);
    AppendSignedFixed2(line, sizeof(line), &l, ChannelPercentX100(parsed.ch3));
    AppendFormat(line, sizeof(line), &l, "%%)\r\n");
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)l);

    l = 0u;
    AppendFormat(line, sizeof(line), &l,
                 "[DATA] DIAL(拨轮): raw=%4u centered=%+4d (",
                 parsed.dial, parsed.dial_c);
    AppendSignedFixed2(line, sizeof(line), &l, ChannelPercentX100(parsed.dial));
    AppendFormat(line, sizeof(line), &l, "%%)\r\n");
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)l);

    l = 0u;
    AppendFormat(line, sizeof(line), &l,
                 "[DATA] mouse(dx,dy,dz)=(%d,%d,%d) mouse_btn(L/R/M)=(%u/%u/%u)\r\n",
                 parsed.mouse_x,
                 parsed.mouse_y,
                 parsed.mouse_z,
                 (unsigned int)parsed.mouse_l,
                 (unsigned int)parsed.mouse_r,
                 (unsigned int)parsed.mouse_m);
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)l);

    l = 0u;
    AppendFormat(line, sizeof(line), &l,
                 "[DATA] keyboard=0x%04X keys=%s\r\n",
                 (unsigned int)parsed.keyboard,
                 key_list);
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)l);
}

static void PrintCRCProbeOnce(void)
{
    CRCProbe_t probe_snapshot;
    __disable_irq();
    probe_snapshot = crc_probe;
    __enable_irq();

    if (!probe_snapshot.captured)
        return;

    ParsedFrame_t probe_parsed;
    ParseFrame(probe_snapshot.frame, &probe_parsed);

    char line[256];
    size_t off = 0u;
    AppendFormat(line, sizeof(line), &off,
                 "[CRCDBG] rx_le=0x%04X rx_be=0x%04X calc_ccitt_false=0x%04X calc_ccitt_reflected=0x%04X calc_without_header=0x%04X\r\n",
                 (unsigned int)probe_snapshot.rx_le,
                 (unsigned int)probe_snapshot.rx_be,
                 (unsigned int)probe_snapshot.calc_with_header,
                 (unsigned int)probe_snapshot.calc_reflected,
                 (unsigned int)probe_snapshot.calc_without_header);
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)off);

    off = 0u;
    AppendFormat(line, sizeof(line), &off, "[CRCDBG] raw=");
    for (uint8_t i = 0u; i < VT_FRAME_SIZE; ++i)
        AppendFormat(line, sizeof(line), &off, "%02X%s", probe_snapshot.frame[i], (i + 1u < VT_FRAME_SIZE) ? " " : "");
    AppendFormat(line, sizeof(line), &off, "\r\n");
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)off);

    off = 0u;
    AppendFormat(line, sizeof(line), &off,
                 "[CRCDBG] decoded(ch0..3,dial)=(%u,%u,%u,%u,%u) centered=(%d,%d,%d,%d,%d) gear=%u kb=0x%04X mouse=(%d,%d,%d)\r\n",
                 probe_parsed.ch0, probe_parsed.ch1, probe_parsed.ch2, probe_parsed.ch3, probe_parsed.dial,
                 probe_parsed.ch0_c, probe_parsed.ch1_c, probe_parsed.ch2_c, probe_parsed.ch3_c, probe_parsed.dial_c,
                 (unsigned int)probe_parsed.gear,
                 (unsigned int)probe_parsed.keyboard,
                 probe_parsed.mouse_x, probe_parsed.mouse_y, probe_parsed.mouse_z);
    Uart1SendBuffer((const uint8_t *)line, (uint16_t)off);
}

int main(void)
{
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_SPI1_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_USART3_UART_Init();
    MX_RNG_Init();
    MX_RTC_Init();
    MX_TIM1_Init();
    MX_TIM10_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM8_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI2_Init();
    MX_CRC_Init();
    MX_DAC_Init();

    BSPInit();

    ReinitUartForVT(&huart6);

    USART_Init_Config_s conf = {
        .module_callback = VTRxCallback,
        .usart_handle = &huart6,
        .recv_buff_size = VT_RX_CHUNK_SIZE,
    };
    vt_rx_usart = USARTRegister(&conf);

    Uart1SendString("[INFO] 打开串口: USART6, baud=921600\r\n");
    Uart1SendString("[INFO] 链路离线阈值: 0.200s\r\n");

    uint32_t last_report_tick = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (link_online_rise_event)
        {
            __disable_irq();
            link_online_rise_event = 0u;
            __enable_irq();
            PrintLinkOnline(now);
        }

        if (link_online)
        {
            uint32_t last_valid = link_stats.last_valid_tick;
            if (last_valid > 0u && (now - last_valid > LINK_TIMEOUT_MS))
            {
                __disable_irq();
                if (link_online && (now - link_stats.last_valid_tick > LINK_TIMEOUT_MS))
                {
                    link_online = 0u;
                    __enable_irq();
                    PrintLinkOffline();
                }
                else
                {
                    __enable_irq();
                }
            }
        }

        if (now - last_report_tick >= REPORT_INTERVAL_MS)
        {
            last_report_tick = now;
            PrintStatAndData(now);
            PrintCRCProbeOnce();
        }

        HAL_Delay(5);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
