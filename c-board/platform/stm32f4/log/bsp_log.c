#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "utils.h"
#include "usart.h"
#include <stdarg.h>

#if BSP_LOG_USE_UART
#if BSP_LOG_UART_PORT == 1
#define BSP_LOG_UART_HANDLE huart1
#elif BSP_LOG_UART_PORT == 3
#define BSP_LOG_UART_HANDLE huart3
#elif BSP_LOG_UART_PORT == 6
#define BSP_LOG_UART_HANDLE huart6
#else
#error "Unsupported BSP_LOG_UART_PORT, use 1/3/6."
#endif
#endif


void BSPLogInit()
{
#if !BSP_LOG_USE_UART
    SEGGER_RTT_Init();
#endif
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
#if BSP_LOG_USE_UART
    char buffer[256];
    int n = RmFormat_Vsnprintf(buffer, sizeof(buffer), fmt, args);
    if (n > 0) {
        size_t len = (n < (int)sizeof(buffer)) ? (size_t)n : (sizeof(buffer) - 1U);
        HAL_UART_Transmit(&BSP_LOG_UART_HANDLE, (uint8_t *)buffer, (uint16_t)len, BSP_LOG_UART_TIMEOUT_MS);
    }
#else
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
#endif
    va_end(args);
    return n;
}

void Float2Str(char *str, float va)
{
    if (str != NULL) {
        (void)RmFormat_Snprintf(str, 24U, "%.3f", (double)va);
    }
}
