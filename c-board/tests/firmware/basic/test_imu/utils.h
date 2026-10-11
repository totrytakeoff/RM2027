/**
  * @file       utils.h
  * @brief      实用工具函数头文件，包含浮点数格式化等常用功能
  */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

/**
  * @brief      完美支持浮点数的格式化函数，类似于snprintf
  * @param[out] buffer: 输出缓冲区
  * @param[in]  buffer_size: 缓冲区大小
  * @param[in]  format: 格式化字符串，支持%s, %d, %u, %x, %f, %.2f等
  * @param[in]  ...: 可变参数列表
  * @retval     实际写入的字符数（不包括终止符）
  * @note       此函数完全兼容snprintf格式，但完美支持浮点数格式化
  */
int safe_snprintf(char* buffer, size_t buffer_size, const char* format, ...);

/**
  * @brief      安全的vsnprintf实现，完美支持浮点数
  * @param[out] buffer: 输出缓冲区
  * @param[in]  buffer_size: 缓冲区大小
  * @param[in]  format: 格式化字符串
  * @param[in]  args: 可变参数列表
  * @retval     实际写入的字符数（不包括终止符）
  */
int safe_vsnprintf(char* buffer, size_t buffer_size, const char* format, va_list args);

#endif /* UTILS_H */