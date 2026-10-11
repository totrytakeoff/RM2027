// Minimal Error_Handler/Assert stubs for test targets to satisfy HAL references.
// 标记为 weak，若目标内已有实现（如 main.c），将自动覆盖。
#include <stdint.h>

__attribute__((weak)) void Error_Handler(void) {
    while (1) {
    }
}

__attribute__((weak)) void assert_failed(uint8_t* file, uint32_t line) {
    (void)file;
    (void)line;
    while (1) {
    }
}
