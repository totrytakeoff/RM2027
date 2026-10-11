#include "h7_private.h"
#include "resources.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "fdcan.h"
static RmBoardInfo board_info;
static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) RmFault_Panic(0x2001U);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    uint32_t start = HAL_GetTick();
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
        if (HAL_GetTick() - start > 100U) RmFault_Panic(0x2002U);
    }
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
    osc.HSEState = RCC_HSE_ON; osc.LSIState = RCC_LSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON; osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 2; osc.PLL.PLLN = 40; osc.PLL.PLLP = 1;
    osc.PLL.PLLQ = 6; osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) RmFault_Panic(0x2003U);
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2; clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2; clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) RmFault_Panic(0x2004U);
}
void RmBoard_HeaterOff(void)
{
    /* PB1 is deliberately GPIO-low until a validated temperature profile is added. */
    GPIOB->BSRR = GPIO_PIN_1 << 16U;
}
bool RmBoard_KeyPressed(void) { return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET; }
void RmBoard_Init(void)
{
    board_info.reset_flags = RCC->RSR;
    board_info.cpuid = SCB->CPUID;
    RmMemory_EnableCaches();
    if (HAL_Init() != HAL_OK) RmFault_Panic(0x2005U);
    /* Reset assembly already supervises supply/clock/peripheral setup. Do not
       renew that startup budget here; runtime takes over with progress feeding. */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    MX_GPIO_Init();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef heater = {0};
    RmBoard_HeaterOff();
    heater.Pin = GPIO_PIN_1; heater.Mode = GPIO_MODE_OUTPUT_PP;
    heater.Pull = GPIO_PULLDOWN; heater.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &heater);
    clock_init();
    RmTime_Init();
    __HAL_RCC_DMA1_CLK_ENABLE(); __HAL_RCC_DMA2_CLK_ENABLE();
    MX_UART5_Init(); MX_USART1_UART_Init(); MX_UART7_Init(); MX_USART10_UART_Init();
    MX_SPI2_Init();
    MX_FDCAN1_Init(); MX_FDCAN2_Init(); MX_FDCAN3_Init();
    RmSensorBus_Init();
    board_info.cpu_hz = SystemCoreClock;
    board_info.hclk_hz = HAL_RCC_GetHCLKFreq();
    board_info.fdcan_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    board_info.hal_version = HAL_GetHalVersion();
    board_info.dcache_enabled = (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
    if (board_info.cpu_hz != 480000000U || board_info.fdcan_hz != MC02_FDCAN_CLOCK_HZ)
        RmFault_Panic(0x2006U);
    if ((SCB->CPUID & 0xFF0FFFF0U) != 0x410FC270U) RmFault_Panic(0x2007U);
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
const RmBoardInfo *RmBoard_GetInfo(void) { return &board_info; }
void Error_Handler(void) { RmFault_Panic(0x20FFU); }
