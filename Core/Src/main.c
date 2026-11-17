/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "app_rtos.h"
#include "Int_can.h"
#include "string.h"
#include <stdio.h>

/* OTA：分区表、跳转、参数区 */
#include "boot_partition.h"
#include "boot_loader.h"
#include "boot_param.h"

/* 外部 SPI Flash（OTA 固件暂存区） */
#include "w25q16.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* OTA 启动方式：
 *   1 = 由 Bootloader 启动，App 链接在 0x0800C000（App A 区）
 *   0 = 独立运行，App 链接在 0x08000000（只用 Keil 调试时的临时模式）
 *
 * 注意：这个宏必须和 Keil 工程里的 IROM1 起始地址保持一致，否则 VTOR 指向错误。 */
#define APP_RUN_BY_BOOTLOADER   1

/* W25Q16 上电自检开关：验证硬件时开 1，正式跑 OTA 前改成 0
 * （自检会擦掉扇区 0，别让它每次上电都清一遍） */
#define W25Q16_RUN_SELFTEST     0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 独立看门狗：LSI 32kHz / 64 分频 = 500Hz，重装载 4095 → 超时约 8.2s
 * 本工程未包含 stm32f4xx_hal_iwdg 驱动，直接操作寄存器（更精简，也不受 CubeMX 覆盖影响） */
#define IWDG_KEY_RELOAD              0xAAAAU   /* 喂狗 */
#define IWDG_KEY_ENABLE              0xCCCCU   /* 启动看门狗 */
#define IWDG_KEY_WRITE_ACCESS_ENABLE 0x5555U   /* 解锁 PR/RLR 写保护 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

#if APP_RUN_BY_BOOTLOADER
  /* 由 Bootloader 启动：复位后 VTOR 指向 0x08000000（Bootloader 的向量表），
   * 必须第一时间重映射，否则 SysTick / 串口中断会跳进 Bootloader 的向量表跑飞。
   * 放在 HAL_Init 之前 —— 越早越好。
   *
   * 注意不能写死 App A：Bootloader 可能跳到 B 区运行。而 B 区运行时如果
   * VTOR 还指着 A，OTA 一擦 A 的扇区（Sector 3/4）向量表就没了，
   * 下一个中断进来直接 HardFault（实机就是这么挂的：B→A 升级擦完扇区就复位）。
   * 所以按参数区记录的 active_slot 来设；参数区无效时（首次烧录）退回 A。 */
  {
    boot_param_t bp;
    uint32_t slot_base = APP_A_BASE_ADDR;

    if (boot_param_read(&bp)) slot_base = BOOT_SLOT_ADDR(bp.active_slot);
    boot_app_set_vtor(slot_base);
  }

  /* 清掉 Bootloader 遗留的 SysTick（HAL_RCC_DeInit 里被重新打开过）。
   * 必须在任何 __enable_irq() 之前执行，否则会立刻进 FreeRTOS 的 tick 中断，
   * 而此时调度器还没启动，直接 HardFault。 */
  boot_app_disable_systick();
#endif

  /* CPU 状态交接：除了 PRIMASK，BASEPRI 也要清干净。
   * BASEPRI 是"优先级屏蔽"：数值 ≥ BASEPRI 的中断被挡住。FreeRTOS 的临界区用
   * configMAX_SYSCALL_INTERRUPT_PRIORITY = 5<<4 = 0x50 屏蔽低优先级中断，
   * 而 HAL 时基 TIM6 的优先级是 15（最低）—— 只要 BASEPRI 残留 0x50，
   * TIM6 中断就永远进不来，HAL_GetTick 冻结，所有 AT 超时死等。
   * 复位值本该是 0，这里做一次兜底，确保不依赖 Bootloader 是否干净。 */
  __set_BASEPRI(0U);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Print_Reset_Cause();                       /* 打印本次复位原因，便于判断是否被看门狗复位 */

  Int_can_init();
  __enable_irq();

  /* 外部 Flash 自检（放在开中断之后：wait_ready 的 3s 超时依赖 HAL_GetTick，
   * 而 HAL_GetTick 靠 TIM6 中断累加，中断关着的话超时判断不会生效） */
  w25q16_init();
#if W25Q16_RUN_SELFTEST
  w25q16_selftest();
#endif

  /* 启动独立看门狗：必须在 RTOS 任务开始喂狗前启动（App_RTOS_Init 内创建监控任务） */
  MX_IWDG_Init();

  printf("--- F407 Gateway Started ---\r\n");

#if APP_RUN_BY_BOOTLOADER
  /* 走到这里说明外设都初始化完了，算"自检通过"：
   * 清启动计数、清升级标志，Bootloader 下次就不会再回滚。 */
  boot_param_mark_ok();
#endif

  /* 内部调用 vTaskStartScheduler()，正常情况不会返回 */
  App_RTOS_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 业务逻辑已全部移入 RTOS 任务（DataProc / RS485Proc / GwCmd）。
       此循环仅在调度器异常返回时兜底。 */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  独立看门狗初始化：系统跑飞/任务集体卡死时自动复位恢复
  * @note   IWDG 使用 LSI（约 32kHz），一旦启动无法软件停止，只能复位。
  *         超时 ≈ 4095 / (32000/64) ≈ 8.2s，喂狗由 Watchdog 监控任务负责。
  *
  *         本板实测：PR/RLR 写入后 PVU/RVU 同步标志一直不清（PR 回读为 0），
  *         说明 LSI 时钟没有真正输出。此时若强行启动，IWDG 会按默认 /4 分频
  *         跑出约 0.5s 的超时，导致系统不停复位。所以同步失败时宁可不启动。
  */
void MX_IWDG_Init(void)
{
  uint32_t spin;

  /* 1. 使能 LSI 并等待就绪（有界等待，不依赖 HAL 时基） */
  __HAL_RCC_LSI_ENABLE();
  for (spin = 0U; spin < 200000U; spin++)
  {
    if (RCC->CSR & RCC_CSR_LSIRDY) break;
  }
  if ((RCC->CSR & RCC_CSR_LSIRDY) == 0U)
  {
    printf("[IWDG] LSI not ready, watchdog skipped\r\n");
    return;
  }

  /* 2. 解锁并配置：LSI/64 = 500Hz，4095 个计数 ≈ 8.2s */
  IWDG->KR  = IWDG_KEY_WRITE_ACCESS_ENABLE;  /* 解锁，允许写 PR / RLR */
  IWDG->PR  = 4;                             /* 预分频 4 → LSI/64 = 500Hz */
  IWDG->RLR = 4095;                          /* 4095/500 ≈ 8.2s */

  /* 3. 等 PVU / RVU 同步完成（同样用有界等待） */
  for (spin = 0U; spin < 200000U; spin++)
  {
    if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) == 0U) break;
  }

  if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U)
  {
    /* 配置没落进 IWDG：启动会按默认 /4 分频跑 ~0.5s，必然误复位 */
    printf("[IWDG] PVU/RVU stuck (LSI dead?), watchdog skipped\r\n");
    return;
  }

  /* 4. 先喂一次，再启动（启动后无法软件关闭） */
  IWDG->KR = IWDG_KEY_RELOAD;
  IWDG->KR = IWDG_KEY_ENABLE;

  printf("[IWDG] enabled, timeout ~8.2s (PR=%lu RLR=%lu)\r\n",
         (unsigned long)IWDG->PR, (unsigned long)IWDG->RLR);
}

/**
  * @brief  打印本次复位原因并清除标志
  * @note   上电/按复位键 = POWER-ON + PIN；看门狗复位 = IWDG
  */
void Print_Reset_Cause(void)
{
  uint32_t csr = RCC->CSR;

  printf("[RST] cause:");

  if (csr & RCC_CSR_IWDGRSTF) printf(" IWDG");
  if (csr & RCC_CSR_WWDGRSTF) printf(" WWDG");
  if (csr & RCC_CSR_SFTRSTF)  printf(" SOFTWARE");
  if (csr & RCC_CSR_PORRSTF)  printf(" POWER-ON");
  if (csr & RCC_CSR_PINRSTF)  printf(" NRST-PIN");
  if (csr & RCC_CSR_BORRSTF)  printf(" BOR");

  printf("\r\n");

  RCC->CSR |= RCC_CSR_RMVF;   /* 清标志，避免下次误判 */
}

/**
  * @brief  喂狗：由 Watchdog 监控任务在确认所有业务任务存活后调用
  */
void IWDG_Feed(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
