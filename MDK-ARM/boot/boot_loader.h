#ifndef __BOOT_LOADER_H__
#define __BOOT_LOADER_H__

#include <stdint.h>

/* ============================================================================
 *  跳转相关的公共接口（Bootloader 和 App 都能用）
 * ========================================================================== */

/* 检查某地址上的固件向量表是否合理：
 *   第 0 项（初始 MSP）要落在 SRAM；
 *   第 1 项（Reset_Handler）必须落在【本槽自己】的地址范围内。
 *
 *   为什么不只判"落在 Flash"：App 是绝对地址镜像，如果下载时把链接在
 *   0x0800C000 的 A 版 bin 写进了 B 槽，它的 Reset_Handler 仍指向 0x0800Cxxx。
 *   只看 Flash 范围会放行，跳过去执行的是别人的代码 —— 症状是静默死机。
 *   限定在本槽内就能在跳转前判死，让调用方走回滚，而不是变砖。
 *
 *   size 传该槽的容量（BOOT_SLOT_SIZE(slot)）。 */
uint8_t boot_vector_ok(uint32_t base, uint32_t size);

/* 检查某个 slot 的固件是否完整：向量表合理 + CRC32 匹配 */
uint8_t boot_slot_valid(uint32_t slot, uint32_t size, uint32_t expect_crc);

/* 跳转到指定 slot 执行，正常情况下不返回。
 * 跳转失败（向量表非法）时直接返回，调用方可以继续尝试别的 slot。 */
void boot_jump(uint32_t slot);

/* App 启动时第一时间调用：把自己的向量表重映射到 base。
 * STM32 复位后 VTOR 指向 0x08000000（Bootloader），
 * App 被链接到 0x0800C000，不重映射的话所有中断都跳到 Bootloader 的向量表里。 */
void boot_app_set_vtor(uint32_t base);

/* App 启动时和 boot_app_set_vtor 一起调用：关掉并清空 SysTick。
 *
 * 为什么需要：Bootloader 的 boot_jump() 里调用 HAL_RCC_DeInit() 复位时钟，
 * 而 HAL_RCC_DeInit() 内部会再调一次 HAL_InitTick()。Bootloader 为了减小体积
 * 没有带 TIM 时基文件，链接到的是 HAL 默认的 SysTick 版本 —— 等于把 SysTick
 * 又打开了一次。Bootloader 全程关中断，SysTick 挂起位就一直挂着。
 * App 里第一次 __enable_irq() 会立刻响应这个中断；如果 App 用 FreeRTOS，
 * SysTick_Handler 就是 xPortSysTickHandler，而这时调度器还没启动 ->
 * xTaskIncrementTick() 遍历未初始化的链表 -> HardFault。
 * 所以在开中断之前必须把 SysTick 彻底清干净。 */
void boot_app_disable_systick(void);

#endif /* __BOOT_LOADER_H__ */
