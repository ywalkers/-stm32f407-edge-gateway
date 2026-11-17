#include "boot_loader.h"
#include "boot_partition.h"
#include "boot_param.h"
#include "stm32f4xx_hal.h"

typedef void (*app_entry_t)(void);

/* ----------------------------------------------------------------------------
 *  向量表合理性检查
 *
 *  STM32 的向量表前两项固定是：
 *      [0] 初始 MSP        -> 必须落在 SRAM
 *      [1] Reset_Handler   -> 必须落在【本槽自己】的地址范围，且最低位为 1（Thumb）
 *
 *  判"落在本槽"而不是"落在 Flash 任意处"，是为了挡住镜像发错的情况：
 *  App 是绝对地址镜像，把链接在 0x0800C000 的 A 版 bin 写进 B 槽，
 *  它的 Reset_Handler 仍是 0x0800Cxxx —— 只看 Flash 范围会放行，
 *  跳过去执行的是 A 区代码（A 区一旦被擦就是静默 HardFault）。
 *  限定在本槽内就能在跳转前判死，交给调用方回滚。
 *
 *  没有这一步，跳到一片 0xFF 的空白 Flash 也会直接 HardFault，
 *  而且 HardFault 里什么都查不出来。
 * -------------------------------------------------------------------------- */
uint8_t boot_vector_ok(uint32_t base, uint32_t size)
{
    uint32_t sp = *(volatile uint32_t *)(base);
    uint32_t pc = *(volatile uint32_t *)(base + 4U);

    /* SP 必须落在 0x2000_0000 ~ 0x2001_FFFF（SRAM1 128KB） */
    if (sp < 0x20000000U || sp > 0x20020000U) return 0;

    /* Reset_Handler 必须落在本槽 [base, base+size) 区间内 */
    if (pc < base || pc >= (base + size)) return 0;

    /* Thumb 位：Cortex-M 只支持 Thumb 指令，最低位必须是 1 */
    if ((pc & 1U) == 0U) return 0;

    return 1;
}

uint8_t boot_slot_valid(uint32_t slot, uint32_t size, uint32_t expect_crc)
{
    uint32_t base;

    if (slot > SLOT_B) return 0;
    if (size == 0U || size > BOOT_SLOT_SIZE(slot)) return 0;

    base = BOOT_SLOT_ADDR(slot);

    if (!boot_vector_ok(base, BOOT_SLOT_SIZE(slot))) return 0;

    if (boot_crc32((const uint8_t *)base, size) != expect_crc) return 0;

    return 1;
}

/* ----------------------------------------------------------------------------
 *  跳转
 *
 *  顺序很重要：
 *    1. 关全局中断         —— 否则新 App 的向量表还没生效，中断会跑飞
 *    2. 关 SysTick         —— HAL 的时基，不关会一直进中断
 *    3. 清 NVIC 使能/挂起  —— 残留的中断标志会让 App 一启动就跳进旧 ISR
 *    4. 时钟复位           —— 让 App 的 SystemClock_Config 从干净的 HSI 开始
 *    5. 设 VTOR            —— 新 App 的向量表
 *    6. 设 MSP             —— 新 App 的栈
 *    7. 跳                 —— 取 Reset_Handler 地址执行
 * -------------------------------------------------------------------------- */
void boot_jump(uint32_t slot)
{
    uint32_t base = BOOT_SLOT_ADDR(slot);
    uint32_t sp, pc;
    int i;

    if (!boot_vector_ok(base, BOOT_SLOT_SIZE(slot))) return;

    sp = *(volatile uint32_t *)(base);
    pc = *(volatile uint32_t *)(base + 4U);

    __disable_irq();

    /* 关掉 SysTick 和全部 NVIC 中断 */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    for (i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFU;   /* 禁止 */
        NVIC->ICPR[i] = 0xFFFFFFFFU;   /* 清挂起 */
    }

    /* 外设和时钟恢复默认（HSI），让 App 重新配置时钟 */
    HAL_RCC_DeInit();

    /* 注意：HAL_RCC_DeInit() 内部会调用 HAL_InitTick()。
     * 本 Bootloader 没有链接 TIM 时基文件，HAL_InitTick 走的是默认的 SysTick 版本，
     * 会把上面刚关掉的 SysTick 重新打开；而我们现在是关中断状态，SysTick 的
     * 挂起位会一直挂着，App 一开中断就立刻跳进 tick 中断。
     * 所以这里必须再关一次，并把挂起位清掉。 */
    boot_app_disable_systick();

    /* 向量表重映射 + 栈指针 */
    SCB->VTOR = base;
    __set_MSP(sp);

    /* 内存屏障：确保上面的写在跳转前全部完成 */
    __DSB();
    __ISB();

    ((app_entry_t)pc)();

    /* 正常不会走到这里 */
    while (1) { }
}

void boot_app_set_vtor(uint32_t base)
{
    SCB->VTOR = base;
    __DSB();
    __ISB();
}

void boot_app_disable_systick(void)
{
    SysTick->CTRL = 0;                        /* 关使能 + 关 tick 中断 + 选外部时钟 */
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* 清 SysTick 挂起位：只写 PENDSTCLR 这一位（写 1 清除），其它位保持不清 */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    /* 同时把 NVIC 里的 SysTick 使能和挂起状态也清掉，
     * 防止前面某次配置留下残余（SysTick 是系统异常，ICER 位 15） */
    NVIC->ICER[0] = (1U << 15);
    NVIC->ICPR[0] = (1U << 15);

    __DSB();
    __ISB();
}
