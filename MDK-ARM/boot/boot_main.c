/* ============================================================================
 *  F407 OTA Bootloader
 *
 *  职责只有三件事：
 *    1. 读参数区，决定该启动哪个 slot
 *    2. 有新固件时校验它（向量表 + CRC32）
 *    3. 跳过去；连续失败超阈值就回滚
 *
 *  刻意保持极小：不引 FreeRTOS、不引 CubeMX 的 MX_* 初始化，
 *  UART 直接用寄存器配置，保证 Bootloader 永远不成为变砖点。
 * ========================================================================== */

#include "stm32f4xx_hal.h"
#include "boot_partition.h"
#include "boot_param.h"
#include "boot_loader.h"

/* --------------------------------------------------------------------------
 *  最小串口输出（USART1: PA9 TX，115200 8N1，HSI 16MHz）
 *  不用 printf，省掉整个 stdio 的代码体积
 * ------------------------------------------------------------------------ */
static void boot_uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9 -> AF7 (USART1_TX) */
    GPIOA->MODER  &= ~(3U << (9U * 2U));
    GPIOA->MODER  |=  (2U << (9U * 2U));
    GPIOA->AFR[1] &= ~(0xFU << ((9U - 8U) * 4U));
    GPIOA->AFR[1] |=  (7U  << ((9U - 8U) * 4U));

    /* 16MHz / 115200 = 138.9 -> BRR = 139 */
    USART1->BRR = 139U;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void boot_putc(char c)
{
    while ((USART1->SR & USART_SR_TXE) == 0U) { }
    USART1->DR = (uint32_t)(uint8_t)c;
}

static void boot_puts(const char *s)
{
    while (*s) boot_putc(*s++);
}

static void boot_put_hex(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    int i;
    boot_puts("0x");
    for (i = 28; i >= 0; i -= 4)
        boot_putc(hex[(v >> i) & 0xFU]);
}

static void boot_put_dec(uint32_t v)
{
    char buf[10];
    int i = 0;

    if (v == 0U) { boot_putc('0'); return; }
    while (v > 0U && i < 10) { buf[i++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (i > 0) boot_putc(buf[--i]);
}

static void boot_dump_param(const boot_param_t *p)
{
    boot_puts("[boot]   active_slot = ");
    boot_put_dec(p->active_slot);
    boot_puts("  upgrade_flag = ");
    boot_put_dec(p->upgrade_flag);
    boot_puts("\r\n[boot]   boot_count  = ");
    boot_put_dec(p->boot_count);
    boot_puts(" / ");
    boot_put_dec(p->boot_max);
    boot_puts("  fw_version = ");
    boot_put_hex(p->fw_version);
    boot_puts("\r\n");
}

int main(void)
{
    boot_param_t p;
    uint8_t param_ok;

    boot_uart_init();

    boot_puts("\r\n");
    boot_puts("========================================\r\n");
    boot_puts("[boot] F407 OTA Bootloader v1.0\r\n");
    boot_puts("[boot] App A = ");
    boot_put_hex(APP_A_BASE_ADDR);
    boot_puts("  App B = ");
    boot_put_hex(APP_B_BASE_ADDR);
    boot_puts("\r\n");

    /* ---- 1. 读参数区 ---- */
    param_ok = boot_param_read(&p);
    if (!param_ok)
    {
        boot_puts("[boot] no valid param (blank or CRC error), use default\r\n");
        boot_param_default(&p);
    }
    else
    {
        boot_puts("[boot] param loaded\r\n");
        boot_dump_param(&p);
    }

    /* ---- 2. 有待验证的新固件？ ---- */
    if (p.upgrade_flag == BOOT_FLAG_PENDING)
    {
        boot_puts("[boot] upgrade pending, verifying slot ");
        boot_put_dec(p.target_slot);
        boot_puts(" (");
        boot_put_dec(p.target_size);
        boot_puts(" bytes)...\r\n");

        if (boot_slot_valid(p.target_slot, p.target_size, p.target_crc))
        {
            boot_puts("[boot] slot valid -> activate\r\n");
            p.active_slot  = p.target_slot;
            p.upgrade_flag = BOOT_FLAG_NONE;
            /* 从 1 起算：本次启动就是新固件的第 1 次尝试。
             * 不能清零 —— 清零的话新固件一启动就崩、IWDG 复位，
             * 下次还是 boot_count=0，永远触发不了回滚，变成无限重启。 */
            p.boot_count   = 1;
            boot_param_write(&p);
        }
        else
        {
            boot_puts("[boot] slot INVALID -> rollback to App A\r\n");
            p.active_slot  = SLOT_A;
            p.upgrade_flag = BOOT_FLAG_NONE;
            p.boot_count   = 0;
            boot_param_write(&p);
        }
    }
    else
    {
        /* ---- 3. 启动计数 ----
         * 只有"新固件还没被 App 确认"（boot_count > 0）时才累加：
         *   - App 自检通过会调 boot_param_mark_ok() 把计数清零 -> 稳态不再写 Flash
         *   - 新固件跳过去但没跑到自检（HardFault / IWDG 复位）-> 一直累加
         *     超过 boot_max 就自动回滚到 A
         * 稳态（跑在 A，或 B 已确认）boot_count 恒为 0，Bootloader 不碰 Flash。 */
        if (p.boot_count > 0U)
        {
            p.boot_count++;

            if (p.boot_count > p.boot_max)
            {
                boot_puts("[boot] boot_count exceeded -> force rollback to App A\r\n");
                p.active_slot = SLOT_A;
                p.boot_count  = 0;
            }
            boot_param_write(&p);
        }
    }

    /* ---- 4. 跳转 ---- */
    boot_puts("[boot] jumping to slot ");
    boot_put_dec(p.active_slot);
    boot_puts(" @ ");
    boot_put_hex(BOOT_SLOT_ADDR(p.active_slot));
    boot_puts("\r\n");

    boot_jump(p.active_slot);

    /* 跳转函数返回 = 目标 slot 的向量表非法 */
    boot_puts("[boot] JUMP FAILED - halt\r\n");
    while (1) { }
}
