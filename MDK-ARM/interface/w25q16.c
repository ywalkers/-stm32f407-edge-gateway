/* ============================================================================
 *  W25Q16 SPI NOR Flash 驱动（寄存器版 SPI1）
 *
 *  为什么不用 HAL：本工程 HAL 目录里没有 stm32f4xx_hal_spi.c，
 *  为了一颗 SPI Flash 引一整个 HAL 模块不划算；而且 SPI1 主机模式
 *  就 CR1 里 5 个位，寄存器写反而更直观。
 * ========================================================================== */

#include "w25q16.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>

/* ---------------------------- 硬件相关宏 ---------------------------- */

/* 片选：PA15，低电平选中。用 BSRR 置位/复位，单周期、不会被中断打断成半个写 */
#define W25Q_CS_PIN         GPIO_PIN_15
#define W25Q_CS_LOW()       (GPIOA->BSRR = ((uint32_t)W25Q_CS_PIN) << 16U)
#define W25Q_CS_HIGH()      (GPIOA->BSRR = (uint32_t)W25Q_CS_PIN)

/* W25Q16 指令集（只列用得到的） */
#define CMD_WRITE_ENABLE    0x06U
#define CMD_READ_STATUS1    0x05U
#define CMD_READ_DATA       0x03U
#define CMD_PAGE_PROGRAM    0x02U
#define CMD_SECTOR_ERASE    0x20U   /* 4KB  */
#define CMD_BLOCK_ERASE_64K 0xD8U   /* 64KB */
#define CMD_JEDEC_ID        0x9FU

#define STATUS_BUSY         0x01U

/* ---------------------------- SPI1 总线锁 ---------------------------- */
/* OTA 下载和离线缓存都会访问同一颗 W25Q16，必须串行化。
 * 用递归互斥量：缓存层可以先 Take 再调用本文件的 API（内部会再 Take 一次）。 */
static SemaphoreHandle_t s_bus_mutex = NULL;

void w25q16_lock_init(void)
{
    if (s_bus_mutex == NULL)
        s_bus_mutex = xSemaphoreCreateRecursiveMutex();
}

int w25q16_lock_take(uint32_t timeout_ms)
{
    TickType_t ticks;

    /* 调度器还没跑（main 里的自检）或锁没创建：直接放行，不做同步 */
    if (s_bus_mutex == NULL) return 1;
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return 1;

    ticks = (timeout_ms == W25Q16_LOCK_FOREVER) ? portMAX_DELAY
                                                : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTakeRecursive(s_bus_mutex, ticks) == pdTRUE) ? 1 : 0;
}

void w25q16_lock_give(void)
{
    if (s_bus_mutex == NULL) return;
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return;
    (void)xSemaphoreGiveRecursive(s_bus_mutex);
}

/* ---------------------------- SPI1 底层 ---------------------------- */

static void spi1_init(void)
{
    /* 时钟：GPIOA/GPIOB 在 AHB1，SPI1 在 APB2 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    __DSB();
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;                       /* 读一次，确保时钟真正打开 */

    /* PA15 / CS：先拉高再切输出，避免切模式瞬间误选中 Flash */
    GPIOA->BSRR = (uint32_t)W25Q_CS_PIN;
    GPIOA->MODER  &= ~(3U << (15U * 2U));
    GPIOA->MODER  |=  (1U << (15U * 2U));     /* 01 = 通用输出 */
    GPIOA->OTYPER &= ~(1U << 15U);            /* 推挽 */
    GPIOA->OSPEEDR |= (3U << (15U * 2U));     /* 高速 */
    GPIOA->PUPDR  &= ~(3U << (15U * 2U));

    /* PB3 SCK / PB4 MISO / PB5 MOSI -> AF5(SPI1)
     *
     * 注意：这 3 个脚复位后默认是 JTAG 功能（JTDO / NJTRST）。
     * STM32F4 没有 AFIO，不需要像 F1 那样写 SWJ_CFG，
     * 只要把 MODER 改成复用、AFR 选 AF5，JTAG 就自动让出来了。
     * 前提是用 SWD 调试（只用 PA13/PA14），别用 JTAG。 */
    GPIOB->MODER  &= ~((3U << (3U * 2U)) | (3U << (4U * 2U)) | (3U << (5U * 2U)));
    GPIOB->MODER  |=  ((2U << (3U * 2U)) | (2U << (4U * 2U)) | (2U << (5U * 2U)));
    GPIOB->OSPEEDR |= ((3U << (3U * 2U)) | (3U << (4U * 2U)) | (3U << (5U * 2U)));
    GPIOB->PUPDR  &= ~((3U << (3U * 2U)) | (3U << (4U * 2U)) | (3U << (5U * 2U)));
    GPIOB->AFR[0] &= ~((0xFU << (3U * 4U)) | (0xFU << (4U * 4U)) | (0xFU << (5U * 4U)));
    GPIOB->AFR[0] |=  ((5U << (3U * 4U)) | (5U << (4U * 4U)) | (5U << (5U * 4U)));

    /* SPI1：主机 / 8bit / 模式0(CPOL=0,CPHA=0) / 软件 NSS / 8 分频
     * APB2 = 84MHz，8 分频 = 10.5MHz（W25Q16 标准读最高 50MHz） */
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    SPI1->CR1 = SPI_CR1_MSTR      /* 主机模式 */
              | SPI_CR1_SSM       /* 软件管理 NSS */
              | SPI_CR1_SSI       /* 内部 NSS 拉高，否则主机认为总线冲突不工作 */
              | (2U << SPI_CR1_BR_Pos)   /* 010 = fPCLK/8 */
              | SPI_CR1_SPE;      /* 使能 SPI */
}

/* 收发一个字节（全双工，发送的同时把收到的读回来） */
static uint8_t spi_xfer(uint8_t tx)
{
    while ((SPI1->SR & SPI_SR_TXE) == 0U) { }   /* 等发送缓冲空 */
    *(volatile uint8_t *)&SPI1->DR = tx;        /* 8bit 数据帧，用字节访问 */
    while ((SPI1->SR & SPI_SR_RXNE) == 0U) { }  /* 等接收完成 */
    return *(volatile uint8_t *)&SPI1->DR;
}

static void w25q_send_addr(uint32_t addr)
{
    spi_xfer((uint8_t)(addr >> 16));
    spi_xfer((uint8_t)(addr >> 8));
    spi_xfer((uint8_t)addr);
}

/* ---------------------------- 基本操作 ---------------------------- */

void w25q16_wait_ready(void)
{
    uint32_t t0;

    w25q16_lock_take(W25Q16_LOCK_FOREVER);

    W25Q_CS_LOW();
    spi_xfer(CMD_READ_STATUS1);

    t0 = HAL_GetTick();
    while (spi_xfer(0xFFU) & STATUS_BUSY)
    {
        if ((HAL_GetTick() - t0) > W25Q16_TIMEOUT_MS)
        {
            printf("[W25Q16] wait_ready timeout!\r\n");
            break;
        }
    }
    W25Q_CS_HIGH();

    w25q16_lock_give();
}

static void w25q_write_enable(void)
{
    W25Q_CS_LOW();
    spi_xfer(CMD_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

void w25q16_init(void)
{
    w25q16_lock_init();
    w25q16_lock_take(W25Q16_LOCK_FOREVER);

    spi1_init();

    /* 清一次残留的 RXNE 标志，避免第一次读拿到脏数据 */
    (void)SPI1->DR;

    /* 唤醒：一次空操作即可让 Flash 退出低功耗 */
    w25q16_wait_ready();

    w25q16_lock_give();
}

uint32_t w25q16_read_id(void)
{
    uint32_t id = 0;
    int i;

    w25q16_lock_take(W25Q16_LOCK_FOREVER);

    W25Q_CS_LOW();
    spi_xfer(CMD_JEDEC_ID);
    for (i = 0; i < 3; i++)
        id = (id << 8) | spi_xfer(0xFFU);
    W25Q_CS_HIGH();

    w25q16_lock_give();

    return id;
}

void w25q16_read(uint32_t addr, void *buf, uint32_t len)
{
    uint8_t *p = (uint8_t *)buf;

    if (len == 0U) return;

    w25q16_lock_take(W25Q16_LOCK_FOREVER);

    W25Q_CS_LOW();
    spi_xfer(CMD_READ_DATA);
    w25q_send_addr(addr);
    while (len--) *p++ = spi_xfer(0xFFU);
    W25Q_CS_HIGH();

    w25q16_lock_give();
}

/* 页编程：len <= 256，且不能跨页（调用方保证） */
static int w25q_page_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t i;

    w25q_write_enable();

    W25Q_CS_LOW();
    spi_xfer(CMD_PAGE_PROGRAM);
    w25q_send_addr(addr);
    for (i = 0; i < len; i++)
        spi_xfer(buf[i]);
    W25Q_CS_HIGH();

    w25q16_wait_ready();
    return 0;
}

int w25q16_write(uint32_t addr, const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    int rc = 0;

    if (((uint64_t)addr + len) > W25Q16_TOTAL_SIZE) return -1;

    w25q16_lock_take(W25Q16_LOCK_FOREVER);

    /* 按页边界拆分：W25Q 的页编程指令跨页会绕回页首，必须自己拆 */
    while (len > 0U)
    {
        uint32_t page_left = W25Q16_PAGE_SIZE - (addr % W25Q16_PAGE_SIZE);
        uint32_t n = (len < page_left) ? len : page_left;

        if (w25q_page_program(addr, p, n) != 0) { rc = -1; break; }

        addr += n;
        p    += n;
        len  -= n;
    }

    w25q16_lock_give();
    return rc;
}

/* ---------------------------- 擦除 ---------------------------- */

static int w25q_erase_cmd(uint8_t cmd, uint32_t addr)
{
    if (addr >= W25Q16_TOTAL_SIZE) return -1;

    w25q_write_enable();

    W25Q_CS_LOW();
    spi_xfer(cmd);
    w25q_send_addr(addr);
    W25Q_CS_HIGH();

    w25q16_wait_ready();
    return 0;
}

int w25q16_erase_sector(uint32_t addr)
{
    int rc;

    w25q16_lock_take(W25Q16_LOCK_FOREVER);
    rc = w25q_erase_cmd(CMD_SECTOR_ERASE, addr & ~(W25Q16_SECTOR_SIZE - 1U));
    w25q16_lock_give();

    return rc;
}

int w25q16_erase_block64(uint32_t addr)
{
    int rc;

    w25q16_lock_take(W25Q16_LOCK_FOREVER);
    rc = w25q_erase_cmd(CMD_BLOCK_ERASE_64K, addr & ~(W25Q16_BLOCK_SIZE - 1U));
    w25q16_lock_give();

    return rc;
}

int w25q16_erase_range(uint32_t addr, uint32_t len)
{
    uint32_t start = addr & ~(W25Q16_SECTOR_SIZE - 1U);
    uint32_t end   = (addr + len + W25Q16_SECTOR_SIZE - 1U) & ~(W25Q16_SECTOR_SIZE - 1U);
    int      rc = 0;

    if (len == 0U || end > W25Q16_TOTAL_SIZE) return -1;

    w25q16_lock_take(W25Q16_LOCK_FOREVER);

    /* 64KB 对齐且剩余够一整块时用块擦除（约 150ms），否则退化成 4KB 扇区擦（约 45ms/个） */
    while (start < end)
    {
        if ((start % W25Q16_BLOCK_SIZE) == 0U && (end - start) >= W25Q16_BLOCK_SIZE)
        {
            if (w25q16_erase_block64(start) != 0) { rc = -1; break; }
            start += W25Q16_BLOCK_SIZE;
        }
        else
        {
            if (w25q16_erase_sector(start) != 0) { rc = -1; break; }
            start += W25Q16_SECTOR_SIZE;
        }
    }

    w25q16_lock_give();
    return rc;
}

/* ---------------------------- 自检 ---------------------------- */

void w25q16_selftest(void)
{
    uint8_t  wr[W25Q16_PAGE_SIZE];
    uint8_t  rd[W25Q16_PAGE_SIZE];
    uint32_t id;
    uint32_t i;
    int      fail = 0;
    uint32_t first_bad = 0;

    id = w25q16_read_id();
    printf("[W25Q16] JEDEC ID = 0x%06lX\r\n", (unsigned long)id);

    if (id != W25Q16_JEDEC_ID)
    {
        printf("[W25Q16] ID mismatch, expect 0x%06lX -- check PA15/PB3/PB4/PB5 wiring\r\n",
               (unsigned long)W25Q16_JEDEC_ID);
        return;
    }
    printf("[W25Q16] detected: Winbond 2MB\r\n");

    /* 扇区 0 做一次 擦除 -> 写 -> 读回比对 */
    for (i = 0; i < W25Q16_PAGE_SIZE; i++)
        wr[i] = (uint8_t)(i * 7U + 0x3CU);

    if (w25q16_erase_sector(0) != 0) { printf("[W25Q16] erase fail\r\n"); return; }

    if (w25q16_write(0, wr, sizeof(wr)) != 0) { printf("[W25Q16] write fail\r\n"); return; }

    w25q16_read(0, rd, sizeof(rd));

    for (i = 0; i < W25Q16_PAGE_SIZE; i++)
    {
        if (rd[i] != wr[i])
        {
            if (fail == 0) first_bad = i;
            fail++;
        }
    }

    if (fail == 0)
    {
        printf("[W25Q16] read/write selftest PASS (256B erase-write-read)\r\n");
    }
    else
    {
        printf("[W25Q16] read/write selftest FAIL: %d bytes differ, first @%lu read 0x%02X want 0x%02X\r\n",
               fail, (unsigned long)first_bad, rd[first_bad], wr[first_bad]);
    }
}
