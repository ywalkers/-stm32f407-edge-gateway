#ifndef __W25Q16_H__
#define __W25Q16_H__

#include <stdint.h>

/* ============================================================================
 *  W25Q16 驱动（2MB SPI NOR Flash）
 *
 *  硬件接线（见原理图）：
 *      PA15 -> CS   (软件片选，GPIO 推挽输出)
 *      PB3  -> SCK  (SPI1_SCK,  AF5)
 *      PB4  -> MISO (SPI1_MISO, AF5)
 *      PB5  -> MOSI (SPI1_MOSI, AF5)
 *
 *  SPI1 挂在 APB2（84MHz），这里配 8 分频 = 10.5MHz。
 *  W25Q16 标准读最高 50MHz，10.5MHz 留足余量，OTA 下载瓶颈本来就在 ESP8266。
 *
 *  SPI 部分直接操作寄存器，没引 stm32f4xx_hal_spi.c
 *  （本工程 HAL 目录里没有这个文件，也不想为它引一整个 HAL 模块）。
 * ========================================================================== */

#define W25Q16_PAGE_SIZE        256U        /* 页编程单位 */
#define W25Q16_SECTOR_SIZE      4096U       /* 扇区擦除单位 */
#define W25Q16_BLOCK_SIZE       65536U      /* 块擦除单位 */
#define W25Q16_TOTAL_SIZE       0x200000UL  /* 2MB */

/* W25Q16 的 JEDEC ID：厂商 0xEF(Winbond) + 类型 0x40 + 容量 0x15(=2MB) */
#define W25Q16_JEDEC_ID         0xEF4015UL

/* 掉电保护 / 握手超时（单位：毫秒），擦除 4KB 约 45ms，64KB 约 150ms */
#define W25Q16_TIMEOUT_MS       3000U

/* 初始化 SPI1 + GPIO，并做一次唤醒 */
void w25q16_init(void);

/* ---- SPI1 总线锁（OTA 与离线缓存共用，必须互斥） ----
 * 递归互斥量：同一任务可以嵌套 Take。
 * 调度器启动前或尚未初始化时 Take/Give 直接返回成功，不报错。
 * 用法：w25q16_lock_take(超时ms) 成功后必须配对 w25q16_lock_give()。 */
#define W25Q16_LOCK_FOREVER  0xFFFFFFFFU

void w25q16_lock_init(void);
int  w25q16_lock_take(uint32_t timeout_ms);   /* 1 = 拿到锁，0 = 超时 */
void w25q16_lock_give(void);

/* 读 JEDEC ID，返回 0xEF4015 表示识别成功；异常时返回读到的原始值 */
uint32_t w25q16_read_id(void);

/* 读数据（0x03 命令，任意地址任意长度） */
void w25q16_read(uint32_t addr, void *buf, uint32_t len);

/* 写数据：内部自动处理「256 字节页整数倍」拆分，
 * 调用前对应区域必须已经擦除过（NOR Flash 只能 1->0，不能 0->1）。
 * 返回 0 = 成功，-1 = 参数非法 */
int w25q16_write(uint32_t addr, const void *buf, uint32_t len);

/* 擦除 4KB 扇区（addr 所在扇区），返回 0 成功 */
int w25q16_erase_sector(uint32_t addr);

/* 擦除 64KB 块（addr 必须 64KB 对齐才能达到最快），返回 0 成功 */
int w25q16_erase_block64(uint32_t addr);

/* 擦除一段区域：内部按 64KB 块优先、剩余按 4KB 扇区处理。
 * addr 和 len 会被向下/向上对齐到扇区边界。返回 0 成功 */
int w25q16_erase_range(uint32_t addr, uint32_t len);

/* 等待芯片内部操作完成（轮询状态寄存器 BUSY 位） */
void w25q16_wait_ready(void);

/* 上电自检：读 ID + 扇区 0 做一次擦-写-读比对，结果打到串口。
 * 只在调试阶段调用，会破坏扇区 0 的数据 */
void w25q16_selftest(void);

#endif /* __W25Q16_H__ */
