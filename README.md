# F407 多协议边缘网关（项目二）

基于 **STM32F407VGT6 + FreeRTOS** 的多协议边缘计算网关：下行接入 **CAN 总线**温湿度节点与 **RS485（Modbus RTU）** 总线，边缘解析过滤后，上行通过 **ESP8266 Wi-Fi 透传 + 自主封装 MQTT** 推送到云端 Broker。

---

## 1. 系统架构

```
                    ┌─────────────────────────────────────────────┐
  F103 温湿度节点 ──CAN──> │  CAN1(PA11/PA12, 500kbps)               │
                          │      │ 中断 -> 队列                      │
                          │      ▼                                  │
                          │  DataProc 任务（解析温湿度）              │
                          │      │  ├─ 组装 JSON ──> Upload_Queue    │
                          │      │  └─ 更新 Modbus 保持寄存器        │
                          │      ▼                                  │
                          │  MqttProc 任务（自主封装 MQTT）           │
                          │      │  PUBLISH                          │
                          │      ▼                                  │
                          │  USART3(PB10/PB11) -> ESP8266 -> WiFi   │──> MQTT Broker(云端)
                          │                                          │
  RS485 主站(上位机) <──> │  USART2(PA2/PA3, 115200, DMA 双缓冲)     │
                          │  RS485Proc 任务（Modbus RTU 从站 0x03）   │
                          └─────────────────────────────────────────┘
```

---

## 2. 硬件组成与接线

### 2.1 模块清单

| 模块 | 型号 | 用途 |
|---|---|---|
| 主控 | STM32F407VGT6 开发板 | 网关核心 |
| CAN 收发器 | TJA1050 | CAN 电平转换 |
| RS485 收发器 | MAX485 / SP3485 | RS485 电平转换 |
| Wi-Fi 模块 | ESP8266-01S（AT 固件） | MQTT 上行 |
| 温湿度节点 | STM32F103C8T6 + DHT11 | CAN 数据源 |
| 调试 | USB 转 TTL | USART1 printf |

### 2.2 引脚接线

| 外设 | 引脚 | 功能 |
|---|---|---|
| CAN1_RX / CAN1_TX | PA11 / PA12 | 接 TJA1050 → CAN 总线 |
| USART1_TX / RX | PA9 / PA10 | printf 调试（USB-TTL） |
| USART2_TX / RX | PA2 / PA3 | RS485（接 MAX485 RO/DI） |
| RS485_EN | PC4 | MAX485 的 DE/RE（方向控制） |
| USART3_TX / RX | PB10 / PB11 | ESP8266 |
| PA1 | 输出 | 预留（可接 LED） |

### 2.3 关键接线说明

- **RS485 方向控制**：PC4 → MAX485 的 `DE`+`RE`（收发使能，高=发送，低=接收）。
- **ESP8266**：独立 3.3V 供电（板载 3.3V 带不动），CH_PD(EN) 拉高，TX/RX 与 USART3 交叉。
- **CAN / RS485 两端**各接 120Ω 终端电阻；所有板子**共地**。

---

## 3. 软件模块划分

```
MDK-ARM/
├── app_rtos/
│   └── app_rtos.c/h      # 任务编排、FreeRTOS 对象、中断回调、RS485 收发
├── interface/
│   ├── Int_can.c/h       # CAN 初始化/过滤器/发送、网关下发命令
│   ├── modbus.c/h        # Modbus RTU 从站（CRC16 + 0x03 读保持寄存器）
│   └── edge.c/h          # 边缘计算：滑动平均/死区/告警/分钟统计
└── upload/
    ├── mqtt.c/h          # 自主封装 MQTT 3.1.1（CONNECT/PUBLISH/SUBSCRIBE/PINGREQ）
    ├── esp8266.c/h       # ESP8266 AT 指令封装（WiFi/TCP/*TLS*/透传）
    └── wifi_config.h     # Wi-Fi/MQTT 凭据（不进版本库）
```

### RTOS 任务一览

| 任务 | 优先级 | 职责 |
|---|---|---|
| `DataProc` | 2 | 消费 CAN 队列，解析温湿度 → JSON 入队 + 更新 Modbus 寄存器 |
| `RS485Proc` | 2 | 消费消息缓冲，Modbus RTU 从站请求处理 |
| `MqttProc` | 2 | ESP8266 连接管理 + 心跳保活 + MQTT 发布 |
| `GwCmd` | 1 | 周期下发 LED/上报周期命令到 F103 节点 |

---

## 4. 通信协议

### 4.1 CAN（下行采集）

- 标准帧 2.0A，波特率 **500 kbps**
- F103 节点 **ID=1**，数据格式：

| 字节 | 含义 |
|---|---|
| data[0..3] | 温度 float（小端） |
| data[4..5] | 湿度 ×10（uint16，高字节在前） |

- 网关下发命令（`Gateway_Send_Cmd`）：
  - `0x02`：修改上报周期（data[1..2] = 周期 ms）
  - `0x03`：控制 LED（data[1] = 0/1）

### 4.2 RS485 / Modbus RTU（下行接入）

- 波特率 **115200**，8N1，**从站地址 = 1**
- 支持功能码 **0x03（读保持寄存器）**
- 保持寄存器映射：

| 寄存器 | 含义 | 说明 |
|---|---|---|
| 0 | 温度 | 单位 0.1°C（260 = 26.0°C） |
| 1 | 湿度 | 单位 0.1%（665 = 66.5%） |

- 读寄存器请求示例（读 0 起 2 个寄存器）：
  ```
  01 03 00 00 00 02 C4 0B
  ```
- 接收采用 **串口空闲中断(IDLE) + DMA Ping-Pong 双缓冲**，DMA 填一块、CPU 处理另一块，无缝切换不丢包；收满的一块以**一条完整消息**写入 4KB MessageBuffer，任务侧按消息边界取帧，溢出时整帧丢弃，不会出现半包错位。

### 4.3 MQTT（上行推送 + 下行控制）

- 协议 **MQTT 3.1.1**，上行 **QoS1**、下行订阅 **QoS1**
- 传输：ESP8266 AT 固件 **Wi-Fi 透传**，MQTT 报文由 F407 自主组帧
- Broker：`broker.emqx.io:1883`（可换自建 EMQX）
- 主题与报文：

| 方向 | 主题 | payload | QoS / retain |
|---|---|---|---|
| 上行 | `gw/node1/sensor` | `{"node":1,"temp":26.0,"humi":66.5}` | QoS1 / 0 |
| 上行 | `gw/node1/online` | `{"online":1}` 上线；`{"online":0}` 遗嘱 | QoS0 / retain=1 |
| 下行 | `gw/node1/cmd` | `{"led":1}` 或 `{"delay":500}` | 订阅 QoS1 |

- **QoS1 上行**：停等协议，发出后等 PUBACK，3s 超时原样重发，最多 3 次；耗尽则判定链路异常重连。
- **QoS1 下行**：收到 PUBLISH 后回 PUBACK（中断只置标志，由任务发送，避免串口发送冲突）。
- **LWT 遗嘱消息**：CONNECT 时注册 `gw/node1/online` 的遗嘱，设备异常掉线（掉电/断网）时由 broker 代发 `{"online":0}`，与正常上线的 `{"online":1}` 配对。
- **下行命令执行**：接收中断里只解析并置标志（`cloud_cmd_pending`），任务里再调 `Gateway_Send_Cmd()` 下发到 CAN。原因是 printf 走互斥量，中断上下文不能调用。
- 心跳保活：每 30s 发一次 PINGREQ（keepalive=60s），5s 无 PINGRESP 判定断线重连。

**传输安全**：

- `ESP8266_USE_TLS = 1` 时走 `AT+CIPSTART="SSL"` + 端口 8883，链路加密；置 0 则回退到 1883 明文。
- TLS 握手需 `AT+CIPSSLSIZE=4096` 预分配缓冲。ESP8266-01S 可用堆仅 40~50KB，若出现 `CIPSTART` 失败或模块重启，说明内存不足，可调小该值或改用 ESP32。
- 凭据全部集中在 `upload/wifi_config.h`，**该文件不应提交到版本库**。
- MQTT 认证：`MQTT_AUTH_ENABLE = 1` 时带用户名密码（公共 broker 保持 0）。

### 4.4 边缘计算（`interface/edge.c`）

把"原始采集"和"上行上报"解耦，网关不再只是转发器：

| 功能 | 做法 | 效果 |
|---|---|---|
| **滑动平均** | 8 点环形窗口 | 抑制传感器抖动，Modbus 寄存器也写滤波值 |
| **死区过滤** | 温度变化 < 0.2°C 且湿度 < 1% 不上报 | 静置场景上行流量大幅下降 |
| **阈值告警** | 高温 > 30°C / 低温 < 5°C，带 1°C 滞回 | 只在进入/退出告警时各报一次，不刷屏 |
| **分钟统计** | 30 次采样（2s × 30 = 60s）聚合 | 上报 `gw/node1/stats`，云端免聚合 |

上报主题：

| 主题 | payload 示例 |
|---|---|
| `gw/node1/sensor` | `{"node":1,"temp":26.0,"humi":66.5}` |
| `gw/node1/alarm` | `{"type":"temp_high","temp":31.2}` / `temp_low` / `recover` |
| `gw/node1/stats` | `{"temp_avg":26.1,"temp_min":25.8,"temp_max":26.5,"humi_avg":66.2,...,"n":30}` |

---

## 5. 数据流

**上行**：F103 温湿度 —CAN→ `DataProc` 解析 → 组装 JSON → `Upload_Queue` → `MqttProc` → MQTT PUBLISH → 云端

**下行（RS485 读）**：主站发 Modbus 0x03 → `RS485Proc` 校验 CRC → 回温湿度寄存器

**下行（网关控制）**：`GwCmd` 周期发 `0x02/0x03` → CAN → F103 节点

---

## 6. 编译与烧录

1. **CubeMX** 打开 `f407.ioc`，确认已启用：CAN1、USART1、USART2、USART3（勾选 NVIC 中断）、DMA（USART2 RX/TX）、FreeRTOS、TIM6 时基；
2. 重新生成代码；
3. **Keil** 打开 `MDK-ARM/f407.uvprojx`，将以下源文件加入工程：
   - `MDK-ARM/interface/modbus.c`
   - `MDK-ARM/interface/edge.c`（新增）
   - `MDK-ARM/upload/mqtt.c`
   - `MDK-ARM/upload/esp8266.c`
4. 修改 `upload/wifi_config.h` 中的 `WIFI_SSID` / `WIFI_PASS`；
5. 编译，ST-Link / 串口下载到 F407。

---

## 7. 测试验证

1. **串口日志**（USART1，115200）：应依次打印
   ```
   --- CAN Start SUCCESS! ---
   --- OS Scheduler Started! ---
   [MQTT] Init ESP8266...
   [MQTT] Connected!
   [MQTT] Online notify sent
   ```
2. **MQTT 上行**：电脑装 MQTTX，连接 `broker.emqx.io:1883`，订阅 `gw/#`，应收到
   `{"online":1}` 及持续不断的 `{"node":1,"temp":...,"humi":...}`。
   串口日志应出现 `[MQTT] Publish(QoS1): ...`，说明收到了 PUBACK。
3. **MQTT 下行控制**：在 MQTTX 里向 `gw/node1/cmd` 发布
   `{"led":1}`（控制 F103 节点 LED）或 `{"delay":1000}`（改上报周期 1s），
   串口应打印 `[Cloud CMD] code=0x03 param=1` / `code=0x02 param=1000`。
4. **LWT 验证**：MQTTX 订阅 `gw/node1/online`，直接给网关断电（不要正常断开），
   约 1.5 × keepalive（90s）后 broker 会代发 `{"online":0}`。
5. **边缘计算**：
   - 静置时上行频率应明显低于 2s 一次（死区生效）；
   - 用手捂住 DHT11 让温度升过 30°C，应收到 `gw/node1/alarm` 的
     `{"type":"temp_high",...}`，松手降温到 29°C 以下收到 `{"type":"recover",...}`；
   - 每 60s 收到一条 `gw/node1/stats`。
6. **Modbus**：用 Modbus Poll 或串口助手发 `01 03 00 00 00 02 C4 0B`，
   应收到类似 `01 03 04 01 04 02 99 xx xx`（温度 26.0°C、湿度 66.5%）。

---

## 8. 开发与版本管理

### 8.1 分支策略

- `main`：始终可编译、可烧录的稳定版本；
- `feat/xxx`：新功能开发，自测通过后合并回 main；
- `fix/xxx`：问题修复，修复后合并。

### 8.2 提交信息规范

| 前缀 | 含义 |
|---|---|
| `feat(模块):` | 新功能 |
| `fix(模块):` | 修复缺陷 |
| `docs:` | 文档变更 |
| `refactor:` | 重构（不改功能） |
| `perf:` | 性能优化 |

示例：`feat(ota): HTTP 分块下载写入 B 分区`

### 8.3 版本发布流程

1. `main` 分支编译并烧录验证通过；
2. 打标签：`git tag -a v1.0 -m "版本说明"`；
3. 推送标签：`git push origin v1.0`；
4. 编译产物 `firmware.bin` 作为 Release 附件上传，并记录固件 CRC32 与版本号。

### 8.4 配置与凭据

`upload/wifi_config.h` 中的 Wi-Fi / MQTT 凭据已用占位符替代，实际使用时填写本机配置，**不要提交真实密码**。
