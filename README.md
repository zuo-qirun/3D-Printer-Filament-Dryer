# 3D 打印耗材烘干箱（ESP32-S3）

一个基于 ESP32-S3 的四盘位耗材烘干箱项目。系统通过 PTC 加热片 + 12V 风扇形成热风循环，结合温湿度闭环控制，实现稳定烘干。

## 主要功能

- 四盘位耗材烘干（目标支持 4 卷料同时烘干）
- AHT10 温湿度采集（I2C `0x38`）
- OLED 实时显示（I2C `0x78`，即 7-bit `0x3C`）
- PID 自动调温（目标温度可调）
- PID 自动校准（继电振荡法）
- 风扇策略控制（按工况动态调速）
- 安全保护（传感器故障/超温）
- 断电恢复（NVS 持久化）
- 数据记录（SPIFFS `dryer_log.csv`）
- 五键菜单（上/下/左/右/中）

## 硬件接线

### 1) 传感器与显示

- `GPIO8` -> I2C SDA（AHT10 SDA + OLED SDA）
- `GPIO9` -> I2C SCL（AHT10 SCL + OLED SCL）
- `3V3` -> AHT10 VCC
- `GND` -> AHT10 GND + OLED GND

### 2) PTC 加热（12V，MOSFET 低边）

- `GPIO4` -> MOSFET Gate（建议串 `100R`）
- Gate -> `10k` 下拉到 GND
- `12V+` -> PTC+
- `PTC-` -> MOSFET Drain
- MOSFET Source -> GND
- 必须共地：ESP32 GND 与 12V GND

### 3) 风扇（12V 四线）

- `GPIO5` -> 风扇 PWM 控制输入
- 风扇供电使用 12V
- 风扇 GND 与 ESP32 共地

### 4) 五键按键（默认低电平按下，`INPUT_PULLUP`）

- 上键：`GPIO10`
- 下键：`GPIO11`
- 左键：`GPIO12`
- 右键：`GPIO13`
- 中键：`GPIO14`

## 菜单与按键逻辑

- 主界面
- `中键` 进入菜单
- `左/右` 切换温度/湿度主页显示
- `上/下` 快速微调目标温度

- 菜单项
- 启动/停止
- 目标温度
- 烘干时长
- 材料预设（已扩展为 Bambu 指南常见材料）
- PID 参数（Kp/Ki/Kd）
- PID 自校准
- 清除故障

## 已内置耗材预设（基于 Bambu 指南）

- PLA（55C / 8h）
- PETG_HF（65C / 8h）
- ABS（80C / 8h）
- ABS_GF（80C / 8h）
- ASA（80C / 8h）
- PC（80C / 8h）
- TPU95A_HF（70C / 8h）
- PLA_CF（55C / 8h）
- PETG_CF（65C / 8h）
- PET_CF（80C / 10h）
- PAHT_CF（80C / 10h）
- PA6_CF（80C / 10h）
- PA6_GF（80C / 10h）
- PPA_CF（100C / 10h）
- PPS_CF（100C / 10h）

## PID 自动校准说明

当前实现为继电振荡法（Relay Auto-Tune）：

- 在目标温度附近做上下阈值振荡
- 采集振荡幅值和周期
- 估算临界参数并自动生成 `Kp/Ki/Kd`

建议：

- 校准时箱体尽量保持稳定（避免开盖、强气流扰动）
- 校准前让系统先预热到接近目标温度

## 串口命令

- `help`
- `start`
- `stop`
- `preset <PLA|PETG|ABS|NYLON>`
- `status`
- `faultreset`
- `autotune`

## 数据记录

日志文件：`/dryer_log.csv`（SPIFFS）

字段：

- `ms`
- `temp_c`
- `humidity`
- `target_c`
- `heater_pct`
- `fan_pct`
- `active`
- `fault`
- `preset`
- `remaining_s`

## 代码结构

```text
.
|- src/main.cpp
|- src/dryer_common.h
|- src/pid_autotune.h
|- platformio.ini
`- README.md
```

## 构建与烧录

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## 建议烘干温度参考（需实测修正）

- PLA: 45-55 C
- PETG: 60-65 C
- ABS: 65-75 C
- NYLON: 70-85 C

> 注意：不同品牌耗材耐温差异较大，请从较低温度开始验证。
