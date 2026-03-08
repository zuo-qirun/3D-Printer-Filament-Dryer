# 3D 打印耗材烘干箱（ESP32-S3）

本项目用于四盘位 3D 打印耗材烘干箱控制。  
核心方案：`PTC 加热片 + 12V 风扇 + AHT10 温湿度传感器 + OLED + ESP32-S3`，实现温湿度监测、闭环控温、任务计时和本地/网页控制。

## 程序介绍

### 1. 核心功能

- 温湿度采集：AHT10（I2C `0x38`）
- OLED 显示：SSD1306（8-bit 地址 `0x78`，等效 7-bit `0x3C`）
- PID 自动调温：按目标温度闭环控制加热输出
- PID 自动校准：继电振荡法（Auto Tune）
- 安全保护：传感器故障、超温故障
- 故障清除：本地菜单、串口、网页均可 `faultreset`
- 材料预设：内置常见耗材温度/时长参数
- 用户自定义：目标温度、时长、空闲风扇转速可改
- 断电恢复：保存运行状态，重启后可继续任务
- 数据记录：SPIFFS 中持续写入 `dryer_log.csv`

### 2. 控制方式

- 本地五键：上/下/左/右/中
- 串口命令：`help/start/stop/status/faultreset/autotune/...`
- 网页控制（局域网）：
  - 实时状态与曲线（温度/湿度/风扇/加热）
  - 修改目标温度、时长、参数模式、PID
  - 启动/停止/故障清除/PID 校准
  - Wi-Fi 配置（STA）
- 开发人员选项（网页隐藏入口）：
  - 连续点击标题 5 次显示/隐藏
  - 支持发送模拟串口指令并查看回显
  - 支持虚拟五键（上/下/左/右/中）

### 3. 网络逻辑

- 若已保存 Wi-Fi：优先 STA 联网
- 未保存或联网失败：自动进入 AP 配网模式
  - AP SSID：`Dryer-Config`
  - AP 密码：`12345678`
  - 默认 IP：`192.168.4.1`
- OLED 会显示当前网络状态，避免黑屏

## 硬件需求

### 1. 主控与外设

- ESP32-S3 开发板（示例：`esp32-s3-devkitm-1`）
- AHT10 温湿度传感器（I2C）
- 0.96" OLED（SSD1306 I2C）
- 五向按键模块（或 5 个独立按键）

### 2. 执行器与电源

- 12V PTC 加热片（示例：100W）
- 12V 风扇（支持 PWM 调速更佳）
- 逻辑级 N 沟道 MOSFET（加热低边开关）
- 电源：
  - 12V 电源（满足加热片与风扇电流）
  - ESP32 供电（可由 5V/USB 或降压模块提供）

### 3. 推荐电气元件

- MOSFET Gate 串联电阻：约 `100R`
- MOSFET Gate 下拉电阻：约 `10k` 到 GND
- 端子台、保险丝、线材、必要散热措施
- 强烈建议：独立过温保护（温控开关/热熔断）

### 4. 引脚连接（当前程序默认）

- I2C SDA：`GPIO8`（AHT10 + OLED）
- I2C SCL：`GPIO9`（AHT10 + OLED）
- 加热控制：`GPIO4`（MOSFET Gate）
- 风扇 PWM：`GPIO5`
- 按键：
  - 上：`GPIO10`
  - 下：`GPIO11`
  - 左：`GPIO12`
  - 右：`GPIO13`
  - 中：`GPIO14`

> 注意：ESP32 GND 与 12V 侧 GND 必须共地。

## 使用方法

### 1. 编译与烧录

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

### 2. 上电后的首次使用

1. 上电后设备尝试连接已保存 Wi-Fi。  
2. 若未连接成功，设备会自动开启 AP 配网热点。  
3. 手机/电脑连接 `Dryer-Config`，访问 `192.168.4.1`。  
4. 在网页输入路由器 SSID/密码并保存。  
5. 联网成功后，通过设备 IP 访问控制页面。

### 3. 本地按键操作

- 主界面：
  - 中键：进入菜单
  - 左/右：切换温度/湿度显示页
  - 上/下：微调目标温度
- 菜单项：
  - 启动/停止
  - 目标温度
  - 烘干时长
  - 空闲风扇
  - 自定义参数
  - 材料预设
  - PID 参数
  - PID 自校准
  - 清除故障

### 4. 串口命令

- `help`
- `start`
- `stop`
- `preset <name>`（如 `preset PLA`）
- `status`
- `faultreset`
- `autotune`
- `wifistatus`
- `wifiap`
- `wificlear`
- `btn <up|down|left|right|ok>`（模拟按键）

### 5. 网页端使用

- 首页查看实时状态与曲线
- 控制区可设置：
  - 参数模式（`user` / 材料预设）
  - 目标温度
  - 烘干时长
  - 空闲风扇转速
- PID 区可保存参数、恢复默认、自动校准
- 可执行启动、停止、故障清除

## 数据与文件

- 日志文件：`/dryer_log.csv`（SPIFFS）
- 主要字段：
  - `ms,temp_c,humidity,target_c,heater_pct,fan_pct,active,fault,preset,remaining_s`

## 目录结构

```text
.
|- src/main.cpp
|- src/dryer_common.h
|- src/pid_autotune.h
|- platformio.ini
`- README.md
```

