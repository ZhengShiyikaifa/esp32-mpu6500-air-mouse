# ESP32 + MPU6050/MPU6500 蓝牙体感鼠标

适用开发板：照片中的 30 针 ESP32 DevKit V1 / ESP32-WROOM-32 类开发板。

## 接线

| MPU6050 | ESP32 | 说明 |
|---|---|---|
| VCC | 3V3 | 建议使用 3.3V |
| GND | GND | 共地 |
| SDA | D21 / GPIO21 | I²C 数据 |
| SCL | D22 / GPIO22 | I²C 时钟 |
| AD0 | GND 或悬空 | 地址保持 0x68 |
| INT | 不接 | 本程序不使用中断 |

可选鼠标按键：

- 左键：按钮接在 GPIO25 与 GND 之间。
- 右键：按钮接在 GPIO26 与 GND 之间。
- 锁定按钮：按钮接在 GPIO27 与 GND 之间；不按时允许体感移动，按住时锁定鼠标。
- 程序启用了内部上拉电阻，不需要额外电阻。

灵敏度旋转编码器：

| 编码器 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| A | D32 / GPIO32 |
| B | D33 / GPIO33 |
| C（旋转公共端） | GND |

- 每旋转一格，灵敏度改变 100。
- 可调范围为 300–4000，开机默认值为 1050。
- 串口中的 `gain=` 是当前灵敏度，旋转时还会输出 `SENS` 信息。
- 如果顺时针和逆时针的增减方向不符合习惯，把代码中的 `ENCODER_DIRECTION` 从 `1` 改为 `-1`。
- 原来的独立左键继续接在 GPIO25 与 GND 之间。

## 调试串口

程序会把相同的调试信息同时发到两个位置：

1. 板载 USB 串口：Arduino 串口监视器选择 `115200 baud`。
2. UART2：板上的 `TX2/GPIO17` 接 USB-TTL 模块的 `RX`，两者 `GND` 相连，选择 `115200 8N1`。如需向 ESP32 发送数据，可将 USB-TTL 的 `TX` 接到 `RX2/GPIO16`；本程序目前只输出调试信息。

注意：USB-TTL 必须使用 3.3V 逻辑电平，不要把 5V 信号接入 ESP32 引脚。

## Arduino IDE 环境

安装：

- 开发板包：`esp32 by Espressif Systems`
- 库：`ESP32 BLE Mouse`（T-vK）

建议先使用 ESP32 Arduino Core 2.0.17；部分旧版 ESP32 BLE Mouse 与 Core 3.x 的 BLE API 不兼容。

编译设置：

- Board：`DOIT ESP32 DEVKIT V1`（没有该项时用 `ESP32 Dev Module`）
- Upload Speed：`921600`；上传不稳定时改为 `115200`
- 串口监视器：`115200 baud`

## 使用步骤

1. 接线后，把开发板和 MPU6050 平放并保持不动。
2. 上传程序；复位后保持静止。程序只会接受连续稳定的约 3 秒数据；检测到移动会自动重新校准，避免光标持续漂移。
3. 在电脑或手机的蓝牙设置中连接 `ESP32 MPU6050 Air Mouse`。
4. 转动模块，观察鼠标移动和串口中的 `DATA` 信息。

## 调节手感

在 `.ino` 顶部修改以下参数：

- `MOUSE_GAIN`：增大后光标更快。
- `GYRO_DEAD_ZONE`：增大可减少静止漂移，但轻微动作会不灵敏。
- `KALMAN_PROCESS_NOISE`：增大后响应更快，但抖动也会增加。
- `KALMAN_MEASUREMENT_NOISE`：增大后更平滑，但响应会稍慢。
- `DEFAULT_MOUSE_GAIN`：开机默认灵敏度。
- `ENCODER_GAIN_STEP`：编码器每格改变的灵敏度。
- 若左右方向相反，把 `X_DIRECTION` 改为 `-1.0f`。
- 若上下方向相反，把 `Y_DIRECTION` 改为 `-1.0f`。

当前安装方向使用 X 轴控制左右、Z 轴控制上下。如果传感器在外壳中的朝向不同，可交换代码中的 `filteredX` 与 `filteredZ` 映射。

## 串口输出示例

```text
CAL: done, bias(rad/s) X=-0.01123 Y=0.00652 Z=-0.00801
BLE: advertising; pair with 'ESP32 MPU6050 Air Mouse'.
DATA: BLE=1 gyro(rad/s) X=+0.142 Y=-0.003 Z=-0.271 move=(-1,0) btn(L,R)=(0,0)l
```

`BLE=1` 表示蓝牙已连接；`move=(x,y)` 是本周期发送的鼠标位移。

## 已验证的编译结果

本工程已使用以下环境完成实际编译：

- PlatformIO Core 6.1.19
- Espressif32 Platform 6.8.1
- Arduino-ESP32 2.0.17
- 开发板：DOIT ESP32 DEVKIT V1（4 MB Flash）
- ESP32 BLE Mouse 0.3.1

编译结果：RAM 使用 40,092 字节（12.2%），程序 Flash 使用 1,155,605 字节（88.2%）。水平轴和垂直轴均使用独立的一维卡尔曼滤波器平滑陀螺仪角速度。

实物模块在 I²C 地址 `0x68` 返回 `WHO_AM_I=0x70`，实际是 MPU6500 兼容芯片，而不是标准 MPU6050。程序已改为原始寄存器驱动，同时支持 MPU6050、MPU6500、MPU9250 和 MPU9255 的陀螺仪寄存器布局。

生成的 `ESP32_MPU6050_AirMouse_merged.bin` 是合并固件，使用烧录工具时写入地址为 `0x0`。目录中也保留了分离固件：

| 文件 | 写入地址 |
|---|---:|
| `bootloader.bin` | `0x1000` |
| `partitions.bin` | `0x8000` |
| `boot_app0.bin` | `0xE000` |
| `firmware.bin` | `0x10000` |
