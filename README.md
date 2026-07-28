# StarrySky C1 蓝牙遥控小车固件

基于 **retroSoC (PicoRV32, RV32IMAC, 72 MHz)** 的蓝牙串口遥控小车，使用 **ECOS SDK v2.0** 开发，通过 **HP_UART** 连接蓝牙模块接收手机指令，PWM 驱动舵机转向 + 双电机差速驱动。

---

## 目录

- [1. 硬件准备](#1-硬件准备)
- [2. 开发环境](#2-开发环境)
- [3. 项目结构](#3-项目结构)
- [4. 引脚映射](#4-引脚映射)
- [5. 编译烧录](#5-编译烧录)
- [6. 蓝牙协议](#6-蓝牙协议)
- [7. 代码架构](#7-代码架构)
- [8. 安全与调试](#8-安全与调试)
- [9. 常见问题](#9-常见问题)
- [10. 版本历史](#10-版本历史)

---

## 1. 硬件准备

### 1.1 所需硬件

| 组件 | 型号/规格 | 数量 |
|------|----------|------|
| 主控板 | StarrySky C1 (retroSoC @ 72 MHz) | 1 |
| 蓝牙模块 | HC-05 / HC-06 (3.3V, 115200) | 1 |
| 舵机 | SG90 / MG90S 等标准 50Hz 舵机 | 1 |
| 电机驱动 | TB6612 / L298N 等双路 H 桥 | 1 |
| 直流电机 | 带编码器/不带均可 | 2 |
| 电池/电源 | 5V / 3.3V 供电 | 1 |
| Type-C 线 | 数据线 (烧录 + 调试串口) | 1 |
| HFP-LINK | 烧录器 (随板附带) | 1 |

### 1.2 接线总图

```
                      ┌─────────────────────┐
                      │   StarrySky C1      │
                      │                     │
 舵机 ◄──PWM_CH0─────┤ PWM_CH0 (CR0)       │
                      │                     │
 左电机PWM ◄─PWM_CH1─┤ PWM_CH1 (CR1)       │
 左电机DIR1 ◄─GPIO_0─┤ GPIO_0              │
 左电机DIR2 ◄─GPIO_1─┤ GPIO_1              │
                      │                     │
 右电机PWM ◄─PWM_CH2─┤ PWM_CH2 (CR2)       │
 右电机DIR1 ◄─GPIO_2─┤ GPIO_2              │
 右电机DIR2 ◄─GPIO_3─┤ GPIO_3              │
                      │                     │
 蓝牙TX ◄────────────┤ HP_UART_RX          │
 蓝牙RX ────────────►┤ HP_UART_TX          │
 蓝牙VCC ────────────┤ 3.3V                │
 蓝牙GND ────────────┤ GND                 │
                      │                     │
 USB(Type-C) ◄───────┤ SYS_UART (CP2102)   │ ◄── 调试串口 115200
                      │                     │
 烧录器(HFP-LINK) ◄──┤ QSPI Flash          │ ◄── 烧录固件
                      └─────────────────────┘
```

### 1.3 电机 H 桥接线

以 TB6612 为例：

| TB6612 引脚 | 接 C1 | 接电机 |
|-------------|-------|--------|
| PWMA | PWM_CH1 (左轮速度) | - |
| AIN1 | GPIO_0 (左轮方向) | - |
| AIN2 | GPIO_1 (左轮方向) | - |
| AO1/AO2 | - | 左电机 |
| PWMB | PWM_CH2 (右轮速度) | - |
| BIN1 | GPIO_2 (右轮方向) | - |
| BIN2 | GPIO_3 (右轮方向) | - |
| BO1/BO2 | - | 右电机 |
| VM | 电池/电源 | - |
| VCC | 3.3V | - |
| GND | GND | - |

### 1.4 舵机接线

| 舵机线 | 颜色 | 接 C1 |
|--------|------|-------|
| 信号 (PWM) | 橙/白 | PWM_CH0 |
| 电源 VCC | 红 | 5V (外部供电) |
| 地 GND | 棕/黑 | GND (共地) |

---

## 2. 开发环境

### 2.1 系统要求

- Ubuntu 24.04 LTS (推荐) / WSL2
- 已安装的 ECOS SDK v2.0

### 2.2 环境检查

```bash
# 确认 SDK 安装
ls -la $HOME/.local/ecos-sdk

# 确认交叉编译器
export PATH="$HOME/.local/ecos-sdk/toolchain/riscv/bin:$PATH"
riscv64-unknown-elf-gcc --version

# 确认 SDK 环境变量
export ECOS_SDK_HOME="$HOME/.local/ecos-sdk"
```

### 2.3 项目路径

```
/home/ysl/OSOC_study/
├── Main-Program_v2.6/         ← STM32 原始工程（供参考）
└── starry/                    ← C1 工程（当前使用）
    ├── main.c                 ← 蓝牙遥控主程序
    ├── main.h                 ← 板级头文件
    ├── Makefile               ← 构建脚本
    ├── start.s                ← RISC-V 启动代码
    ├── sections.lds           ← 链接脚本
    ├── board.h                ← 寄存器地址定义
    ├── configs/.config        ← Kconfig 配置
    ├── scripts/               ← 构建辅助
    └── build/                 ← 编译输出
        └── retrosoc_fw.bin    ← 固件（最终产物）
```

---

## 3. 项目结构

```
starry/
├── main.c                # 主程序：蓝牙命令解析 + 电机/舵机驱动
├── main.h                # 统一头文件（包含所有 ECOS HAL/组件头文件）
├── start.s               # RISC-V 启动代码：清零寄存器 → 初始化 data/bss → call main
├── sections.lds          # 链接脚本：FLASH@0x30000000, PSRAM@0x04000000
├── board.h               # 寄存器地址：GPIO/PWM/UART/Timer/I2C/QSPI
├── Makefile              # 构建系统：CROSS=riscv64-unknown-elf-
├── configs/
│   ├── .config           # Kconfig 配置（已使能 PWM/GPIO/HP_UART/Timer）
│   └── generated/        # 自动生成的配置头文件
├── scripts/              # Kconfig 工具 + 内存报告
└── build/                # 构建产物目录
    └── retrosoc_fw.bin   # 烧录固件
```

关键文件说明：

| 文件 | 作用 |
|------|------|
| `main.c` | **唯一需要修改的源文件**。主循环：HP_UART 轮询 → 命令解析 → motor_apply |
| `main.h` | **包含所有 SDK 头文件的入口**，新增外设时在此添加 include |
| `configs/.config` | 外设驱动开关，用 `make menuconfig` 修改 |
| `start.s` | 不需要修改，标准 RISC-V 启动 |
| `sections.lds` | 不需要修改，标准链接脚本 |

---

## 4. 引脚映射

| C1 外设 | 功能 | 映射寄存器/引脚 |
|---------|------|----------------|
| **PWM_CH0** | 舵机转向 | `REG_PWM_0_CR0` (0x03004010) |
| **PWM_CH1** | 左轮速度 | `REG_PWM_0_CR1` (0x03004014) |
| **PWM_CH2** | 右轮速度 | `REG_PWM_0_CR2` (0x03004018) |
| **GPIO_0** | 左轮方向 1 | `REG_GPIO_0_DR` bit 0 |
| **GPIO_1** | 左轮方向 2 | `REG_GPIO_0_DR` bit 1 |
| **GPIO_2** | 右轮方向 1 | `REG_GPIO_0_DR` bit 2 |
| **GPIO_3** | 右轮方向 2 | `REG_GPIO_0_DR` bit 3 |
| **SYS_UART** | 调试输出 (CP2102) | `REG_UART_0_DATA`, 115200 |
| **HP_UART** | 蓝牙通信 | `REG_UART_1_TRX`, 115200 |
| **PWM_CLK/PSCR** | PWM 时基 | `pscr=71` → 1MHz 计数时钟 |
| **PWM_CMP** | PWM 周期 | `cmp=20000` → 50Hz 输出 |

### PWM 时序

```
CPU 72 MHz
   │
   ├── pscr=71  ──►  1 MHz (计数时钟)
   │                    │
   │                    ├── cmp=20000  ──►  50 Hz (PWM 周期)
   │                    │                   │
   │                    │                   ├── CH0 (CR0=1500)  ──► 舵机中位 (1.5ms)
   │                    │                   ├── CH1 (CR1=0~20000) ──► 左轮速度
   │                    │                   └── CH2 (CR2=0~20000) ──► 右轮速度
```

---

## 5. 编译烧录

### 5.1 编译

```bash
cd /home/ysl/OSOC_study/starry

# 设置环境变量
export PATH="$HOME/.local/ecos-sdk/toolchain/riscv/bin:$PATH"
export ECOS_SDK_HOME="$HOME/.local/ecos-sdk"

# 清理并编译
make clean && make
```

成功输出示例：

```
Building retrosoc_fw...
Linking retrosoc_fw...
Post-processing retrosoc_fw.hex...
Generating retrosoc_fw.bin...
Generating retrosoc_fw.txt...
Memory Usage:
  FLASH: 1.83 KB / 16.00 MB  (0.01%)
  PSRAM: 40 B  / 8.00 MB     (0.00%)
Done.
```

产物位于 `build/retrosoc_fw.bin`。

### 5.2 烧录 (HFP-LINK)

```
1. FLASH_SEL 拨到 MCU
2. Type-C 连接电脑（同时供电）
3. HFP-LINK 作为 U 盘出现
```

```bash
# 复制固件到烧录器
cp build/retrosoc_fw.bin /media/$USER/YSYX-HFPLnk/

# 同步卸载（触发烧录）
sync
umount /dev/sdb

# 重新挂载验证
udisksctl mount -b /dev/sdb
cat /media/$USER/YSYX-HFPLnk/STATE.TXT
# → write successful !!!
```

### 5.3 启动运行

```
1. 断电
2. FLASH_SEL 拨到 CHIP
3. 重新上电
4. 按 RESET 键
5. 打开串口终端
```

```bash
sudo cat /dev/ttyUSB0
# 输出: C1 Bluetooth Car Boot OK
# 输出: Bluetooth ready
```

### 5.4 烧录疑难解答

| 问题 | 解决 |
|------|------|
| STATE.TXT 显示 "copy binary..." | `umount /dev/sdb` → `udisksctl mount -b /dev/sdb` 再看 |
| HFP-LINK 不识别 | `udisksctl power-off -b /dev/sdb` → 物理拔插 USB |
| 挂载点 busy | `umount -l /media/.../YSYX-HFPLnk` 后重试 |
| 无 /dev/ttyUSB0 | `lsmod \| grep cp210x` → 若无则 `sudo modprobe cp210x` |

> **Windows/WSL2 下最稳妥**: 直接拖拽 `retrosoc_fw.bin` 到 HFP-LINK 盘符。

---

## 6. 蓝牙协议

### 6.1 连接

| 参数 | 值 |
|------|-----|
| 串口 | HP_UART (C1) |
| 波特率 | **115200** |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |

手机安装蓝牙串口 APP（如「蓝牙调试器」「Serial Bluetooth Terminal」），搜索配对蓝牙模块（HC-05 默认密码 `1234`）。

### 6.2 控制指令

单字符命令，大小写均可：

| 命令 | 动作 | 效果 |
|------|------|------|
| `W` / `w` | **前进** | 舵机回中，双轮正转 |
| `S` / `s` | **停车** | 舵机回中，双轮停止 |
| `A` / `a` | **左转** | 舵机左打，左轮减速 |
| `D` / `d` | **右转** | 舵机右打，右轮减速 |
| `X` / `x` | **后退** | 舵机回中，双轮反转 |
| `+` | **加速** | 基准速度 +200，上限 4800 |
| `-` | **减速** | 基准速度 -200，下限 200 |

加速/减速时蓝牙会回传 `SPD+ ` / `SPD- ` 确认。

### 6.3 速度映射关系

```
概念速度 (Std_Speed)     PWM 占空比     等效原 STM32
    200  ─────►  833  (4.2%)    约 200
    650  ─────►  2708 (13.5%)   约 650 (默认)
   1000  ─────►  4167 (20.8%)   约 1000
   2000  ─────►  8333 (41.7%)
   4800  ─────►  20000 (100%)   最大
```

### 6.4 自定义协议

如需扩展指令，在 `main.c` 的 `switch(cmd)` 中添加 case：

```c
case 'T': // 新增：鸣笛
    buzzer_on();
    break;
```

---

## 7. 代码架构

### 7.1 main.c 详解

```
main()
│
├── hal_sys_uart_init()        ← 初始化调试串口 (CP2102)
├── gpio_set_direction() x4    ← 初始化电机方向 GPIO
├── pwm_init()                 ← 初始化 PWM (50Hz, 所有通道)
├── hal_hp_uart_init()         ← 初始化蓝牙串口 (115200)
│
└── while (1) 主循环
    │
    ├── hp_uart_data_ready()   ← 非阻塞检查蓝牙是否收到数据
    │   └── hal_hp_uart_recv() ← 读取一个字节
    │       └── switch(cmd)    ← 解析命令 → 修改 Wheel_Left/Right_Speed + SteerPWM
    │
    ├── motor_apply()          ← 执行电机/舵机输出
    │   ├── speed_to_pwm()     ← 速度值 → PWM 占空比
    │   ├── gpio_set_level()   ← 设置方向引脚
    │   └── pwm_set_compare()  ← 设置 PWM 比较值
    │
    └── hal_delay_ms(0, 10)   ← 10ms 控制周期
```

### 7.2 核心函数说明

| 函数 | 作用 |
|------|------|
| `speed_to_pwm(spd)` | 将 STM32 兼容的速度值 (0-4800) 映射到 C1 的 PWM 值 (0-20000) |
| `hp_uart_data_ready()` | 读取 HP_UART LSR 寄存器 bit 7，判断是否有数据可读（非阻塞） |
| `motor_apply()` | 根据全局速度/舵机值，一次性更新所有 GPIO 和 PWM 输出 |

### 7.3 驱动调用链

```
main.c 调用          组件层 (components/)       HAL 层 (hal/)             寄存器 (board.h)
──────────           ──────────────────        ──────────────────         ─────────────────
pwm_init()      →    pwm.c::pwm_init()    →    hal_pwm.c::pwm_hal_init() → REG_PWM_0_*
pwm_set_compare() →  pwm.c::pwm_set_cmp() →    hal_pwm.c::pwm_hal_set()  → REG_PWM_0_CRx
gpio_set_level() →    gpio.c::gpio_set_lev() →  hal_gpio.c::gpio_hal_set()→ REG_GPIO_0_DR
hal_hp_uart_*()  →    (直接 HAL)            →    (直接寄存器操作)          → REG_UART_1_*
hal_delay_ms()   →    timer.c::timer_delay() →   hal_timer.c::hal_delay() → REG_TIM_0_*
```

---

## 8. 安全与调试

### 8.1 安全机制

| 机制 | 说明 |
|------|------|
| **速度限幅** | 内部限幅 `±4800`（概念值），`motor_apply` 会二次限幅至 `±20000`（PWM） |
| **舵机限幅** | 概念限制 `±85` 范围内，实际限制 `PWM 500~2500` |
| **默认停车** | 上电默认 Wheel_Left/Right_Speed = 0，电机不转 |
| **逐周期更新** | 主循环 10ms 周期，未收到指令时保持上一状态 |

### 8.2 调试串口输出

CP2102 (SYS_UART) 通过 Type-C 输出调试信息：

```
C1 Bluetooth Car Boot OK     ← 系统启动
Bluetooth ready              ← 蓝牙初始化完成
```

如需添加调试输出：

```c
char buf[32];
sprintf(buf, "Speed=%d\n", Std_Speed);
hal_sys_putstr(buf);
```

### 8.3 关键调参位置

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| SERVO_CENTER | `main.c` L8 | 1500 | 舵机中位脉宽 (50Hz 下 1.5ms) |
| SERVO_LEFT/RIGHT | `main.c` L9-10 | 900/2100 | 左右极限 |
| Std_Speed 默认值 | `main.c` L27 | 650 | 上电默认速度 |
| 速度步长 (+/-) | `main.c` L106/111 | 200 | 每按一次加减的量 |
| 控制周期 | `main.c` L119 | 10ms | hal_delay_ms(0, 10) |

---

## 9. 常见问题

### 9.1 电机不转

- 检查电机驱动板供电（VM 是否接电池？）
- 检查使能引脚（H 桥的 EN 脚是否拉高？）
- 检查蓝牙模块是否配对成功（手机 APP 显示已连接）
- 发 `W` 命令后有没有收到蓝牙回传？没有 → 蓝牙连接问题

### 9.2 舵机不工作 / 抖动

- 舵机需要 **5V 外部供电**，不要从 C1 的 3.3V 取电
- C1 与舵机必须**共地**
- 50Hz PWM 是否正确？用示波器量 PWM_CH0 引脚
- 中值 1500 是否匹配舵机型号？（部分舵机中值在 1520）

### 9.3 编译错误

```
undefined reference to `__muldf3'
```
→ 代码中出现了浮点运算。C1 的 RV32IMAC 没有 FPU，不得使用 `float`/`double`。

### 9.4 蓝牙连接不上

- 模块供电是否为 3.3V？（HC-05 可接受 5V，HC-06 可能不行）
- 波特率是否匹配？（C1 使用 115200，部分模块默认 9600，需 AT 指令修改）
- 手机 APP 是否配对成功？

### 9.5 前进/后退方向反了

调换电机的两根线，或者反转 `main.c` 中 `motor_apply()` 里的 GPIO 电平逻辑。

---

## 10. 版本历史

| 版本 | 日期 | 作者 | 说明 |
|------|------|------|------|
| v1.0 | 2026.07 | YSL | 首次发布：C1 蓝牙遥控小车，ECOS SDK v2.0 |
| - | 2025.11 | YSL | STM32 循迹版本 (Main_V2.8) |

---

> **参考链接**
> - ECOS SDK: https://github.com/openecos-projects/embedded-sdk
> - C1 文档: https://embedded.openecos.com/zh-cn/latest/page/brd/starry-sky-c/v1.0/
> - SDK 快速入门: https://embedded.openecos.com/zh-cn/latest/page/sdk/common/quickstart/
