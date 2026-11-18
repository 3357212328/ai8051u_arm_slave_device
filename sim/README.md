# Keil 软件仿真（Simulator）说明

本目录用于在没有 AI8051U 实物板时，用 Keil uVision 自带的 **软件仿真器** 运行、单步调试本工程。

相关文件：

| 文件 | 作用 |
| --- | --- |
| `debug_sim.ini` | 调试启动时自动执行的命令脚本（映射 XFR 区、加 Watch、停在 `main`、启动 Timer0） |
| `ai8051u_templete.uvproj` | Debug 页已选 `Use Simulator`，初始化文件指向 `.\sim\debug_sim.ini` |

---

## 一、怎么开始

1. 打开 `ai8051u_templete.uvproj`。
2. `Project → Rebuild all target files`（或运行 `build.bat`），确认 0 Error / 0 Warning。
3. `Debug → Start/Stop Debug Session`（`Ctrl+F5`）。
   - 仿真器会加载 `Objects\ai8051u_templete`，按 `debug_sim.ini` 执行命令，
     **最终停在 `main()` 第一行**（`main.c` 约 188 行，PC≈`0x0FF0046`）。
4. 按 `F5`（Run）继续，程序会跑完初始化并进入主循环（每轮都会调用 `draw_ui()`）。
   > ⚠️ 仿真器没有 USB 模型，直接跑会卡在 `usb_init()`；
   > 请先按第三节第 5 条在 `uart.h` 打开 `UART_SIM_SKIP_USB` 再编译。
5. 需要看变量时：`View → Watch Windows → Watch 1`。
   初始化文件已经加入 5 个常用表达式：
   `current_mode`、`g_sys_tick_count`、`active_servo_count`、`servo_angles[0]`、`OLED_DisplayBuf[0][0]`。

> 初始化文件里的断点、Watch 每次进入调试都会重新设置，重新编译后地址变了也不用改（全部用符号，不写死地址）。

## 二、debug_sim.ini 做了什么

```text
MAP 0x7EF000, 0x7EFFFF READ WRITE   ; STC 的 XFR(far) 区默认只读，改成可读写
WS current_mode                     ; 加 Watch 表达式
...
BS main                             ; 在 main 下断点
G                                   ; 运行到 main
TMOD = 0x01                         ; 手工配置 Timer0（见第三节第 1 条）
TH0 = 0x00
TL0 = 0x00
TCON = 0x10                         ; TR0 = 1，启动定时器
```

- `MAP` 必须写：`HIRCCR`、`CLKDIV`、`EAXFR` 等 STC 扩展寄存器在 far 地址
  `0x7EFE00` 一带，仿真器默认没有写权限，不加这一行会在 `Clock_Init()` 里报
  `*** error 65: access violation ... no 'write' permission`。
  注意写法是 `MAP 起始, 结束 READ WRITE`，**`READ WRITE` 前面没有逗号**，否则报语法错误。
- `BS main` + `G`：让仿真一启动就停在 `main`，而不是直接跑飞。

## 三、仿真器的限制（重要）

本工程大量使用 STC 独有外设，Keil 的通用 251 仿真器只模拟 **CPU 内核 + 标准 8051 定时器/串口**，
下列内容**没有模型**，调试时要心里有数：

1. **TFPU / MDU32 硬件浮点、硬件乘除**
   仿真器不认识 STC 的协处理器指令，浮点结果不可靠。
   直接影响：`set_timer_mode(Timer0, "1ms", ...)` 内部用浮点计算重装值，仿真中会算失败、
   定时器不会被真正启动。所以 `debug_sim.ini` 在 `main` 处手工写了
   `TMOD/TH0/TL0/TCON` 把 Timer0 跑起来。
   - 仿真中 1 个 `g_sys_tick_count` ≈ **2~4 ms**（真机 1 ms），
     报警计时、播放延时、平滑移动等"按 tick 计时"的功能会**偏慢 2~4 倍**。
   - 其它依赖浮点的计算（速度倍率、角度插值等）在仿真中可能不生效或数值异常，
     这些逻辑请以真机为准。
2. **OLED（SSD1306，软件 I2C）**
   没有显示屏模型，`OLED_Update()` 只是把数据写进 `OLED_DisplayBuf`，仿真中**看不到画面**。
   验证界面内容：Watch 里展开 `OLED_DisplayBuf`（4 页 × 128 列），
   或 `View → Memory Windows → Memory 1`，地址填 `OLED_DisplayBuf`。
3. **PCA9685（硬件 I2C0）**
   没有 I2C 从机模型，每次事务都会跑满 `I2C_TIMEOUT`（20000 次轮询）后由
   `set_i2c_recover()` 跳过。启动阶段会因此多花约 0.2 s 仿真时间，属正常现象。
   舵机不会动，`servo_angles[]` 仍然会按逻辑更新。
4. **HIRCCR / IRC48MCR 稳定位、`T4IF` 等 XFR 状态位**
   仿真器不会置位。`Clock_Init()` 已改成带超时的等待（见 `main.c`），
   否则仿真会永远卡在 `while (!(HIRCCR & 1));`。
5. **USB 外设（P3.0/P3.1）**
   仿真器完全没有 USB 模型：`usb_init()` 里写 `USBADR` 后，其 bit7 忙位
   永远不会清零，`while (USBADR & 0x80);` 会**死循环**（`IRC48MCR` 稳定位
   也不会置位）。所以先用 Keil 仿真调试时，要在 `uart.h` 里把

   ```c
   //#define UART_SIM_SKIP_USB
   ```

   这一行的注释去掉、重新编译（跳过 `usb_init()`），调试完再改回来。
   此时 `DeviceState` 恒为 0，USB 发送自动跳过、USB 接收永远无数据，
   其余逻辑（菜单、按键注入、播放等）照常仿真。
6. **DHT11、编码器、按键、蜂鸣器、DMA、串口（UART4）收发**
   都是纯 GPIO / 外设行为，仿真中读回固定电平：
   - DHT11 读失败，温湿度保持 0；
   - 编码器计数为 0，按键永远返回"无按键"；
   - UART4 发送只写 DMA 缓冲，收不到上位机数据；
     因此 `key <...>` / `enc <n>` 远程控制命令只能在真机验证。
   界面切换、录制/播放流程适合用"改 Watch 变量 + 单步"的方式验证，
   例如直接把 `current_mode` 改成某个模式值，再单步看 `draw_ui()` 的分支。

## 四、常用调试手法

| 目的 | 操作 |
| --- | --- |
| 停在某个函数 | 命令窗口输入 `BS 函数名`，或直接在源码行号左边双击 |
| 看调用栈 | `View → Call Stack + Locals` |
| 改变量后继续 | Watch 窗口里双击 Value 直接改；或在命令窗口输入 `current_mode = 2` |
| 看某段内存 | `View → Memory Windows`，地址填符号名（如 `OLED_DisplayBuf`） |
| 重新从头跑 | `Debug → Reset`，再 `F5`（初始化文件不会重跑；需要重跑请 `Stop` 后 `Ctrl+F5`） |

## 五、改回"真机在线调试"

`Options for Target → Debug`：

1. 左侧选 **Use: STC Monitor-51 Driver**（`BIN\STCMON251.DLL`），
   `TargetSelection` 相应改回 1；
2. 把 **Initialization File** 清空（`MAP/BS/G` 这些命令是给仿真器用的，
   硬件调试时不需要，留着可能报错）；
3. 用 STC-ISP 的"串口助手/下载"占用串口时，Keil 连不上会报
   `Port Error: The selected communication port is not installed !`，
   先关掉 STC-ISP 或换一个未被占用的串口。

> 当前仓库里的 `ai8051u_templete.uvopt` 被 `.gitignore` 忽略，
> 所以"Use Simulator / 初始化文件路径"这类设置在 `uvproj` 里；
> 本机若改过 Debug 页，记得同步改回，避免下次打开又进仿真。
