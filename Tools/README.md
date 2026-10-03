# 底盘键盘串口控制

在 Linux 终端中运行 `chassis_keyboard.py`，用 WASD 平移、QE 转向，用 Ctrl+L 在角速度和目标角度模式之间切换，终端界面显示当前模式、目标速度、目标角度和发送帧数。默认通过 evdev 读取选定键盘的按下、松开和当前键位，支持连续长按与多键组合，不依赖操作系统的按键自动重复。

Xbox 360 手柄也可使用同一串口协议，脚本为 `chassis_gamepad.py`，Hat 按次调整巡航速度。参见 [手柄使用说明](README_gamepad.md)。

## 安装和启动

依赖已安装在本机 `embedded` Conda 环境中。重新部署时，在工程根目录执行：

```sh
conda activate embedded
python -m pip install -r Tools/requirements.txt
python Tools/chassis_keyboard.py --list-keyboards
python Tools/chassis_keyboard.py --list-ports
```

先选实际使用的键盘。`--list-keyboards` 会显示设备路径、名称及是否可读取；推荐使用 `/dev/input/by-id/...-event-kbd` 路径。

如果显示“需要读取权限”，授予当前用户对这一个键盘事件设备的读取权限。例如，本机的 2.4G Dongle：

```sh
sudo setfacl -m u:"$(id -un)":r /dev/input/by-id/usb-05ac_2.4G_Dongle-event-kbd
```

只需这一步使用 sudo，Python 仍在普通用户的 Conda 环境中运行。设备重新插拔后可能需要再次授权；若该键盘不是你正在使用的键盘，应换成列表中的相应路径。

当前固件已调换串口角色：UART8 接收电脑控制帧，UART7 输出打印。DAP CDC 对应 UART8，USB 转 TTL 对应 UART7：

```sh
python Tools/chassis_keyboard.py --port /dev/ttyACM0 \
  --keyboard /dev/input/by-id/usb-05ac_2.4G_Dongle-event-kbd
```

UART7 的打印可通过 `/dev/ttyUSB0` 的串口终端查看。两端均使用 115200、8N1、无流控。UART8 的 RX 为 PE0、TX 为 PE1；UART7 的 RX 为 PF6、TX 为 PF7。

未指定 `--keyboard` 时，只有一个可读取且支持 WASD/QE/Ctrl+L 的设备才能自动选中；多个设备时需要明确指定。读取权限缺失不会自动退回旧输入模式，程序会提示原因。

没有连接底盘时，可以使用回环串口：

```sh
python Tools/chassis_keyboard.py --port loop:// \
  --keyboard /dev/input/by-id/usb-05ac_2.4G_Dongle-event-kbd
```

在交互式终端中运行，不要将标准输入重定向为文件或管道。evdev 读取的是本机选定的物理键盘。

## 按键与速度

平移速度沿用脚本中的 `LINEAR_SPEED`，当前为 **0.8 m/s**。修改这个常量后，实际指令和界面文字会一同变化。

| 按键 | 指令 |
| --- | --- |
| W / S | 向前 / 向后，vx = ±0.8 m/s |
| A / D | 向左 / 向右，vy = ±0.8 m/s |
| Q / E（角速度模式） | 俯视逆时针 / 顺时针，d_yaw = ±0.5 rad/s |
| Q / E（角度模式） | 按住时以 ±0.5 rad/s 连续调整目标 yaw；松开后保持该目标 |
| Ctrl+L | 切换角速度 / 角度模式，清除并屏蔽已按住的方向键 |
| 空格 / Enter | 立即提交零速度并关闭角度环；已按住的方向键需松开再按才能重新运动 |
| Esc / X / Ctrl+C | 退出，退出前尝试提交零速度帧 |

按住一个键就持续运动，不会在首次自动重复之前停顿。多个方向分别保持：W+A 为左前移动，W+Q 为前进并转向；松开 A 时若 W 仍按住，则继续向前。对角平移合速度仍为 0.8 m/s，两个分量各约 0.566 m/s。同一轴的相反键同时按住时相互抵消；松开其中一个后，剩下的键继续生效。

程序默认使用**角速度模式**。Ctrl+L 切到**角度模式**后，按住 Q 增大目标 yaw、按住 E 减小目标 yaw；默认调整速率 0.5 rad/s，约 28.6°/s。目标根据实际经过的时间变化，不受界面刷新频率和按键自动重复速度影响。松开 Q/E 后继续发送该目标，由主控 IMU 和 yaw PID 保持角度；WASD 平移仍可组合使用。

目标 yaw 初始为 **0°**，使用 IMU 的 yaw 坐标基准；它是绝对目标角度，并不是“进入模式时的当前朝向”。进入角度模式会跟踪界面显示的目标，因此若实际 yaw 与目标不同，底盘会转向该目标。界面显示度数和弧度，线上传输弧度；目标环绕在 ±180° 内，主控 PID 按最短角度误差计算。界面没有实际 yaw 回传。切回角速度模式后保留之前的角度目标，再次进入角度模式会使用该目标。

空格 / Enter 会停止平移和转向，并暂停角度环，避免把停止帧理解为“转回 yaw=0”。目标角度仍保留在界面中，角度模式下重新按方向键后才恢复跟踪；已按住的键必须先松开再按。

固件的 `chassisCmd_t.mode` 为 `HEADLOCK=0` 时使用 yaw PID，为 `HEADFREE=1` 时直接使用 `d_yaw`。角度环每 10 ms 更新，轮速环每 1 ms 更新，间隔内保持最近的角度环输出。保留项目现有 PID 参数；角度模式缺少有效 IMU 数据时输出零速度。**测试角度模式前需要烧录本次编译的固件**，旧固件不识别新增的模式帧。

保持本控制终端在前台。终端支持焦点报告时，切换窗口会提交零速度，并屏蔽仍按住的键直到松开后重新按下；若终端不支持焦点报告，切换前先按空格停止。脚本不会独占键盘，也不会向系统注入按键。

转向速度与角度模式下的目标调整速率可一同指定：

```sh
python Tools/chassis_keyboard.py --port /dev/ttyACM0 --yaw-rate 0.3 \
  --keyboard /dev/input/by-id/usb-05ac_2.4G_Dongle-event-kbd
```

界面速度是发送目标；正常以约 20 Hz 提交控制帧，方向改变、松键和停止时尽快提交新指令。当前协议没有主控命令应答。

## 普通终端兼容模式

没有本机物理键盘读取条件时，可显式使用原来的终端输入：

```sh
python Tools/chassis_keyboard.py --port /dev/ttyACM0 --input terminal --hold-time 0.6
```

该模式也支持 Ctrl+L。它无需读取键盘设备，但只有字符输入和超时，没有真实松键事件；首次重复延迟和多键保持问题仍受终端限制。`--hold-time` 只影响这个模式，不能替代真实按键状态。

读取键盘事件所用的 API 参考 [python-evdev 文档](https://python-evdev.readthedocs.io/en/latest/tutorial.html#reading-events)。

## 发送超时与暂停

一次 `Write timeout` 或一帧只写入部分字节时，界面会显示“控制暂停”，清空三个轴的运动指令，尝试丢弃系统中尚未发出的数据，并以约 4 Hz 重试零速度帧。暂停期间方向键不起作用；串口恢复后按空格或 Enter，成功提交零速度帧后重新启用方向键。不会自动恢复超时前的运动。

即使持续超时，仍可以按 X、Esc 或 Ctrl+C 退出。如果最后的停止帧也无法写入，程序会明确报告失败。设备断开等其他串口错误仍结束程序。

默认单次写入最多等待 0.1 秒。USB 虚拟串口可能有瞬时延迟，可以用 1 秒进行对照：

```sh
python Tools/chassis_keyboard.py --port /dev/ttyACM0 --write-timeout 1
```

更长的超时也会延长写入阻塞时的键盘响应时间，不能解决线路接错或主控未开启接收的问题。

界面的“串口提交”帧数只表示 `write()` 完成，不代表 STM32 已接收或通过 CRC 校验。清理系统发送缓冲区也未必能撤销 USB 设备内部已经缓存的旧指令。当前固件没有命令失联归零逻辑，因此通信故障时电脑不能保证底盘停止。

## 与固件的协议

角速度模式和停止指令沿用原来的 23 字节帧，主控按 `HEADFREE` 解析：

```text
0x72 | 0x65 | vx(float32) | vy(float32) | yaw(float32) | d_yaw(float32) | CRC32 | 0x64
```

角度模式使用新增的 27 字节帧：

```text
0x72 | 0x6d | vx(float32) | vy(float32) | yaw(float32) | d_yaw(float32) | mode(uint32) | CRC32 | 0x64
```

`mode=0` 为 `HEADLOCK`（角度模式），`mode=1` 为 `HEADFREE`（角速度模式）。Python 在角度模式发送 `mode=0`、目标 yaw 和 `d_yaw=0`；主控用 PID 计算实际角速度。新格式也接受 `mode=1`，未知模式会被拒绝。

float、mode 和 CRC 均为小端。CRC 覆盖帧头之后、CRC 之前的全部载荷：旧格式 16 字节，新格式 20 字节，包括 mode。每四字节按小端组成 uint32_t 后依次计算，使用多项式 0x04C11DB7、初值 0xFFFFFFFF，不反射、不进行最终异或。载荷 00 01 … 0f 的参考 CRC 为 0x081B46CA。

## 验证

```sh
conda activate embedded
python -m unittest discover -s Tools/tests -v
```

测试覆盖 Ctrl+L 左右 Ctrl 键和重复过滤、目标角度按时间积分及松键保持、模式切换和停止时关闭角度环、CRC 和字节序、长按期间没有重复输入、组合键、只松开一个键、相反方向同时按住、停止后必须松键再按、焦点事件分包、失去焦点、发送超时后禁止恢复旧运动、普通终端兼容模式及串口回环。键盘事件测试使用与 evdev 相同的事件值和可读文件描述符；命令行交互使用伪终端。固件集成测试用本机 `cc` 编译实际的 remote.c、chassis.c 和 pid.c，并以 HAL/RTOS 替身验证 Python 控制帧解析、CRC 错帧重同步、模式选择、10 ms 间隔内的输出保持及 ±180° 边界。真实键盘与底盘的联动需要授权设备读取权限后手动测试。
