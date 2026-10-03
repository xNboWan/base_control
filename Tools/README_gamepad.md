# Xbox 360 手柄控制底盘

`chassis_gamepad.py` 使用 pygame 读取 Xbox 360 手柄，在终端显示速度、巡航设置、当前转向模式和目标角度。参考实现为 [go1/deploy/joystick.py](/media/nbowan/rl_wp/go1/deploy/joystick.py)，沿用 Hat 步进巡航、左摇杆覆盖巡航、右摇杆转向的操作方式。

脚本复用同目录 `chassis_keyboard.py` 中的串口协议和 CRC。请保留两个脚本。角速度指令使用 23 字节帧；角度指令使用包含模式字段的 27 字节帧，需要前面接入 `HEADLOCK/HEADFREE` 的固件。

## 启动

pygame 已安装在本机 `embedded` 环境中。首次在另一台电脑部署时，安装 `Tools/requirements.txt` 中的依赖。

在工程根目录运行：

```sh
conda activate embedded
python Tools/chassis_gamepad.py --list-gamepads
python Tools/chassis_gamepad.py --port /dev/ttyACM0 --gamepad 0
```

`--list-gamepads` 会列出 pygame 识别的手柄编号、名称、轴数、按钮数、Hat 数和 GUID。`--gamepad` 使用这里的编号，不是 `/dev/input/jsX` 的编号。脚本运行时显示实际打开的手柄名称；多个手柄时请选择对应编号。

若没有识别到手柄，先确认 Xbox 360 已连接、接收器正常、Linux 能识别该设备，并检查当前用户对相应 `/dev/input` 设备的读取权限。脚本不需要读取物理键盘事件设备；Ctrl+L 等快捷键直接从控制终端读取。

串口仍为 UART8 接收控制指令、UART7 打印，两端默认 115200、8N1、无流控。使用 DAP CDC 的控制连接示例为 `/dev/ttyACM0`。

只测试手柄与终端界面时，用回环串口：

```sh
python Tools/chassis_gamepad.py --port loop:// --gamepad 0
```

程序需要交互式终端。pygame 使用隐藏的输入环境，终端是操作界面。启动前已经偏转的摇杆、已按住的 Hat 和按钮不会触发运动，需要先回中或松开，再操作。

## 操作

| 操作 | 效果 |
| --- | --- |
| Hat 上 / 下 | 巡航 vx 增加 / 减少 0.1 m/s，向前为正 |
| Hat 左 / 右 | 巡航 vy 增加 / 减少 0.1 m/s，向左为正 |
| 左摇杆前后 / 左右 | 临时覆盖对应轴的巡航速度；回中后恢复该轴巡航设置 |
| 右摇杆左右（角速度模式） | 连续控制 d_yaw，向左为正，默认上限 0.5 rad/s |
| 右摇杆左右（角度模式） | 连续增减目标 yaw，默认最大调整速率 0.5 rad/s；回中后保持目标角度 |
| Y / Ctrl+L | 切换角速度与角度模式，清空巡航设置；已经偏转的摇杆须回中后再操作 |
| B / 空格 / Enter | 停止、清空巡航设置并暂停角度环；通信暂停时也用它重试并恢复 |
| Start / Esc / X / Ctrl+C | 退出，退出前尝试提交零速度帧 |

Hat **每次新按下一个方向步进一次**，长按不会不断增加速度。方向松开后保持巡航，例如上、松开、再上，vx 依次为 0.1、0.1、0.2 m/s。上与左可组合，两个方向分别调整。停止后继续按住原方向不会恢复巡航，需松开再按。

左摇杆有死区，默认 0.08，死区之外线性缩放。推到最大时使用速度上限；松手回中时，若 Hat 巡航设置非零，仍会按巡航设置移动。清空巡航使用 B、空格或 Enter。默认平移合速度上限沿用键盘脚本，当前为 **0.8 m/s**；组合方向也受这个合速度上限约束。界面的 Hat 设置显示各轴的巡航设定，vx/vy 显示最终发送值。

程序默认角速度模式。角度模式的目标 yaw 初始为 **0°**，使用 IMU 的 yaw 基准；进入模式会跟踪界面显示的目标角度。目标在 ±180° 内环绕；主控 yaw PID 计算转向速度。角度目标在切换模式与停止后保留，停止时关闭角度环，重新操作方向后才恢复跟踪。界面显示的是目标，没有实际 yaw 回传。

保持控制终端的焦点。终端支持焦点报告时，切出窗口会清空巡航和停止转向，回来后摇杆先回中再操作；终端不支持焦点报告时，切换前用 B 或空格停止。

## 速度与映射配置

例如，降低速度上限并减小每次 Hat 的步长：

```sh
python Tools/chassis_gamepad.py --port /dev/ttyACM0 --gamepad 0 \
  --max-speed 0.3 --speed-step 0.05 --yaw-rate 0.3
```

默认沿用参考代码的 Xbox 360 映射：左摇杆 X/Y 为轴 0/1，右摇杆 X 为轴 3，Hat 为 0，Y/B/Start 为按钮 3/1/7。不同驱动或兼容手柄的编号可能不同，可调整：

```sh
python Tools/chassis_gamepad.py --port loop:// \
  --axis-left-x 0 --axis-left-y 1 --axis-right-x 3 \
  --hat-index 0 --button-mode 3 --button-stop 1 --button-exit 7
```

轴和 Hat 的读取方式参考 [pygame.joystick 官方文档](https://www.pygame.org/docs/ref/joystick.html)。脚本在主线程处理事件，用手柄 instance ID 筛选事件，避免其他手柄影响当前控制。

## 通信状态与退出

正常周期发送约 20 Hz，Hat、模式切换和停止按钮尽快提交新指令。一次写入超时或部分帧写入时，暂停控制、清空巡航并屏蔽当前偏转的摇杆，以约 4 Hz 重试零速度。继续操作摇杆不会提高自动重试频率。串口恢复后按 B、空格或 Enter，成功提交零速度后恢复；不会自动继续原有巡航。

所选手柄断开或发生读取错误时退出，并尝试发送零速度。最终停止帧发送失败时会明确打印原因。帧数只表示电脑向串口提交完成；协议没有主控接收确认，固件目前也没有命令失联归零逻辑，通信断开时停止不能由电脑保证。

## 验证

```sh
conda activate embedded
python -m unittest discover -s Tools/tests -v
```

手柄测试覆盖 Hat 按下、长按、松开、组合方向、一次轮询内的短按、巡航与摇杆覆盖、死区和合速度限幅、模式切换、目标角度积分与保持、停止、发送超时后的恢复、焦点报告、实例筛选和断开处理。完整命令行测试使用模拟手柄、真实伪终端和 pyserial，检查速度帧、角度帧、CRC 和退出停止帧。

当前环境尚未识别到 Xbox 360，真实手柄和底盘联动需要连接设备后验证。
