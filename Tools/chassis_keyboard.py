#!/usr/bin/env python3
"""用 WASD / QE 控制底盘，Ctrl+L 切换角速度与目标角度模式。"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import select
import signal
import struct
import sys
import termios
import time
import tty

import evdev
import serial
from serial.tools import list_ports
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

LINEAR_SPEED = 0.8
SEND_PERIOD = 0.05
RETRY_PERIOD = 0.25
HEADLOCK = 0
HEADFREE = 1


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    d_yaw: float = 0.0
    mode: int | None = None  # None 使用原来的 23 字节角速度帧。


def crc32_stm32(payload: bytes) -> int:
    """匹配 remote.c：4/5 个小端 uint32_t，F427 CRC 初值和多项式。"""
    if len(payload) not in (16, 20):
        raise ValueError("CRC 载荷必须为 16 或 20 字节")
    crc = 0xFFFFFFFF
    for (word,) in struct.iter_unpack("<I", payload):
        crc ^= word
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
    return crc


def encode_frame(command: Command) -> bytes:
    values = (command.vx, command.vy, command.yaw, command.d_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("控制指令必须为有限数值")
    try:
        payload = struct.pack("<4f", *values)
    except (OverflowError, struct.error) as error:
        raise ValueError("指令超出了 float32 的范围") from error
    header = b're'
    if command.mode is not None:
        if type(command.mode) is not int or command.mode not in (HEADLOCK, HEADFREE):
            raise ValueError('控制模式必须为 HEADLOCK(0) 或 HEADFREE(1)')
        payload += struct.pack('<I', command.mode)
        header = b'rm'
    return header + payload + struct.pack("<I", crc32_stm32(payload)) + b"\x64"


def write_command(link: serial.SerialBase, command: Command) -> None:
    frame = encode_frame(command)
    written = link.write(frame)
    if written != len(frame):
        raise serial.SerialTimeoutException(f"串口只提交了 {written}/{len(frame)} 字节")


class KeyboardController:
    """维护实际按住的键；普通终端输入可显式使用超时模式。"""

    def __init__(self, yaw_rate: float = 0.5, hold_time: float = 0.25,
                 key_release_events: bool = True):
        if not math.isfinite(yaw_rate) or yaw_rate <= 0:
            raise ValueError("角速度必须是大于零的有限数值")
        if not math.isfinite(hold_time) or hold_time <= 0:
            raise ValueError("按键维持时间必须是大于零的有限数值")
        encode_frame(Command(d_yaw=yaw_rate))
        self.yaw_rate = yaw_rate
        self.hold_time = hold_time
        self.key_release_events = key_release_events
        self.held = set()
        self.blocked = set()
        self.signs = [0, 0, 0]
        self.deadlines = [0.0, 0.0, 0.0]
        self.mode = 'velocity'
        self.target_yaw = 0.0
        self.angle_active = False
        self.last_update = None

    def _advance(self, now: float) -> None:
        """按实际经过时间积分；界面刷新和发送共用同一个时间基准。"""
        if self.last_update is not None and self.mode == 'angle' and self.angle_active:
            if self.key_release_events:
                rotation = int('q' in self.held) - int('e' in self.held)
                elapsed = max(0.0, now - self.last_update)
            else:
                rotation = self.signs[2]
                elapsed = max(0.0, min(now, self.deadlines[2]) - self.last_update)
            self.target_yaw = math.remainder(self.target_yaw + rotation * self.yaw_rate * elapsed, math.tau)
        self.last_update = now

    def _clear_motion(self) -> None:
        self.blocked.update(self.held)
        self.held.clear()
        self.signs[:] = [0, 0, 0]
        self.deadlines[:] = [0.0, 0.0, 0.0]
        self.angle_active = False

    def press(self, key: str, now: float) -> bool:
        self._advance(now)
        if key == '\x0c':
            self._clear_motion()
            self.mode = 'angle' if self.mode == 'velocity' else 'velocity'
            self.angle_active = self.mode == 'angle'
            return True
        if key in (" ", "\r", "\n"):
            self._clear_motion()
            return True
        movement = {"w": (0, 1), "s": (0, -1), "a": (1, 1),
                    "d": (1, -1), "q": (2, 1), "e": (2, -1)}.get(key.lower())
        if movement is None:
            return False
        if self.key_release_events:
            key = key.lower()
            if key in self.blocked:
                return False
            self.held.add(key)
            self.angle_active = self.mode == 'angle'
            return True
        axis, sign = movement
        self.signs[axis] = sign
        self.deadlines[axis] = now + self.hold_time
        self.angle_active = self.mode == 'angle'
        return True

    def release(self, key: str, now: float | None = None) -> None:
        if now is not None:
            self._advance(now)
        self.held.discard(key.lower())
        self.blocked.discard(key.lower())

    def set_pressed_keys(self, keys, now: float | None = None) -> None:
        if now is not None:
            self._advance(now)
        pressed = {key.lower() for key in keys} & set("wasdqe")
        self.blocked.intersection_update(pressed)
        if pressed - self.blocked - self.held:
            self.angle_active = self.mode == 'angle'
        self.held = pressed - self.blocked

    def command(self, now: float) -> Command:
        self._advance(now)
        if self.key_release_events:
            x = int("w" in self.held) - int("s" in self.held)
            y = int("a" in self.held) - int("d" in self.held)
            rotation = int("q" in self.held) - int("e" in self.held)
        else:
            x, y, rotation = (sign if now < deadline else 0
                              for sign, deadline in zip(self.signs, self.deadlines))
        length = math.hypot(x, y)
        scale = LINEAR_SPEED / length if length else 0.0
        if self.mode == 'angle' and self.angle_active:
            return Command(x * scale, y * scale, self.target_yaw, 0.0, mode=HEADLOCK)
        return Command(x * scale, y * scale, 0.0, rotation * self.yaw_rate)


def keyboard_devices():
    """从 sysfs 列出键盘，即使当前用户尚未获准读取事件设备。"""
    result = []
    aliases = {p.resolve(): p for p in sorted(Path('/dev/input/by-id').glob('*-event-kbd'))}
    required = (evdev.ecodes.KEY_W, evdev.ecodes.KEY_A, evdev.ecodes.KEY_S,
                evdev.ecodes.KEY_D, evdev.ecodes.KEY_Q, evdev.ecodes.KEY_E, evdev.ecodes.KEY_L)
    for node in sorted(Path('/sys/class/input').glob('event*')):
        try:
            words = (node / 'device/capabilities/key').read_text().split()
            bits = sum(int(word, 16) << (i * struct.calcsize('L') * 8)
                       for i, word in enumerate(reversed(words)))
            if not all(bits & (1 << code) for code in required):
                continue
            if not any(bits & (1 << code) for code in (evdev.ecodes.KEY_LEFTCTRL, evdev.ecodes.KEY_RIGHTCTRL)):
                continue
            path = Path('/dev/input') / node.name
            path = aliases.get(path, path)
            result.append((str(path), (node / 'device/name').read_text().strip(), os.access(path, os.R_OK)))
        except OSError:
            continue
    return result


class LinuxKeyboard:
    """读取所选键盘的按下/松开事件和当前键位，不独占键盘。"""

    def __init__(self, path=None):
        self.path = path
        self.device = None
        self.pressed_keys = set()
        self.ctrl_keys = set()
        self.l_held = False
        e = evdev.ecodes
        self.mapping = {e.KEY_W: 'w', e.KEY_A: 'a', e.KEY_S: 's', e.KEY_D: 'd',
                        e.KEY_Q: 'q', e.KEY_E: 'e', e.KEY_SPACE: ' ',
                        e.KEY_ENTER: '\n', e.KEY_KPENTER: '\n', e.KEY_X: 'x', e.KEY_ESC: '\x1b'}

    def __enter__(self):
        if not self.path:
            available = [path for path, name, readable in keyboard_devices() if readable]
            if len(available) != 1:
                raise ValueError('请用 --list-keyboards 查看设备，再用 --keyboard 指定键盘；当前需要一个有读取权限的键盘设备。')
            self.path = available[0]
        try:
            self.device = evdev.InputDevice(self.path, readonly=True)
        except PermissionError as error:
            raise ValueError(f'没有读取键盘 {self.path} 的权限。请按 Tools/README.md 的 setfacl 说明授权后重试。') from error
        try:
            supported = set(self.device.capabilities().get(evdev.ecodes.EV_KEY, []))
            required = {code for code, key in self.mapping.items() if key in 'wasdqe'} | {evdev.ecodes.KEY_L}
            ctrl_codes = {evdev.ecodes.KEY_LEFTCTRL, evdev.ecodes.KEY_RIGHTCTRL}
            if not required <= supported or not ctrl_codes & supported:
                raise ValueError(f'{self.path} 不支持全部 WASD/QE/Ctrl+L 键，请选择实际使用的键盘。')
            self.name = self.device.name
            self.read(0)  # 丢弃启动前排队的事件，由调用方屏蔽已按住的键。
            active = set(self.device.active_keys())
            self.ctrl_keys = active & {evdev.ecodes.KEY_LEFTCTRL, evdev.ecodes.KEY_RIGHTCTRL}
            self.l_held = evdev.ecodes.KEY_L in active
            return self
        except BaseException:
            self.device.close()
            raise

    def read(self, timeout):
        events = []
        if select.select([self.device.fd], [], [], timeout)[0]:
            try:
                for event in self.device.read():
                    if event.type != evdev.ecodes.EV_KEY or event.value not in (0, 1):
                        continue
                    if event.code in (evdev.ecodes.KEY_LEFTCTRL, evdev.ecodes.KEY_RIGHTCTRL):
                        if event.value:
                            self.ctrl_keys.add(event.code)
                        else:
                            self.ctrl_keys.discard(event.code)
                    elif event.code == evdev.ecodes.KEY_L:
                        if event.value and not self.l_held and self.ctrl_keys:
                            events.append(('\x0c', True))
                        self.l_held = bool(event.value)
                    elif event.code in self.mapping:
                        events.append((self.mapping[event.code], event.value == 1))
            except BlockingIOError:
                pass
        active = set(self.device.active_keys())
        self.ctrl_keys = active & {evdev.ecodes.KEY_LEFTCTRL, evdev.ecodes.KEY_RIGHTCTRL}
        self.l_held = evdev.ecodes.KEY_L in active
        self.pressed_keys = {self.mapping[code] for code in active
                             if code in self.mapping and self.mapping[code] in 'wasdqe'}
        return events

    def __exit__(self, *args):
        self.device.close()


class TerminalKeyboard:
    def __init__(self, stream=None, focus_events=False):
        self.stream = stream if stream is not None else sys.stdin
        self.fd = self.stream.fileno()
        self.original = None
        self.focus_events = focus_events
        self.focused = True
        self.focus_buffer = ''

    def __enter__(self):
        if not self.stream.isatty():
            raise ValueError("请在交互式终端中运行此脚本")
        self.original = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        if self.focus_events:
            sys.stdout.write('\x1b[?1004h')
            sys.stdout.flush()
        return self

    def read(self, timeout: float) -> str:
        if not select.select([self.fd], [], [], timeout)[0]:
            return ""
        data = os.read(self.fd, 64)
        if not data:
            raise EOFError("终端输入已关闭")
        text = data.decode("ascii", errors="ignore")
        if self.focus_events:
            text = self.focus_buffer + text
            for match in re.finditer('\x1b\\[([IO])', text):
                self.focused = match.group(1) == 'I'
            self.focus_buffer = next((suffix for suffix in ('\x1b[', '\x1b') if text.endswith(suffix)), '')
            return ''  # 真实按键来自 evdev，终端只用于焦点及 Ctrl+C。
        return text

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)
        if self.focus_events:
            sys.stdout.write('\x1b[?1004l')
            sys.stdout.flush()


def render_dashboard(command: Command, port: str, sent: int, last_key: str,
                     hold_time: float, yaw_rate: float, baud: int = 115200,
                     warning: str = "", input_mode: str = 'terminal', keyboard_name: str = '',
                     control_mode: str = 'velocity', target_yaw: float = 0.0):
    angle_mode = control_mode == 'angle'
    mode_name = '角度模式' if angle_mode else '角速度模式'
    speeds = Table.grid(padding=(0, 3))
    speeds.add_column(style="bold")
    speeds.add_column(justify="right", style="bold cyan")
    speeds.add_column()
    speeds.add_column(style="dim")
    speeds.add_row("vx", f"{command.vx:+.3f}", "m/s", "向前为正")
    speeds.add_row("vy", f"{command.vy:+.3f}", "m/s", "向左为正")
    speeds.add_row("d_yaw", f"{command.d_yaw:+.3f}", "rad/s", "俯视逆时针为正")
    if angle_mode:
        speeds.add_row('目标 yaw', f'{math.degrees(target_yaw):+.1f}', '°', f'{target_yaw:+.3f} rad；IMU 的 yaw 基准')
    speeds.add_row("平移合速度", f"{math.hypot(command.vx, command.vy):.3f}", "m/s", f"固定 {LINEAR_SPEED:g} m/s")

    keys = Table.grid(padding=(0, 4))
    keys.add_row("[bold]W[/] 前进", "[bold]S[/] 后退", f"{LINEAR_SPEED:g} m/s")
    keys.add_row("[bold]A[/] 左移", "[bold]D[/] 右移", f"{LINEAR_SPEED:g} m/s")
    keys.add_row("[bold]Q[/] 左转", "[bold]E[/] 右转",
                 f'目标角度增减 {math.degrees(yaw_rate):.1f} °/s；松开保持' if angle_mode else f'{yaw_rate:g} rad/s')
    keys.add_row('[bold magenta]Ctrl+L[/] 切换模式', mode_name, '切换时清除已按住的方向')
    keys.add_row("[bold yellow]空格 / Enter[/] 停止", "[bold]Esc / X[/] 退出", "Ctrl+C 也可退出")

    moving = any((command.vx, command.vy, command.d_yaw))
    holding = command.mode == HEADLOCK
    status = Text()
    if warning:
        status.append("控制暂停", style="bold red")
    else:
        status.append('目标角度保持' if holding else "运动指令有效" if moving else "零速度指令",
                      style="bold green" if moving or holding else "bold yellow")
    status.append(f"  |  最近按键：{last_key}  |  串口提交：{sent} 帧\n")
    status.append(f"串口：{port}  |  {baud} 波特率  |  正常发送 20 Hz\n", style="dim")
    if warning:
        status.append(warning + "\n", style="bold yellow")
    status.append("帧数表示串口写入完成；当前协议没有主控接收确认。\n", style="dim")
    if angle_mode:
        status.append('目标 yaw 从 0° 开始；进入角度模式会追踪显示的目标。空格暂停角度环。\n', style='dim')
    if input_mode == 'evdev':
        status.append(f"键盘：{keyboard_name}\n按住持续运动，松开取消对应方向；支持组合键。请保持本终端焦点。", style="dim")
    else:
        status.append(f"普通终端模式：按键续期 {hold_time:g} s；无重复输入自动归零。", style="dim")
    return Group(
        Panel(f"[bold cyan]底盘键盘控制[/]  ·  {mode_name}  ·  WASD 平移 / QE 转向", border_style="cyan"),
        Panel(speeds, title="目标速度指令", border_style="cyan"),
        Panel(keys, title="按键操作", border_style="blue"),
        Panel(status, border_style="red" if warning else "green" if moving or holding else "yellow"),
    )


def run_control(link: serial.SerialBase, args, console: Console) -> int:
    input_mode = getattr(args, 'input_mode', 'terminal')
    physical = input_mode == 'evdev'
    controller = KeyboardController(args.yaw_rate, args.hold_time, key_release_events=physical)
    sent = 0
    timeouts = 0
    paused = False
    warning = ""
    last_key = "—"
    next_send = next_render = 0.0
    keyboard_name = ''
    focused = True

    def transmit(command: Command) -> bool:
        nonlocal sent, timeouts, paused, warning
        try:
            write_command(link, command)
        except serial.SerialTimeoutException as error:
            controller.press(" ", time.monotonic())
            paused = True
            timeouts += 1
            warning = f"发送未完成（累计 {timeouts} 次）：{error}。仅重试零速度；空格重试并恢复控制。"
            # 超时可能已经写入一部分数据，先丢弃系统中尚未发送的旧指令。
            # USB 设备内部的缓冲区未必能被清除，仍需主控自己的失联保护。
            link.reset_output_buffer()
            return False
        sent += 1
        if paused:
            warning = f"零速度帧已提交。按空格恢复控制，再输入方向键；累计发送未完成 {timeouts} 次。"
        return True

    def dashboard():
        command = Command() if paused else controller.command(time.monotonic())
        return render_dashboard(command, args.port, sent, last_key,
                                args.hold_time, args.yaw_rate, args.baud, warning,
                                input_mode, keyboard_name, controller.mode, controller.target_yaw)

    transmit(Command())
    next_send = time.monotonic() + (RETRY_PERIOD if paused else SEND_PERIOD)
    with ExitStack() as stack:
        terminal = stack.enter_context(TerminalKeyboard(focus_events=physical))
        keyboard = stack.enter_context(LinuxKeyboard(getattr(args, 'keyboard', None))) if physical else terminal
        if physical:
            keyboard_name = f'{keyboard.name} ({keyboard.path})'
            controller.set_pressed_keys(keyboard.pressed_keys, time.monotonic())
            controller.press(' ', time.monotonic())
        with Live(dashboard(),
                  console=console, screen=True, auto_refresh=False, transient=True) as live:
            while True:
                text = terminal.read(0.01)
                events = keyboard.read(0) if physical else [(key, True) for key in text]
                now = time.monotonic()
                if physical and focused != terminal.focused:
                    focused = terminal.focused
                    controller.set_pressed_keys(keyboard.pressed_keys, now)
                    controller.press(' ', now)
                    transmit(Command())
                    next_render = next_send = 0.0
                for key, pressed in events:
                    if not pressed:
                        controller.release(key, now)
                        next_send = next_render = 0.0
                        continue
                    if physical and not focused:
                        continue
                    if key.lower() == "x" or key == "\x1b":
                        return sent
                    stop_key = key in (" ", "\r", "\n")
                    if (stop_key or not paused) and controller.press(key, now):
                        last_key = 'Ctrl+L' if key == '\x0c' else "空格" if key == " " else "Enter" if key in ("\r", "\n") else key.upper()
                        next_render = 0.0
                        if physical or key == '\x0c':
                            next_send = 0.0
                        if stop_key:
                            if transmit(Command()):
                                paused = False
                                warning = ""
                            next_send = time.monotonic() + (RETRY_PERIOD if paused else SEND_PERIOD)

                if physical:
                    controller.set_pressed_keys(keyboard.pressed_keys, now)
                    if paused or not focused:
                        controller.press(' ', now)
                command = Command() if paused else controller.command(now)
                if now >= next_send:
                    if not transmit(command):
                        next_render = 0.0
                    next_send = time.monotonic() + (RETRY_PERIOD if paused else SEND_PERIOD)
                if args.port == "loop://" and link.in_waiting:
                    link.read(link.in_waiting)
                if now >= next_render:
                    live.update(dashboard(), refresh=True)
                    next_render = time.monotonic() + 0.1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="串口，例如 /dev/ttyUSB0；loop:// 用于无硬件演示")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--yaw-rate", type=float, default=0.5, help="Q/E 角速度或目标角度调整速率，默认 0.5 rad/s")
    parser.add_argument("--hold-time", type=float, default=0.25, help="仅 terminal 模式：按键维持秒数，默认 0.25")
    parser.add_argument('--input', dest='input_mode', choices=('evdev', 'terminal'), default='evdev',
                        help='输入方式：默认 evdev 支持真实按住/松开；terminal 为旧超时模式')
    parser.add_argument('--keyboard', help='键盘事件设备，例如 /dev/input/by-id/...-event-kbd')
    parser.add_argument('--list-keyboards', action='store_true', help='列出支持 WASD/QE/Ctrl+L 的键盘及读取权限')
    parser.add_argument("--write-timeout", type=float, default=0.1, help="串口单次写入超时秒数，默认 0.1；可用 1 做延迟对照")
    parser.add_argument("--list-ports", action="store_true", help="列出可用串口后退出")
    args = parser.parse_args(argv)
    console = Console()
    if args.list_keyboards:
        devices = keyboard_devices()
        for path, name, readable in devices:
            console.print(Text(f'{path}  {name}  {"可读取" if readable else "需要读取权限"}'))
        if not devices:
            console.print('未找到支持 WASD/QE 的键盘设备。')
        return 0
    if args.list_ports:
        ports = list(list_ports.comports())
        for port in ports:
            console.print(Text(f"{port.device}  {port.description}"))
        if not ports:
            console.print("未检测到串口。")
        return 0
    if not args.port:
        parser.error("请用 --port 指定串口；--list-ports 可查看可用端口")
    if args.baud <= 0:
        parser.error("波特率必须大于零")
    if not math.isfinite(args.write_timeout) or args.write_timeout <= 0:
        parser.error("串口写入超时必须是大于零的有限数值")
    try:
        KeyboardController(args.yaw_rate, args.hold_time)
    except ValueError as error:
        parser.error(str(error))
    if not sys.stdin.isatty():
        console.print("请在交互式终端中运行，不能重定向键盘输入。", style="red")
        return 2

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, interrupt)
    link = None
    failure = stop_failure = None
    try:
        link = serial.serial_for_url(args.port, baudrate=args.baud, bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                     timeout=0, write_timeout=args.write_timeout,
                                     xonxoff=False, rtscts=False, dsrdtr=False)
        run_control(link, args, console)
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError, EOFError, ValueError) as error:
        failure = str(error)
    finally:
        if link is not None:
            try:
                write_command(link, Command())
            except (serial.SerialException, OSError) as error:
                stop_failure = str(error)
            finally:
                link.close()
        signal.signal(signal.SIGTERM, previous_sigterm)

    if failure:
        console.print(Text(f"控制已结束：{failure}", style="red"))
    if stop_failure:
        console.print(Text(f"停止帧发送失败：{stop_failure}", style="red"))
    elif link is not None:
        console.print("已退出，零速度帧已提交给串口；当前协议没有主控接收确认。", style="green")
    return 1 if failure or stop_failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
