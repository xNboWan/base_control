#!/usr/bin/env python3
"""Xbox 360 底盘串口控制：Hat 步进巡航速度，Y/Ctrl+L 切换转向模式。"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import math
import os
import re
import select
import signal
import sys
import time

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
import pygame
import serial
from serial.tools import list_ports
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from chassis_keyboard import (
    Command, HEADLOCK, LINEAR_SPEED, RETRY_PERIOD, SEND_PERIOD,
    TerminalKeyboard, encode_frame, write_command,
)


@dataclass(frozen=True)
class GamepadState:
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    hat: tuple[int, int] = (0, 0)
    buttons: frozenset[int] = frozenset()


@dataclass(frozen=True)
class GamepadBatch:
    state: GamepadState
    events: tuple = ()


class GamepadController:
    """Hat 上升沿步进，摇杆覆盖巡航，目标 yaw 按实际时间积分。"""

    def __init__(self, max_speed=LINEAR_SPEED, speed_step=0.1, yaw_rate=0.5,
                 deadzone=0.08, button_mode=3, button_stop=1, button_exit=7):
        for name, value in (('速度上限', max_speed), ('速度步长', speed_step), ('角速度', yaw_rate)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name}必须是大于零的有限数值')
        if not math.isfinite(deadzone) or not 0 <= deadzone < 1:
            raise ValueError('摇杆死区必须满足 0 <= deadzone < 1')
        buttons = (button_mode, button_stop, button_exit)
        if any(type(value) is not int or value < 0 for value in buttons) or len(set(buttons)) != 3:
            raise ValueError('模式、停止和退出按钮必须是三个不同的非负整数编号')
        encode_frame(Command(vx=max_speed, d_yaw=yaw_rate))
        self.max_speed, self.speed_step, self.yaw_rate, self.deadzone = max_speed, speed_step, yaw_rate, deadzone
        self.button_mode, self.button_stop, self.button_exit = buttons
        self.mode = 'velocity'
        self.target_yaw = 0.0
        self.angle_active = False
        self.cruise_vx = self.cruise_vy = 0.0
        self.axes = [0.0] * 3
        self.blocked = [False] * 3
        self.hat = (0, 0)
        self.buttons = set()
        self.last_update = None

    def _axis(self, value):
        if not math.isfinite(value):
            raise ValueError('手柄轴值必须为有限数值')
        value = max(-1.0, min(1.0, value))
        return math.copysign(max(0.0, abs(value) - self.deadzone) / (1.0 - self.deadzone), value)

    def _effective_axes(self):
        return [0.0 if blocked else axis for axis, blocked in zip(self.axes, self.blocked)]

    def _advance(self, now):
        if self.last_update is not None and self.angle_active:
            rotation = -self._effective_axes()[2] * self.yaw_rate
            self.target_yaw = math.remainder(self.target_yaw + rotation * max(0, now - self.last_update), math.tau)
        self.last_update = now

    def stop(self, now):
        self._advance(now)
        self.cruise_vx = self.cruise_vy = 0.0
        self.blocked = [blocked or axis != 0 for axis, blocked in zip(self.axes, self.blocked)]
        self.angle_active = False

    def toggle_mode(self, now):
        self.stop(now)
        self.mode = 'angle' if self.mode == 'velocity' else 'velocity'
        self.angle_active = self.mode == 'angle'

    def seed(self, state, now):
        self._advance(now)
        self.axes = [self._axis(value) for value in (state.left_x, state.left_y, state.right_x)]
        self.hat, self.buttons = state.hat, set(state.buttons)
        self.stop(now)

    def _hat_event(self, hat, enabled):
        if len(hat) != 2 or any(value not in (-1, 0, 1) for value in hat):
            raise ValueError('Hat 值必须为两个 -1/0/1 分量')
        if enabled and self.button_stop not in self.buttons:
            if hat[1] and hat[1] != self.hat[1]:
                self.cruise_vx = max(-self.max_speed, min(self.max_speed, self.cruise_vx + hat[1] * self.speed_step))
                self.angle_active = self.mode == 'angle'
            if hat[0] and hat[0] != self.hat[0]:
                self.cruise_vy = max(-self.max_speed, min(self.max_speed, self.cruise_vy - hat[0] * self.speed_step))
                self.angle_active = self.mode == 'angle'
        self.hat = tuple(hat)

    def _button_event(self, button, down, now, enabled, actions):
        new_press = down and button not in self.buttons
        if down:
            self.buttons.add(button)
        else:
            self.buttons.discard(button)
        if not new_press:
            return
        if button == self.button_stop:
            self.stop(now)
            actions.append('stop')
        elif button == self.button_exit:
            actions.append('exit')
        elif button == self.button_mode and enabled and self.button_stop not in self.buttons:
            self.toggle_mode(now)
            actions.append('mode')

    def update(self, state, now, events=(), enabled=True):
        self._advance(now)
        self.axes = [self._axis(value) for value in (state.left_x, state.left_y, state.right_x)]
        self.blocked = [blocked and axis != 0 for axis, blocked in zip(self.axes, self.blocked)]
        if enabled and any(self._effective_axes()):
            self.angle_active = self.mode == 'angle'
        actions = []
        for kind, value in events:
            if kind == 'hat':
                self._hat_event(value, enabled)
            elif kind == 'button':
                self._button_event(value[0], value[1], now, enabled, actions)
        # 补齐当前状态，同时保留一次轮询间完整按下/松开的事件。
        for button in sorted(self.buttons - set(state.buttons)):
            self._button_event(button, False, now, enabled, actions)
        for button in sorted(set(state.buttons) - self.buttons):
            self._button_event(button, True, now, enabled, actions)
        self._hat_event(state.hat, enabled)
        if not enabled or self.button_stop in self.buttons:
            self.stop(now)
        return actions

    def command(self, now):
        self._advance(now)
        if self.button_stop in self.buttons:
            return Command()
        left_x, left_y, right_x = self._effective_axes()
        vx = -left_y * self.max_speed if left_y else self.cruise_vx
        vy = -left_x * self.max_speed if left_x else self.cruise_vy
        length = math.hypot(vx, vy)
        if length > self.max_speed:
            vx, vy = vx * self.max_speed / length, vy * self.max_speed / length
        if self.mode == 'angle' and self.angle_active:
            return Command(vx, vy, self.target_yaw, 0.0, mode=HEADLOCK)
        return Command(vx, vy, 0.0, -right_x * self.yaw_rate)


class GamepadDisconnected(RuntimeError):
    pass


@contextmanager
def pygame_input():
    # 终端界面无需图形窗口和音频；事件泵在主线程运行。
    os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
    os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')
    try:
        pygame.display.init()
        pygame.display.set_mode((1, 1), pygame.HIDDEN)
        pygame.joystick.init()
        yield
    finally:
        pygame.joystick.quit()
        pygame.display.quit()


class PygameGamepad:
    def __init__(self, index=0, axis_left_x=0, axis_left_y=1, axis_right_x=3,
                 hat_index=0, button_mode=3, button_stop=1, button_exit=7):
        self.index, self.axis_indices, self.hat_index = index, (axis_left_x, axis_left_y, axis_right_x), hat_index
        self.button_indices = (button_mode, button_stop, button_exit)
        self.device = None

    def __enter__(self):
        self.environment = pygame_input()
        self.environment.__enter__()
        try:
            if not 0 <= self.index < pygame.joystick.get_count():
                raise ValueError('未找到所选手柄；先连接 Xbox 360，再用 --list-gamepads 查看编号及读取情况')
            self.device = pygame.joystick.Joystick(self.index)
            if any(not 0 <= axis < self.device.get_numaxes() for axis in self.axis_indices):
                raise ValueError('手柄轴编号不匹配，请查看 --list-gamepads 并指定 --axis-left-x/--axis-left-y/--axis-right-x')
            if not 0 <= self.hat_index < self.device.get_numhats():
                raise ValueError('手柄没有所选 Hat；请查看 --list-gamepads 并检查 Xbox 360 模式及 --hat-index')
            if any(not 0 <= button < self.device.get_numbuttons() for button in self.button_indices):
                raise ValueError('手柄按钮编号不匹配，请检查 --button-mode/--button-stop/--button-exit')
            self.name = self.device.get_name()
            self.instance_id = self.device.get_instance_id()
            return self
        except BaseException:
            self.__exit__(*sys.exc_info())
            raise

    def poll(self):
        events = []
        for event in pygame.event.get():
            if getattr(event, 'instance_id', None) != self.instance_id:
                continue
            if event.type == pygame.JOYDEVICEREMOVED:
                raise GamepadDisconnected('所选手柄已断开，控制结束')
            if event.type == pygame.JOYHATMOTION and event.hat == self.hat_index:
                events.append(('hat', tuple(event.value)))
            elif event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                events.append(('button', (event.button, event.type == pygame.JOYBUTTONDOWN)))
        axes = [self.device.get_axis(axis) for axis in self.axis_indices]
        buttons = frozenset(button for button in range(self.device.get_numbuttons()) if self.device.get_button(button))
        return GamepadBatch(GamepadState(*axes, tuple(self.device.get_hat(self.hat_index)), buttons), tuple(events))

    def __exit__(self, *args):
        if self.device is not None:
            self.device.quit()
        self.environment.__exit__(*args)


class GamepadTerminal(TerminalKeyboard):
    """终端保留快捷键，同时读取焦点报告；支持分包的 CSI I/O。"""

    def __init__(self, stream=None):
        super().__init__(stream, focus_events=True)
        self.escape_since = 0.0

    def read(self, timeout):
        if not select.select([self.fd], [], [], timeout)[0]:
            if self.focus_buffer == '\x1b' and time.monotonic() - self.escape_since >= 0.05:
                self.focus_buffer = ''
                return '\x1b'
            return ''
        data = os.read(self.fd, 64)
        if not data:
            raise EOFError('终端输入已关闭')
        text = self.focus_buffer + data.decode('ascii', errors='ignore')
        for match in re.finditer('\x1b\\[([IO])', text):
            self.focused = match.group(1) == 'I'
        text = re.sub('\x1b\\[[IO]', '', text)
        self.focus_buffer = next((suffix for suffix in ('\x1b[', '\x1b') if text.endswith(suffix)), '')
        if self.focus_buffer:
            text = text[:-len(self.focus_buffer)]
            self.escape_since = time.monotonic()
        return re.sub('\x1b\\[[0-?]*[ -/]*[@-~]', '', text)


def render_dashboard(command, controller, port, baud, sent, gamepad_name, warning=''):
    angle = controller.mode == 'angle'
    mode = '角度模式' if angle else '角速度模式'
    values = Table.grid(padding=(0, 3))
    values.add_row('vx / vy', f'{command.vx:+.3f} / {command.vy:+.3f} m/s', '向前 / 向左为正')
    values.add_row('平移合速度', f'{math.hypot(command.vx, command.vy):.3f} m/s', f'上限 {controller.max_speed:g} m/s')
    values.add_row('Hat 巡航设置', f'{controller.cruise_vx:+.3f} / {controller.cruise_vy:+.3f} m/s', '松开后保持；摇杆回中恢复')
    values.add_row('d_yaw（发送值）', f'{command.d_yaw:+.3f} rad/s', '俯视逆时针为正')
    if angle:
        values.add_row('目标 yaw', f'{math.degrees(controller.target_yaw):+.1f}° / {controller.target_yaw:+.3f} rad', '右摇杆回中后保持')
    controls = Table.grid(padding=(0, 3))
    controls.add_row('左摇杆', '前后 / 左右平移', '回中恢复 Hat 巡航速度')
    controls.add_row('右摇杆左右', '转向 / 连续调整目标角度', f'上限 {controller.yaw_rate:g} rad/s')
    controls.add_row('Hat ↑ / ↓', 'vx 增加 / 减少', f'每次 {controller.speed_step:g} m/s')
    controls.add_row('Hat ← / →', 'vy 增加 / 减少', '每次按下步进，长按不重复')
    controls.add_row(f'Y [{controller.button_mode}] / Ctrl+L', '切换模式', '切换后摇杆先回中')
    controls.add_row(f'B [{controller.button_stop}] / 空格 / Enter', '停止并重试恢复控制', '清除巡航并暂停角度环')
    controls.add_row(f'Start [{controller.button_exit}] / Esc / X / Ctrl+C', '退出', '退出前尝试发送零速度')
    status = Text(f'手柄：{gamepad_name}\n串口：{port}  |  {baud} 波特率  |  正常发送 20 Hz  |  串口提交 {sent} 帧\n')
    if warning:
        status.append(warning + '\n', style='bold yellow')
    status.append('帧数表示串口写入完成；当前协议没有主控接收确认。\n', style='dim')
    if angle:
        status.append('目标角度初始 0°，使用 IMU 的 yaw 基准；进入模式后跟踪显示的目标。\n', style='dim')
    status.append('请保持控制终端焦点。停止后，已偏转的摇杆需回中再操作。', style='dim')
    return Group(Panel(f'[bold cyan]Xbox 360 底盘控制[/] · {mode}', border_style='cyan'),
                 Panel(values, title='目标控制指令'), Panel(controls, title='操作'),
                 Panel(status, border_style='red' if warning else 'green'))


def run_control(link, args, console, gamepad):
    controller = GamepadController(args.max_speed, args.speed_step, args.yaw_rate, args.deadzone,
                                   args.button_mode, args.button_stop, args.button_exit)
    initial = gamepad.poll()
    controller.seed(initial.state, time.monotonic())
    previous_state = initial.state
    sent = timeouts = 0
    paused = False
    warning = ''
    next_send = next_render = 0.0
    focused = True

    def transmit(command):
        nonlocal sent, timeouts, paused, warning
        try:
            write_command(link, command)
        except serial.SerialTimeoutException as error:
            controller.stop(time.monotonic())
            paused = True
            timeouts += 1
            warning = f'控制暂停（发送未完成 {timeouts} 次）：{error}。仅重试零速度；B/空格重试并恢复。'
            link.reset_output_buffer()
            return False
        sent += 1
        if paused:
            warning = '零速度帧已提交；按 B/空格恢复控制，将摇杆回中后再操作。'
        return True

    def dashboard():
        command = Command() if paused else controller.command(time.monotonic())
        message = warning or ('' if focused else '终端失去焦点，控制暂停；回到终端后摇杆先回中再操作。')
        return render_dashboard(command, controller, args.port, args.baud, sent, gamepad.name, message)

    transmit(Command())
    next_send = time.monotonic() + (RETRY_PERIOD if paused else SEND_PERIOD)
    with GamepadTerminal() as terminal, Live(dashboard(), console=console, screen=True,
                                            auto_refresh=False, transient=True) as live:
        while True:
            keys = terminal.read(0.01)
            batch = gamepad.poll()
            now = time.monotonic()
            events = batch.events
            if focused != terminal.focused:
                focused = terminal.focused
                controller.seed(batch.state, now)
                # 快照已接管输入基准，丢弃跨越焦点切换的旧事件。
                events = ()
                transmit(Command())
                next_send = time.monotonic() + (RETRY_PERIOD if paused else SEND_PERIOD)
                next_render = 0.0
            actions = controller.update(batch.state, now, events, enabled=focused and not paused)
            if batch.state != previous_state or batch.events:
                next_render = 0.0
            if events and focused and not paused:
                next_send = 0.0
            previous_state = batch.state
            for key in keys:
                if key.lower() == 'x' or key == '\x1b':
                    actions.append('exit')
                elif key in (' ', '\r', '\n'):
                    controller.stop(now)
                    actions.append('stop')
                elif key == '\x0c' and focused and not paused:
                    controller.toggle_mode(now)
                    actions.append('mode')
            if 'exit' in actions:
                return sent
            if 'stop' in actions:
                controller.stop(now)
                if transmit(Command()):
                    paused, warning = False, ''
                next_send = time.monotonic() + (RETRY_PERIOD if paused else SEND_PERIOD)
            elif 'mode' in actions:
                next_send = 0.0
            if actions:
                next_render = 0.0
            command = Command() if paused else controller.command(now)
            if now >= next_send:
                if not transmit(command):
                    next_render = 0.0
                next_send = time.monotonic() + (RETRY_PERIOD if paused else SEND_PERIOD)
            if args.port == 'loop://' and link.in_waiting:
                link.read(link.in_waiting)
            if now >= next_render:
                live.update(dashboard(), refresh=True)
                next_render = time.monotonic() + 0.1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='串口，例如 /dev/ttyACM0；loop:// 用于无底盘测试')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--gamepad', type=int, default=0, help='手柄编号，默认 0；--list-gamepads 查看')
    parser.add_argument('--list-gamepads', action='store_true')
    parser.add_argument('--list-ports', action='store_true')
    parser.add_argument('--max-speed', type=float, default=LINEAR_SPEED, help='平移合速度上限，默认沿用键盘脚本')
    parser.add_argument('--speed-step', type=float, default=0.1, help='Hat 每次按下增减速度，默认 0.1 m/s')
    parser.add_argument('--yaw-rate', type=float, default=0.5, help='右摇杆角速度 / 目标角度调整速率上限，rad/s')
    parser.add_argument('--deadzone', type=float, default=0.08, help='摇杆死区，默认 0.08')
    parser.add_argument('--axis-left-x', type=int, default=0)
    parser.add_argument('--axis-left-y', type=int, default=1)
    parser.add_argument('--axis-right-x', type=int, default=3)
    parser.add_argument('--hat-index', type=int, default=0)
    parser.add_argument('--button-mode', type=int, default=3, help='Y：切换模式')
    parser.add_argument('--button-stop', type=int, default=1, help='B：停止 / 恢复发送')
    parser.add_argument('--button-exit', type=int, default=7, help='Start：退出')
    parser.add_argument('--write-timeout', type=float, default=0.1)
    args = parser.parse_args(argv)
    console = Console()
    if args.list_ports:
        ports = list(list_ports.comports())
        for port in ports:
            console.print(Text(f'{port.device}  {port.description}'))
        if not ports:
            console.print('未检测到串口。')
        return 0
    if args.list_gamepads:
        try:
            with pygame_input():
                count = pygame.joystick.get_count()
                for index in range(count):
                    device = pygame.joystick.Joystick(index)
                    try:
                        console.print(Text(f'[{index}] {device.get_name()}  轴={device.get_numaxes()} 按钮={device.get_numbuttons()} Hat={device.get_numhats()}  GUID={device.get_guid()}'))
                    finally:
                        device.quit()
                if not count:
                    console.print('未检测到可读取的手柄；连接 Xbox 360 并检查输入设备读取权限后重试。')
            return 0
        except pygame.error as error:
            console.print(Text(f'手柄初始化失败：{error}', style='red'))
            return 1
    if not args.port:
        parser.error('请用 --port 指定串口')
    if args.baud <= 0 or not math.isfinite(args.write_timeout) or args.write_timeout <= 0:
        parser.error('波特率和串口写入超时必须大于零，超时必须为有限数值')
    try:
        GamepadController(args.max_speed, args.speed_step, args.yaw_rate, args.deadzone,
                          args.button_mode, args.button_stop, args.button_exit)
    except ValueError as error:
        parser.error(str(error))
    if not sys.stdin.isatty():
        console.print('请在交互式终端中运行，不能重定向标准输入。', style='red')
        return 2

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, interrupt)
    link = None
    failure = stop_failure = None
    try:
        with PygameGamepad(args.gamepad, args.axis_left_x, args.axis_left_y, args.axis_right_x,
                          args.hat_index, args.button_mode, args.button_stop, args.button_exit) as gamepad:
            link = serial.serial_for_url(args.port, baudrate=args.baud, bytesize=serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                         timeout=0, write_timeout=args.write_timeout,
                                         xonxoff=False, rtscts=False, dsrdtr=False)
            run_control(link, args, console, gamepad)
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError, EOFError, ValueError, GamepadDisconnected, pygame.error) as error:
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
        console.print(Text(f'控制已结束：{failure}', style='red'))
    if stop_failure:
        console.print(Text(f'停止帧发送失败：{stop_failure}', style='red'))
    elif link is not None:
        console.print('已退出，零速度帧已提交给串口；当前协议没有主控接收确认。', style='green')
    return 1 if failure or stop_failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
