import io
import os
from pathlib import Path
import pty
import sys
import termios
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evdev
import serial
import chassis_keyboard as app
from rich.console import Console
from chassis_keyboard import Command, KeyboardController


@patch('chassis_keyboard.LINEAR_SPEED', 0.8)
class PhysicalKeyboardTests(unittest.TestCase):
    def test_hold_continues_across_initial_repeat_delay(self):
        controller = KeyboardController()
        controller.press('w', 1.0)
        for now in (1.26, 1.8, 3.0):
            self.assertEqual(controller.command(now), Command(vx=0.8))

    def test_second_key_repeat_does_not_expire_first_key(self):
        controller = KeyboardController()
        controller.press('w', 1.0)
        controller.press('a', 1.1)
        controller.press('a', 1.6)
        command = controller.command(1.7)
        self.assertAlmostEqual(command.vx, 0.565685424949238)
        self.assertAlmostEqual(command.vy, 0.565685424949238)

    def test_release_cancels_only_released_direction(self):
        controller = KeyboardController()
        for key in 'waq':
            controller.press(key, 1.0)
        controller.release('a')
        self.assertEqual(controller.command(5.0), Command(vx=0.8, d_yaw=0.5))
        controller.release('w')
        self.assertEqual(controller.command(5.1), Command(d_yaw=0.5))
        controller.release('q')
        self.assertEqual(controller.command(5.2), Command())

    def test_opposite_keys_cancel_and_release_restores_held_key(self):
        controller = KeyboardController()
        controller.press('w', 1.0)
        controller.press('s', 1.1)
        self.assertEqual(controller.command(1.2), Command())
        controller.release('s')
        self.assertEqual(controller.command(2.0), Command(vx=0.8))

    def test_stop_blocks_held_keys_until_release_and_new_press(self):
        controller = KeyboardController()
        controller.press('w', 1.0)
        controller.press(' ', 1.1)
        controller.press('w', 1.2)
        self.assertEqual(controller.command(1.3), Command())
        controller.release('w')
        controller.press('w', 1.4)
        self.assertEqual(controller.command(1.5), Command(vx=0.8))

    def test_physical_state_resynchronizes_release_and_blocks_stopped_keys(self):
        controller = KeyboardController()
        controller.set_pressed_keys({'w', 'a'})
        controller.press(' ', 1.0)
        controller.set_pressed_keys({'w', 'a'})
        self.assertEqual(controller.command(2.0), Command())
        controller.set_pressed_keys({'w'})
        controller.set_pressed_keys({'w', 'a'})
        self.assertEqual(controller.command(2.1), Command(vy=0.8))


class TestInputDevice:
    def __init__(self):
        self.fd, self.writer = os.pipe()
        self.path = '/dev/input/test'
        self.name = 'Test keyboard'
        self.pending = []
        self.keys = set()
        self.closed = False

    def emit(self, code, value):
        self.pending.append(evdev.InputEvent(0, 0, evdev.ecodes.EV_KEY, code, value))
        if value == 1:
            self.keys.add(code)
        elif value == 0:
            self.keys.discard(code)
        os.write(self.writer, b'x')

    def capabilities(self):
        e = evdev.ecodes
        return {e.EV_SYN: [e.SYN_REPORT], e.EV_KEY: [e.KEY_W, e.KEY_A, e.KEY_S, e.KEY_D, e.KEY_Q, e.KEY_E, e.KEY_L, e.KEY_LEFTCTRL, e.KEY_RIGHTCTRL, e.KEY_SPACE, e.KEY_ENTER, e.KEY_X, e.KEY_ESC]}

    def active_keys(self):
        return list(self.keys)

    def read(self):
        os.read(self.fd, 4096)
        events, self.pending = self.pending, []
        return iter(events)

    def close(self):
        if not self.closed:
            os.close(self.fd)
            os.close(self.writer)
            self.closed = True


class DeviceKeyboardTests(unittest.TestCase):
    def test_real_event_values_preserve_two_keys_and_ignore_repeat(self):
        device = TestInputDevice()
        controller = KeyboardController()
        e = evdev.ecodes
        try:
            with patch('evdev.InputDevice', return_value=device):
                with app.LinuxKeyboard('/dev/input/test') as keyboard:
                    for code, value in ((e.KEY_W, 1), (e.KEY_A, 1), (e.KEY_A, 2), (e.KEY_A, 0)):
                        device.emit(code, value)
                        for key, pressed in keyboard.read(0):
                            if pressed:
                                controller.press(key, 1.0)
                            else:
                                controller.release(key)
                        controller.set_pressed_keys(keyboard.pressed_keys)
                    self.assertEqual(controller.command(3.0), Command(vx=0.8))
                    device.emit(e.KEY_W, 0)
                    for key, pressed in keyboard.read(0):
                        if not pressed:
                            controller.release(key)
                    controller.set_pressed_keys(keyboard.pressed_keys)
                    self.assertEqual(controller.command(3.1), Command())
        finally:
            device.close()

    def test_focus_events_across_reads_stop_focus_and_restore_terminal(self):
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        try:
            with os.fdopen(os.dup(slave), 'r') as stream, patch('sys.stdout', new=io.StringIO()):
                with app.TerminalKeyboard(stream, focus_events=True) as keyboard:
                    os.write(master, b'\x1b[')
                    keyboard.read(0.1)
                    os.write(master, b'O')
                    keyboard.read(0.1)
                    self.assertFalse(keyboard.focused)
                    os.write(master, b'\x1b[I')
                    keyboard.read(0.1)
                    self.assertTrue(keyboard.focused)
            self.assertEqual(termios.tcgetattr(slave), original)
        finally:
            os.close(master)
            os.close(slave)

    def test_control_loop_uses_real_state_for_hold_release_focus_and_stop(self):
        device = TestInputDevice()
        clock = SimpleNamespace(now=0.0, step=0)
        terminal = SimpleNamespace(focused=True)
        e = evdev.ecodes
        steps = {1: [(e.KEY_W, 1)], 2: [(e.KEY_A, 1)], 3: [(e.KEY_A, 2)],
                 5: [(e.KEY_A, 0)], 8: [(e.KEY_W, 0)], 9: [(e.KEY_W, 1)],
                 10: [(e.KEY_SPACE, 1)], 11: [(e.KEY_W, 2)],
                 12: [(e.KEY_W, 0), (e.KEY_SPACE, 0)], 13: [(e.KEY_W, 1)],
                 14: [(e.KEY_ESC, 1)]}
        submitted = []

        def read(timeout):
            clock.step += 1
            clock.now += 0.1
            if clock.step == 6:
                terminal.focused = False
            elif clock.step == 7:
                terminal.focused = True
            for code, value in steps.get(clock.step, []):
                device.emit(code, value)
            if clock.step > 14:
                raise RuntimeError('Exit key was not handled')
            return ''

        class TerminalContext:
            def __enter__(self):
                return terminal

            def __exit__(self, *args):
                return False

        terminal.read = read
        args = SimpleNamespace(port='/dev/fake', baud=115200, yaw_rate=0.5, hold_time=0.25,
                               input_mode='evdev', keyboard='/dev/input/test')
        with serial.serial_for_url('loop://', timeout=0, write_timeout=0.1) as link:
            real_write = app.write_command

            def record(command_link, command):
                real_write(command_link, command)
                submitted.append((clock.step, command))

            try:
                with patch('evdev.InputDevice', return_value=device), \
                     patch('chassis_keyboard.TerminalKeyboard', return_value=TerminalContext()), \
                     patch('chassis_keyboard.time.monotonic', side_effect=lambda: clock.now), \
                     patch('chassis_keyboard.write_command', side_effect=record):
                    app.run_control(link, args, Console(file=io.StringIO(), width=120))
            finally:
                device.close()
        by_step = dict(submitted)
        self.assertEqual(by_step[1], Command(vx=0.8))
        self.assertAlmostEqual(by_step[4].vx, 0.565685424949238)
        self.assertAlmostEqual(by_step[4].vy, 0.565685424949238)
        self.assertEqual(by_step[5], Command(vx=0.8))
        for step in (6, 7, 10, 11, 12):
            self.assertEqual(by_step[step], Command())
        self.assertEqual(by_step[9], Command(vx=0.8))
        self.assertEqual(by_step[13], Command(vx=0.8))

    def test_moving_timeout_does_not_resume_held_keys_after_space(self):
        device = TestInputDevice()
        clock = SimpleNamespace(now=0.0, step=0)
        e = evdev.ecodes
        steps = {1: [(e.KEY_W, 1)], 2: [(e.KEY_A, 1)], 5: [(e.KEY_SPACE, 1)],
                 6: [(e.KEY_A, 0)], 7: [(e.KEY_A, 1)], 8: [(e.KEY_ESC, 1)]}
        submitted = []
        failed = False

        class TerminalContext:
            focused = True

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def read(self, timeout):
                clock.now += 0.1
                clock.step += 1
                for code, value in steps.get(clock.step, []):
                    device.emit(code, value)
                if clock.step > 8:
                    raise RuntimeError('Exit key was not handled')
                return ''

        args = SimpleNamespace(port='/dev/fake', baud=115200, yaw_rate=0.5, hold_time=0.25,
                               input_mode='evdev', keyboard='/dev/input/test')
        with serial.serial_for_url('loop://', timeout=0, write_timeout=0.1) as link:
            real_write = app.write_command

            def record(command_link, command):
                nonlocal failed
                submitted.append((clock.step, command))
                if command.vx > 0 and not failed:
                    failed = True
                    raise serial.SerialTimeoutException('Write timeout')
                real_write(command_link, command)

            try:
                with patch('evdev.InputDevice', return_value=device), \
                     patch('chassis_keyboard.TerminalKeyboard', return_value=TerminalContext()), \
                     patch('chassis_keyboard.time.monotonic', side_effect=lambda: clock.now), \
                     patch('chassis_keyboard.write_command', side_effect=record):
                    app.run_control(link, args, Console(file=io.StringIO(), width=120))
            finally:
                device.close()
        self.assertTrue(failed)
        self.assertTrue(all(cmd == Command() for step, cmd in submitted if 1 < step <= 6))
        self.assertTrue(any(step == 7 and cmd == Command(vy=0.8) for step, cmd in submitted))


if __name__ == '__main__':
    unittest.main()
