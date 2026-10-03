import io
import math
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import termios
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pygame
from rich.console import Console
import serial
import chassis_gamepad as app
from chassis_keyboard import Command, crc32_stm32


class GamepadControllerTests(unittest.TestCase):
    def test_hat_steps_on_new_direction_and_keeps_cruise_after_release(self):
        controller = app.GamepadController(max_speed=0.8)
        controller.update(app.GamepadState(hat=(0, 1)), 1.0)
        for now in (1.3, 2.0, 10.0):
            controller.update(app.GamepadState(hat=(0, 1)), now)
            self.assertAlmostEqual(controller.command(now).vx, 0.1)
        controller.update(app.GamepadState(), 10.1)
        self.assertAlmostEqual(controller.command(20.0).vx, 0.1)
        controller.update(app.GamepadState(hat=(0, 1)), 20.0)
        self.assertAlmostEqual(controller.command(20.0).vx, 0.2)
        controller.update(app.GamepadState(hat=(1, -1)), 20.1)
        self.assertAlmostEqual(controller.command(20.1).vx, 0.1)
        self.assertAlmostEqual(controller.command(20.1).vy, -0.1)

    def test_hat_diagonal_changes_each_axis_only_on_its_own_edge(self):
        controller = app.GamepadController()
        controller.update(app.GamepadState(hat=(0, 1)), 1.0)
        controller.update(app.GamepadState(hat=(-1, 1)), 1.1)
        self.assertAlmostEqual(controller.command(1.1).vx, 0.1)
        self.assertAlmostEqual(controller.command(1.1).vy, 0.1)

    def test_axis_overrides_cruise_and_center_restores_cruise(self):
        controller = app.GamepadController(max_speed=0.8)
        controller.update(app.GamepadState(hat=(0, 1)), 1.0)
        controller.update(app.GamepadState(left_x=-0.5, left_y=-1), 1.1)
        command = controller.command(1.1)
        self.assertGreater(command.vy, 0)
        self.assertAlmostEqual(math.hypot(command.vx, command.vy), 0.8)
        controller.update(app.GamepadState(), 1.2)
        self.assertAlmostEqual(controller.command(1.2).vx, 0.1)
        self.assertEqual(controller.command(1.2).vy, 0)

    def test_deadzone_and_right_stick_rotation_direction(self):
        controller = app.GamepadController(deadzone=0.08)
        controller.update(app.GamepadState(left_x=0.02, left_y=-0.03, right_x=0.05), 1.0)
        self.assertEqual(controller.command(1.0), Command())
        controller.update(app.GamepadState(right_x=-1), 1.1)
        self.assertEqual(controller.command(2.0), Command(d_yaw=0.5))

    def test_hat_and_analog_translation_stay_within_total_speed_limit(self):
        controller = app.GamepadController(max_speed=0.3, speed_step=0.1)
        for i in range(10):
            controller.update(app.GamepadState(hat=(-1, 1)), 1.0 + i)
            controller.update(app.GamepadState(), 1.1 + i)
        self.assertAlmostEqual(controller.cruise_vx, 0.3)
        self.assertAlmostEqual(controller.cruise_vy, 0.3)
        self.assertAlmostEqual(math.hypot(controller.command(12).vx, controller.command(12).vy), 0.3)

    def test_mode_button_is_edge_triggered_and_angle_release_keeps_target(self):
        controller = app.GamepadController()
        controller.update(app.GamepadState(buttons=frozenset({3})), 1.0)
        self.assertEqual(controller.mode, 'angle')
        controller.update(app.GamepadState(right_x=-1, buttons=frozenset({3})), 1.1)
        self.assertEqual(controller.mode, 'angle')
        self.assertAlmostEqual(controller.command(2.1).yaw, 0.5)
        controller.update(app.GamepadState(), 2.1)
        command = controller.command(10)
        self.assertEqual(command.mode, 0)
        self.assertEqual(command.d_yaw, 0)
        self.assertAlmostEqual(command.yaw, 0.5)

    def test_switch_blocks_sticks_held_until_center_and_fresh_motion(self):
        controller = app.GamepadController()
        controller.update(app.GamepadState(left_y=-1, right_x=-1), 1.0)
        controller.update(app.GamepadState(left_y=-1, right_x=-1, buttons=frozenset({3})), 1.1)
        self.assertEqual(controller.command(3.0), Command(mode=0))
        controller.update(app.GamepadState(), 3.0)
        controller.update(app.GamepadState(right_x=-1), 3.1)
        self.assertAlmostEqual(controller.command(4.1).yaw, 0.5)

    def test_stop_disables_angle_loop_and_clears_cruise_until_fresh_input(self):
        controller = app.GamepadController()
        controller.update(app.GamepadState(hat=(0, 1), buttons=frozenset({3})), 1.0)
        controller.update(app.GamepadState(right_x=-1, hat=(0, 1)), 1.1)
        controller.update(app.GamepadState(right_x=-1, hat=(0, 1), buttons=frozenset({1})), 2.1)
        self.assertEqual(controller.command(2.1), Command())
        controller.update(app.GamepadState(right_x=-1, hat=(0, 1)), 3.1)
        self.assertEqual(controller.command(3.1), Command())
        self.assertEqual(controller.cruise_vx, 0)
        controller.update(app.GamepadState(), 3.2)
        controller.update(app.GamepadState(hat=(0, 1)), 3.3)
        self.assertAlmostEqual(controller.command(3.3).vx, 0.1)
        self.assertEqual(controller.command(3.3).mode, 0)

    def test_hat_press_and_release_between_polls_are_not_lost(self):
        controller = app.GamepadController()
        events = (('hat', (0, 1)), ('hat', (0, 0)))
        controller.update(app.GamepadState(), 1.0, events=events)
        self.assertAlmostEqual(controller.command(1.0).vx, 0.1)

    def test_seed_state_does_not_use_sticks_hat_or_buttons_already_held(self):
        controller = app.GamepadController()
        state = app.GamepadState(left_y=-1, right_x=-1, hat=(0, 1), buttons=frozenset({3}))
        controller.seed(state, 1.0)
        controller.update(state, 2.0)
        self.assertEqual(controller.mode, 'velocity')
        self.assertEqual(controller.command(2.0), Command())
        controller.update(app.GamepadState(), 2.1)
        controller.update(app.GamepadState(hat=(0, 1)), 2.2)
        self.assertAlmostEqual(controller.command(2.2).vx, 0.1)

    def test_angle_wrap_and_disabled_input_do_not_restore_old_motion(self):
        controller = app.GamepadController()
        controller.toggle_mode(1.0)
        controller.update(app.GamepadState(right_x=-1), 1.0)
        self.assertAlmostEqual(controller.command(9).yaw, 4 - 2 * math.pi)
        controller.update(app.GamepadState(right_x=-1, hat=(0, 1)), 9.1, enabled=False)
        self.assertEqual(controller.command(11), Command())

    def test_invalid_configuration_and_nonfinite_axis_are_rejected(self):
        for option in ('max_speed', 'speed_step', 'yaw_rate'):
            for value in (0, -1, math.nan, math.inf):
                with self.subTest(option=option, value=value), self.assertRaises(ValueError):
                    app.GamepadController(**{option: value})
        for value in (-1, 1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                app.GamepadController(deadzone=value)
        with self.assertRaises(ValueError):
            app.GamepadController().update(app.GamepadState(left_x=math.nan), 1.0)


class FakeJoystick:
    def __init__(self):
        self.axes = [0.0] * 6
        self.buttons = [False] * 11
        self.hat = (0, 0)
        self.closed = False

    def get_name(self): return 'Xbox 360 test controller'
    def get_guid(self): return 'test-guid'
    def get_instance_id(self): return 42
    def get_numaxes(self): return len(self.axes)
    def get_numbuttons(self): return len(self.buttons)
    def get_numhats(self): return 1
    def get_axis(self, i): return self.axes[i]
    def get_button(self, i): return self.buttons[i]
    def get_hat(self, i): return self.hat
    def quit(self): self.closed = True


class GamepadBackendTests(unittest.TestCase):
    def test_reads_standard_xbox_axes_and_filters_events_by_instance_id(self):
        joystick = FakeJoystick()
        joystick.axes = [-0.2, -0.4, 1.0, -0.6, 0.0, 1.0]
        with patch('pygame.joystick.get_count', return_value=1), \
             patch('pygame.joystick.Joystick', return_value=joystick):
            with app.PygameGamepad() as gamepad:
                events = [pygame.event.Event(pygame.JOYHATMOTION, instance_id=99, hat=0, value=(1, 0)),
                          pygame.event.Event(pygame.JOYHATMOTION, instance_id=42, hat=0, value=(0, 1)),
                          pygame.event.Event(pygame.JOYBUTTONDOWN, instance_id=42, button=3)]
                with patch('pygame.event.get', return_value=events):
                    batch = gamepad.poll()
                self.assertEqual((batch.state.left_x, batch.state.left_y, batch.state.right_x), (-0.2, -0.4, -0.6))
                self.assertEqual(batch.events, (('hat', (0, 1)), ('button', (3, True))))
        self.assertTrue(joystick.closed)

    def test_removed_selected_controller_raises_and_closes_handle(self):
        joystick = FakeJoystick()
        with patch('pygame.joystick.get_count', return_value=1), \
             patch('pygame.joystick.Joystick', return_value=joystick):
            with app.PygameGamepad() as gamepad:
                event = pygame.event.Event(pygame.JOYDEVICEREMOVED, instance_id=42)
                with patch('pygame.event.get', return_value=[event]), self.assertRaises(app.GamepadDisconnected):
                    gamepad.poll()
        self.assertTrue(joystick.closed)


class GamepadDisplayTests(unittest.TestCase):
    def test_dashboard_shows_cruise_velocity_angle_and_hat_controls(self):
        controller = app.GamepadController()
        controller.toggle_mode(1.0)
        controller.update(app.GamepadState(hat=(0, 1), right_x=-1), 1.0)
        controller.command(2.0)
        output = io.StringIO()
        Console(file=output, width=140, color_system=None).print(
            app.render_dashboard(controller.command(2.0), controller, 'loop://', 115200, 12, 'Xbox test'))
        for expected in ('Hat', '巡航', '角度模式', 'yaw', 'Y', 'B', '0.100', 'Ctrl+L', '没有主控接收确认'):
            self.assertIn(expected, output.getvalue())


class GamepadSessionTests(unittest.TestCase):
    def exercise(self, states, focus=None, fail_first_motion=False):
        clock = SimpleNamespace(now=0.0, step=0)
        focus = focus or {}
        submitted = []
        failed = False

        class Terminal:
            focused = True

            def __enter__(self): return self
            def __exit__(self, *args): return False
            def read(self, timeout):
                clock.step += 1
                clock.now += 0.1
                if clock.step > max(states):
                    raise RuntimeError('Start button was not handled')
                self.focused = focus.get(clock.step, self.focused)
                return ''

        gamepad = SimpleNamespace(name='Test Xbox', poll=lambda: app.GamepadBatch(states.get(clock.step, app.GamepadState())))
        args = SimpleNamespace(port='loop://', baud=115200, max_speed=0.8, speed_step=0.1,
                               yaw_rate=0.5, deadzone=0.08, button_mode=3, button_stop=1, button_exit=7)
        with serial.serial_for_url('loop://', timeout=0, write_timeout=0.1) as link:
            real_write = app.write_command

            def record(command_link, command):
                nonlocal failed
                submitted.append((clock.now, clock.step, command))
                if fail_first_motion and command.vx and not failed:
                    failed = True
                    raise serial.SerialTimeoutException('Write timeout')
                real_write(command_link, command)

            with patch('chassis_gamepad.GamepadTerminal', return_value=Terminal()), \
                 patch('chassis_gamepad.time.monotonic', side_effect=lambda: clock.now), \
                 patch('chassis_gamepad.write_command', side_effect=record):
                app.run_control(link, args, Console(file=io.StringIO(), width=140), gamepad)
        return submitted

    def test_paused_input_changes_do_not_retry_faster_than_four_hertz(self):
        states = {1: app.GamepadState(hat=(0, 1)),
                  2: app.GamepadState(right_x=-1), 3: app.GamepadState(right_x=1),
                  4: app.GamepadState(right_x=-1), 5: app.GamepadState(right_x=1),
                  6: app.GamepadState(right_x=-1), 7: app.GamepadState(buttons=frozenset({7}))}
        submitted = self.exercise(states, fail_first_motion=True)
        retries = [stamp for stamp, step, command in submitted if step > 1]
        self.assertGreaterEqual(len(retries), 1)
        self.assertTrue(all(command == Command() for stamp, step, command in submitted if step > 1))
        self.assertGreaterEqual(retries[0] - 0.1, 0.25 - 1e-9)
        self.assertTrue(all(right - left >= 0.25 - 1e-9 for left, right in zip(retries, retries[1:])))

    def test_space_equivalent_button_recovers_without_resuming_held_stick(self):
        states = {1: app.GamepadState(left_y=-1), 2: app.GamepadState(left_y=-1),
                  3: app.GamepadState(left_y=-1), 4: app.GamepadState(left_y=-1, buttons=frozenset({1})),
                  5: app.GamepadState(left_y=-1), 6: app.GamepadState(),
                  7: app.GamepadState(left_y=-1), 8: app.GamepadState(buttons=frozenset({7}))}
        submitted = self.exercise(states, fail_first_motion=True)
        self.assertTrue(all(command == Command() for stamp, step, command in submitted if 1 < step <= 6))
        self.assertTrue(any(step == 7 and command.vx > 0 for stamp, step, command in submitted))

    def test_focus_return_blocks_current_stick_and_clears_cruise(self):
        states = {1: app.GamepadState(hat=(0, 1)), 2: app.GamepadState(),
                  3: app.GamepadState(left_y=-1), 4: app.GamepadState(left_y=-1),
                  5: app.GamepadState(), 6: app.GamepadState(left_y=-1),
                  7: app.GamepadState(buttons=frozenset({7}))}
        submitted = self.exercise(states, focus={2: False, 3: True})
        self.assertTrue(all(command == Command() for stamp, step, command in submitted if 2 <= step <= 5))
        self.assertTrue(any(step == 6 and command.vx > 0 for stamp, step, command in submitted))

    def test_terminal_focus_reports_preserve_ctrl_l_space_and_tty_settings(self):
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        try:
            with os.fdopen(os.dup(slave), 'r') as stream, patch('sys.stdout', new=io.StringIO()):
                with app.GamepadTerminal(stream) as terminal:
                    os.write(master, b'\x1b[')
                    self.assertEqual(terminal.read(0.1), '')
                    os.write(master, b'O\x0c ')
                    self.assertEqual(terminal.read(0.1), '\x0c ')
                    self.assertFalse(terminal.focused)
                    os.write(master, b'\x1b[I')
                    self.assertEqual(terminal.read(0.1), '')
                    self.assertTrue(terminal.focused)
            self.assertEqual(termios.tcgetattr(slave), original)
        finally:
            os.close(master)
            os.close(slave)

    def test_complete_cli_encodes_velocity_angle_and_final_stop_over_uart_pty(self):
        keyboard_master, keyboard_slave = pty.openpty()
        uart_master, uart_slave = pty.openpty()
        termios.tcsetwinsize(keyboard_slave, (36, 140))
        original = termios.tcgetattr(keyboard_slave)
        script_dir = Path(__file__).resolve().parents[1]
        program = f'''import sys
sys.path.insert(0, {str(script_dir)!r})
import chassis_gamepad as app
class Pad:
    name = 'Simulated Xbox 360'
    def __init__(self, *args):
        self.step = 0
        self.previous = app.GamepadState()
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def poll(self):
        self.step += 1
        states = {{
            2: app.GamepadState(hat=(0,1)),
            4: app.GamepadState(buttons=frozenset({{3}})),
            6: app.GamepadState(right_x=-1),
            7: app.GamepadState(right_x=-1),
            8: app.GamepadState(right_x=-1),
            9: app.GamepadState(right_x=-1),
            10: app.GamepadState(right_x=-1),
            12: app.GamepadState(buttons=frozenset({{1}})),
            14: app.GamepadState(buttons=frozenset({{7}})),
        }}
        state = states.get(self.step, app.GamepadState())
        events = []
        if state.hat != self.previous.hat:
            events.append(('hat', state.hat))
        for button in state.buttons - self.previous.buttons:
            events.append(('button', (button, True)))
        for button in self.previous.buttons - state.buttons:
            events.append(('button', (button, False)))
        self.previous = state
        return app.GamepadBatch(state, tuple(events))
app.PygameGamepad = Pad
sys.exit(app.main(['--port', {os.ttyname(uart_slave)!r}]))
'''
        process = subprocess.Popen([sys.executable, '-c', program], stdin=keyboard_slave,
                                   stdout=keyboard_slave, stderr=keyboard_slave,
                                   env={**os.environ, 'TERM': 'xterm-256color', 'PYTHONIOENCODING': 'utf-8'})
        ui = bytearray()
        received = bytearray()
        packets = []
        try:
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                for fd in select.select([keyboard_master, uart_master], [], [], 0.05)[0]:
                    data = os.read(fd, 65536)
                    if fd == keyboard_master:
                        ui.extend(data)
                    else:
                        received.extend(data)
                while len(received) >= 2:
                    self.assertIn(received[:2], (b're', b'rm'))
                    length = 23 if received[1] == ord('e') else 27
                    if len(received) < length:
                        break
                    frame = bytes(received[:length])
                    del received[:length]
                    self.assertEqual(frame[-1], ord('d'))
                    self.assertEqual(struct.unpack('<I', frame[-5:-1])[0], crc32_stm32(frame[2:-5]))
                    packets.append((length, struct.unpack('<4f', frame[2:18])))
                if process.poll() is not None and not select.select([keyboard_master, uart_master], [], [], 0.05)[0]:
                    break
            self.assertEqual(process.wait(timeout=2), 0, ui.decode(errors='replace')[-1600:])
            self.assertTrue(any(length == 23 and fields[0] > 0.09 for length, fields in packets))
            self.assertTrue(any(length == 27 and fields[2] > 0.001 for length, fields in packets))
            self.assertEqual(packets[-1], (23, (0, 0, 0, 0)))
            self.assertEqual(termios.tcgetattr(keyboard_slave), original)
            self.assertIn('角度模式', ui.decode(errors='replace'))
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=2)
            for fd in (keyboard_master, keyboard_slave, uart_master, uart_slave):
                os.close(fd)


if __name__ == '__main__':
    unittest.main()
