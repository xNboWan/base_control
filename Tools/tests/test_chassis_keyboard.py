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
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import serial
from rich.console import Console

from chassis_keyboard import (
    Command,
    KeyboardController,
    TerminalKeyboard,
    crc32_stm32,
    encode_frame,
    render_dashboard,
    run_control,
    write_command,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_matches_f427_reference_vector(self):
        self.assertEqual(crc32_stm32(bytes(range(16))), 0x081B46CA)

    def test_frame_field_order_and_little_endian(self):
        command = Command(0.3, -0.2, 0.0, 0.5)
        frame = encode_frame(command)
        self.assertEqual(len(frame), 23)
        self.assertEqual(frame[:2], b"\x72\x65")
        self.assertEqual(frame[-1:], b"\x64")
        self.assertEqual(frame[2:18], struct.pack("<4f", 0.3, -0.2, 0.0, 0.5))
        self.assertEqual(struct.unpack("<I", frame[18:22])[0], crc32_stm32(frame[2:18]))

    def test_invalid_commands_are_rejected(self):
        for field in range(4):
            for value in (math.nan, math.inf, -math.inf, 1e40):
                with self.subTest(field=field, value=value):
                    values = [0.0] * 4
                    values[field] = value
                    with self.assertRaises(ValueError):
                        encode_frame(Command(*values))

    def test_crc_rejects_wrong_payload_size(self):
        for size in (0, 15, 17):
            with self.assertRaises(ValueError):
                crc32_stm32(bytes(size))

    def test_partial_write_is_reported(self):
        link = Mock()
        link.write.return_value = 8
        with self.assertRaises(serial.SerialException):
            write_command(link, Command())

    def test_real_pyserial_loopback(self):
        with serial.serial_for_url("loop://", timeout=0.1, write_timeout=0.1) as link:
            command = Command(0.3, 0.0, 0.0, -0.5)
            write_command(link, command)
            self.assertEqual(link.read(23), encode_frame(command))


@patch("chassis_keyboard.LINEAR_SPEED", 0.3)
class KeyboardTests(unittest.TestCase):
    def test_key_directions_and_fixed_speed(self):
        expected = {
            "w": (0.3, 0.0, 0.0), "s": (-0.3, 0.0, 0.0),
            "a": (0.0, 0.3, 0.0), "d": (0.0, -0.3, 0.0),
            "q": (0.0, 0.0, 0.5), "e": (0.0, 0.0, -0.5),
        }
        for key, values in expected.items():
            with self.subTest(key=key):
                controller = KeyboardController(key_release_events=False)
                controller.press(key, 10.0)
                command = controller.command(10.1)
                self.assertEqual((command.vx, command.vy, command.d_yaw), values)
                self.assertEqual(command.yaw, 0.0)

    def test_timeout_stops_without_key_release(self):
        controller = KeyboardController(key_release_events=False)
        controller.press("w", 1.0)
        self.assertEqual(controller.command(1.249).vx, 0.3)
        self.assertEqual(controller.command(1.25), Command())

    def test_repeated_input_renews_deadline(self):
        controller = KeyboardController(key_release_events=False)
        controller.press("w", 1.0)
        controller.press("W", 1.2)
        self.assertEqual(controller.command(1.4).vx, 0.3)
        self.assertEqual(controller.command(1.451), Command())

    def test_unknown_key_does_not_extend_motion(self):
        controller = KeyboardController(key_release_events=False)
        controller.press("w", 1.0)
        controller.press("z", 1.2)
        self.assertEqual(controller.command(1.3), Command())

    def test_opposite_direction_replaces_current_axis(self):
        controller = KeyboardController(key_release_events=False)
        controller.press("w", 1.0)
        controller.press("s", 1.1)
        self.assertEqual(controller.command(1.2).vx, -0.3)

    def test_diagonal_speed_stays_at_point_three(self):
        controller = KeyboardController(key_release_events=False)
        controller.press("w", 1.0)
        controller.press("a", 1.0)
        command = controller.command(1.1)
        self.assertGreater(command.vx, 0.0)
        self.assertGreater(command.vy, 0.0)
        self.assertAlmostEqual(math.hypot(command.vx, command.vy), 0.3)

    def test_axes_expire_independently(self):
        controller = KeyboardController(key_release_events=False)
        controller.press("w", 1.0)
        controller.press("q", 1.1)
        command = controller.command(1.3)
        self.assertEqual(command.vx, 0.0)
        self.assertEqual(command.d_yaw, 0.5)

    def test_stop_keys_clear_all_axes(self):
        for key in (" ", "\r", "\n"):
            controller = KeyboardController(key_release_events=False)
            for movement in "waq":
                controller.press(movement, 1.0)
            controller.press(key, 1.1)
            self.assertEqual(controller.command(1.1), Command())
            self.assertEqual(controller.command(1.2), Command())

    def test_config_validation(self):
        for parameter in ("yaw_rate", "hold_time"):
            for value in (0, -1, math.nan, math.inf):
                with self.assertRaises(ValueError):
                    KeyboardController(**{parameter: value})

    def test_terminal_settings_restore_after_exception(self):
        master, slave = pty.openpty()
        try:
            original = termios.tcgetattr(slave)
            with os.fdopen(os.dup(slave), "r") as stream:
                with self.assertRaises(RuntimeError):
                    with TerminalKeyboard(stream) as keyboard:
                        self.assertFalse(termios.tcgetattr(slave)[3] & termios.ECHO)
                        os.write(master, b"w")
                        self.assertEqual(keyboard.read(0.2), "w")
                        raise RuntimeError("simulated exit")
            self.assertEqual(termios.tcgetattr(slave), original)
        finally:
            os.close(master)
            os.close(slave)


class TerminalTests(unittest.TestCase):
    def test_dashboard_shows_target_command_and_controls(self):
        output = io.StringIO()
        console = Console(file=output, width=100, color_system=None)
        console.print(render_dashboard(Command(0.3, 0, 0, -0.5), "loop://", 12, "W", 0.25, 0.5))
        text = output.getvalue()
        for expected in ("目标速度指令", "vx", "vy", "d_yaw", "+0.300", "-0.500", "W", "Q", "空格"):
            self.assertIn(expected, text)

    def test_dashboard_uses_selected_baudrate(self):
        output = io.StringIO()
        Console(file=output, width=100, color_system=None).print(
            render_dashboard(Command(), "loop://", 0, "—", 0.25, 0.5, 57600))
        self.assertIn("57600", output.getvalue())
        self.assertNotIn("115200", output.getvalue())

    def test_dashboard_explains_pause_and_unconfirmed_delivery(self):
        output = io.StringIO()
        Console(file=output, width=140, color_system=None).print(
            render_dashboard(Command(), "/dev/fake", 2, "W", 0.25, 0.5,
                             warning="Write timeout；空格重试并恢复控制。"))
        for expected in ("控制暂停", "Write timeout", "空格", "没有主控接收确认", "串口提交"):
            self.assertIn(expected, output.getvalue())

    def test_cli_persistent_timeout_keeps_screen_open_and_restores_terminal(self):
        master, slave = pty.openpty()
        termios.tcsetwinsize(slave, (35, 120))
        original = termios.tcgetattr(slave)
        script_directory = Path(__file__).resolve().parents[1]
        program = (
            "import sys\n"
            f"sys.path.insert(0, {str(script_directory)!r})\n"
            "import chassis_keyboard as app\n"
            "class BlockedSerial:\n"
            "    def write(self, data):\n"
            "        raise app.serial.SerialTimeoutException('Write timeout')\n"
            "    def reset_output_buffer(self): pass\n"
            "    def close(self): pass\n"
            "app.serial.serial_for_url = lambda *a, **kw: BlockedSerial()\n"
            "sys.exit(app.main(['--port', '/dev/fake', '--input', 'terminal']))\n"
        )
        process = subprocess.Popen(
            [sys.executable, "-c", program], stdin=slave, stdout=slave, stderr=slave,
            env={**os.environ, "TERM": "xterm-256color", "PYTHONIOENCODING": "utf-8"},
        )
        output = bytearray()
        try:
            deadline = time.monotonic() + 3
            while "控制暂停" not in output.decode(errors="replace") and time.monotonic() < deadline:
                if select.select([master], [], [], 0.05)[0]:
                    output.extend(os.read(master, 65536))
            self.assertIn("控制暂停", output.decode(errors="replace"))
            self.assertIsNone(process.poll())
            os.write(master, b"x")
            self.assertEqual(process.wait(timeout=2), 1)  # 最后的停止帧也发送失败。
            while select.select([master], [], [], 0)[0]:
                output.extend(os.read(master, 65536))
            self.assertEqual(termios.tcgetattr(slave), original)
            self.assertIn("停止帧发送失败", output.decode(errors="replace"))
            self.assertNotIn("控制已结束：Write timeout", output.decode(errors="replace"))
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=2)
            os.close(master)
            os.close(slave)

    def test_complete_cli_over_two_pseudo_terminals(self):
        keyboard_master, keyboard_slave = pty.openpty()
        uart_master, uart_slave = pty.openpty()
        termios.tcsetwinsize(keyboard_slave, (30, 100))
        original = termios.tcgetattr(keyboard_slave)
        env = {**os.environ, "TERM": "xterm-256color", "PYTHONIOENCODING": "utf-8"}
        script = Path(__file__).resolve().parents[1] / "chassis_keyboard.py"
        program = (
            "import sys\n"
            f"sys.path.insert(0, {str(script.parent)!r})\n"
            "import chassis_keyboard as app\n"
            "app.LINEAR_SPEED = 0.3\n"
            "sys.exit(app.main(sys.argv[1:]))\n"
        )
        process = subprocess.Popen(
            [sys.executable, "-c", program, "--port", os.ttyname(uart_slave), "--input", "terminal"],
            stdin=keyboard_slave, stdout=keyboard_slave, stderr=keyboard_slave, env=env,
        )
        buffer = bytearray()
        commands = []
        ui = bytearray()

        def receive_until(predicate, timeout=3.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                ready, _, _ = select.select([keyboard_master, uart_master], [], [], 0.05)
                for fd in ready:
                    data = os.read(fd, 65536)
                    if fd == keyboard_master:
                        ui.extend(data)
                    else:
                        buffer.extend(data)
                        while len(buffer) >= 23:
                            packet = bytes(buffer[:23])
                            del buffer[:23]
                            self.assertEqual(packet[:2], b"\x72\x65")
                            self.assertEqual(packet[-1], 0x64)
                            self.assertEqual(struct.unpack("<I", packet[18:22])[0], crc32_stm32(packet[2:18]))
                            commands.append(struct.unpack("<4f", packet[2:18]))
                if predicate():
                    return
            self.fail(f"CLI condition not reached; exit={process.poll()}, output={ui.decode(errors='replace')[-1200:]}")

        try:
            receive_until(lambda: bool(commands))
            self.assertEqual(commands[-1], (0, 0, 0, 0))
            os.write(keyboard_master, b"w")
            receive_until(lambda: commands[-1][0] > 0.29)
            active_count = len(commands)
            receive_until(lambda: len(commands) > active_count and commands[-1] == (0, 0, 0, 0))
            os.write(keyboard_master, b"e")
            receive_until(lambda: commands[-1][3] < -0.49)
            os.write(keyboard_master, b" ")
            receive_until(lambda: commands[-1] == (0, 0, 0, 0))
            os.write(keyboard_master, b"wq")
            receive_until(lambda: commands[-1][0] > 0.29 and commands[-1][3] > 0.49)
            before_exit = len(commands)
            os.write(keyboard_master, b"\x1b")
            receive_until(lambda: process.poll() is not None)
            self.assertEqual(process.wait(timeout=2), 0)
            self.assertGreater(len(commands), before_exit)
            self.assertEqual(commands[-1], (0, 0, 0, 0))
            self.assertEqual(termios.tcgetattr(keyboard_slave), original)
            self.assertIn("目标速度指令", ui.decode(errors="replace"))
            self.assertIn("0.300", ui.decode(errors="replace"))
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=2)
            for fd in (keyboard_master, keyboard_slave, uart_master, uart_slave):
                os.close(fd)


class SendFailureTests(unittest.TestCase):
    def exercise(self, keys, fail_write):
        clock = SimpleNamespace(now=0.0, step=0)
        inputs = iter(keys)
        attempts = []
        link = Mock()
        link.in_waiting = 0

        def read(timeout):
            clock.now += 0.1
            clock.step += 1
            return next(inputs)

        def write(frame):
            command = struct.unpack("<4f", frame[2:18])
            attempts.append((clock.step, command))
            result = fail_write(clock.step, command, len(attempts))
            if type(result) is int:
                return result
            if result:
                raise serial.SerialTimeoutException("Write timeout")
            return len(frame)

        keyboard = Mock()
        keyboard.__enter__ = Mock(return_value=keyboard)
        keyboard.__exit__ = Mock(return_value=False)
        keyboard.read.side_effect = read
        live = Mock()
        live.__enter__ = Mock(return_value=live)
        live.__exit__ = Mock(return_value=False)
        link.write.side_effect = write
        args = SimpleNamespace(port="/dev/fake", baud=115200, yaw_rate=0.5, hold_time=0.25)
        with patch("chassis_keyboard.TerminalKeyboard", return_value=keyboard), \
             patch("chassis_keyboard.Live", return_value=live), \
             patch("chassis_keyboard.time.monotonic", side_effect=lambda: clock.now):
            sent = run_control(link, args, Console(file=io.StringIO()))
        return attempts, sent, link

    def test_startup_timeout_keeps_interface_running_and_requires_space_to_resume(self):
        attempts, sent, link = self.exercise(
            ["w", "", "", "w", " ", "w", "", "x"],
            lambda step, command, count: count == 1,
        )
        self.assertTrue(all(command == (0, 0, 0, 0) for step, command in attempts if step <= 5))
        self.assertTrue(any(step >= 6 and command[0] > 0.29 for step, command in attempts))
        self.assertGreater(sent, 0)
        link.reset_output_buffer.assert_called()

    def test_moving_timeout_clears_all_axes_and_ignores_motion_until_space(self):
        failed = False

        def fail_once(step, command, count):
            nonlocal failed
            if not failed and command[0] > 0:
                failed = True
                return True
            return False

        attempts, _, _ = self.exercise(
            ["waq", "", "waq", "", "", "w", " ", "", "d", "", "x"], fail_once,
        )
        self.assertTrue(failed)
        self.assertTrue(all(command == (0, 0, 0, 0) for step, command in attempts if 1 < step <= 8))
        self.assertTrue(any(step >= 9 and command[1] < -0.29 for step, command in attempts))
        self.assertTrue(all(command[0] == 0 and command[3] == 0 for step, command in attempts if step >= 9))

    def test_persistent_timeouts_still_allow_exit_and_only_retry_zero(self):
        attempts, sent, link = self.exercise(
            ["waq", "", "w", "", " ", "", "", "", "x"],
            lambda step, command, count: True,
        )
        self.assertEqual(sent, 0)
        self.assertGreaterEqual(len(attempts), 2)
        self.assertTrue(all(command == (0, 0, 0, 0) for _, command in attempts))
        self.assertEqual(link.reset_output_buffer.call_count, len(attempts))

    def test_disconnect_is_not_treated_as_a_recoverable_timeout(self):
        def disconnected(step, command, count):
            raise serial.SerialException("device disconnected")

        with self.assertRaisesRegex(serial.SerialException, "device disconnected"):
            self.exercise(["", "x"], disconnected)

    def test_partial_frame_clears_motion_and_recovers_with_a_complete_zero_frame(self):
        attempts, _, link = self.exercise(
            ["waq", "", "w", "", " ", "d", "", "x"],
            lambda step, command, count: 8 if count == 2 else False,
        )
        self.assertTrue(all(command == (0, 0, 0, 0) for step, command in attempts if 1 < step <= 5))
        self.assertTrue(any(step >= 6 and command[1] < -0.29 for step, command in attempts))
        self.assertTrue(all(command[0] == 0 and command[3] == 0 for step, command in attempts if step >= 6))
        link.reset_output_buffer.assert_called_once()


if __name__ == "__main__":
    unittest.main()
