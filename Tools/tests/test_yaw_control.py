import io
import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import evdev
from rich.console import Console
import chassis_keyboard as app
from test_keyboard_events import TestInputDevice


class AngleControlTests(unittest.TestCase):
    def angle_controller(self, **kwargs):
        controller = app.KeyboardController(**kwargs)
        controller.press('\x0c', 1.0)
        self.assertEqual(controller.mode, 'angle')
        return controller

    def test_ctrl_l_switches_modes_and_blocks_keys_already_held(self):
        controller = app.KeyboardController()
        controller.press('w', 1.0)
        controller.press('q', 1.0)
        self.assertTrue(controller.press('\x0c', 1.1))
        controller.set_pressed_keys({'w', 'q'}, 1.2)
        self.assertEqual(controller.command(2.0), app.Command(mode=0))
        controller.release('q', 2.0)
        controller.press('q', 2.1)
        self.assertAlmostEqual(controller.command(3.1).yaw, 0.5)
        controller.press('\x0c', 3.1)
        self.assertEqual(controller.mode, 'velocity')
        controller.set_pressed_keys({'w', 'q'}, 3.1)
        self.assertEqual(controller.command(4.0), app.Command())

    def test_hold_changes_target_continuously_and_release_keeps_target(self):
        controller = self.angle_controller()
        controller.press('q', 1.0)
        self.assertAlmostEqual(controller.command(2.0).yaw, 0.5)
        self.assertAlmostEqual(controller.command(3.0).yaw, 1.0)
        controller.release('q', 3.0)
        held = controller.command(10.0)
        self.assertEqual(held.mode, 0)
        self.assertEqual(held.d_yaw, 0.0)
        self.assertAlmostEqual(held.yaw, 1.0)
        controller.press('e', 10.0)
        self.assertAlmostEqual(controller.command(11.0).yaw, 0.5)

    def test_angle_mode_preserves_translation_and_combination_keys(self):
        controller = self.angle_controller()
        for key in 'waq':
            controller.press(key, 1.0)
        command = controller.command(2.0)
        self.assertAlmostEqual(math.hypot(command.vx, command.vy), app.LINEAR_SPEED)
        self.assertAlmostEqual(command.yaw, 0.5)
        controller.release('a', 2.0)
        self.assertEqual(controller.command(2.0).vx, app.LINEAR_SPEED)
        self.assertEqual(controller.command(2.0).vy, 0.0)

    def test_opposite_rotation_keys_cancel_and_angle_wraps(self):
        controller = self.angle_controller()
        controller.press('q', 1.0)
        controller.press('e', 2.0)
        self.assertAlmostEqual(controller.command(5.0).yaw, 0.5)
        controller.release('e', 5.0)
        self.assertAlmostEqual(controller.command(12.0).yaw, 4.0 - 2 * math.pi)

    def test_repeated_render_at_same_time_does_not_change_target(self):
        controller = self.angle_controller()
        controller.press('q', 1.0)
        controller.command(1.5)
        first = controller.command(2.0)
        self.assertEqual(controller.command(2.0), first)
        self.assertAlmostEqual(first.yaw, 0.5)

    def test_stop_disables_angle_loop_and_requires_fresh_press(self):
        controller = self.angle_controller()
        controller.press('q', 1.0)
        controller.press(' ', 2.0)
        controller.set_pressed_keys({'q'}, 3.0)
        self.assertEqual(controller.mode, 'angle')
        self.assertEqual(controller.command(4.0), app.Command())
        controller.release('q', 4.0)
        controller.press('q', 4.1)
        command = controller.command(5.1)
        self.assertEqual(command.mode, 0)
        self.assertAlmostEqual(command.yaw, 1.0)

    def test_terminal_mode_integrates_only_until_key_deadline(self):
        controller = self.angle_controller(key_release_events=False)
        controller.press('q', 1.0)
        command = controller.command(10.0)
        self.assertEqual(command.mode, 0)
        self.assertAlmostEqual(command.yaw, 0.125)

    def test_dashboard_shows_mode_target_angle_and_toggle_key(self):
        output = io.StringIO()
        Console(file=output, width=140, color_system=None).print(
            app.render_dashboard(app.Command(yaw=math.pi / 2, mode=0),
                                 'loop://', 12, 'Ctrl+L', 0.25, 0.5,
                                 control_mode='angle', target_yaw=math.pi / 2))
        for expected in ('角度模式', 'yaw', '+90.0', 'Ctrl+L', '保持'):
            self.assertIn(expected, output.getvalue())


class ModeProtocolTests(unittest.TestCase):
    def test_mode_frame_contains_angle_radians_and_crc_covers_mode(self):
        frame = app.encode_frame(app.Command(0.8, -0.2, math.pi / 2, 0.0, mode=0))
        self.assertEqual(len(frame), 27)
        self.assertEqual(frame[:2], b'rm')
        self.assertEqual(frame[-1:], b'd')
        self.assertEqual(frame[2:18], struct.pack('<4f', 0.8, -0.2, math.pi / 2, 0.0))
        self.assertEqual(frame[18:22], b'\0\0\0\0')
        self.assertEqual(struct.unpack('<I', frame[22:26])[0], app.crc32_stm32(frame[2:22]))
        velocity_frame = app.encode_frame(app.Command(0.8, -0.2, math.pi / 2, 0.0, mode=1))
        self.assertNotEqual(frame[22:26], velocity_frame[22:26])

    def test_invalid_mode_is_rejected(self):
        for mode in (-1, 2, 1.5, 'angle'):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                app.encode_frame(app.Command(mode=mode))

    def test_zero_and_velocity_frames_keep_the_legacy_protocol(self):
        self.assertEqual(app.encode_frame(app.Command()).hex(),
                         '726500000000000000000000000000000000c8222d5564')
        self.assertEqual(len(app.encode_frame(app.Command(d_yaw=0.5))), 23)


class ModeChordTests(unittest.TestCase):
    def test_either_ctrl_l_toggles_once_and_plain_l_does_not(self):
        e = evdev.ecodes
        for ctrl in (e.KEY_LEFTCTRL, e.KEY_RIGHTCTRL):
            with self.subTest(ctrl=ctrl):
                device = TestInputDevice()
                try:
                    with patch('evdev.InputDevice', return_value=device):
                        with app.LinuxKeyboard('/dev/input/test') as keyboard:
                            for code, value in ((e.KEY_L, 1), (e.KEY_L, 0),
                                                (ctrl, 1), (e.KEY_L, 1),
                                                (e.KEY_L, 2), (e.KEY_L, 2),
                                                (ctrl, 0), (e.KEY_L, 0)):
                                device.emit(code, value)
                            events = keyboard.read(0)
                            self.assertEqual([key for key, down in events if down], ['\x0c'])
                            device.emit(ctrl, 1)
                            device.emit(e.KEY_L, 1)
                            self.assertEqual([key for key, down in keyboard.read(0) if down], ['\x0c'])
                finally:
                    device.close()


if __name__ == '__main__':
    unittest.main()
