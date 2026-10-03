import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import chassis_keyboard as app


class FirmwareYawTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('cc'):
            raise unittest.SkipTest('Host C compiler is needed for firmware integration tests')
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        temp = Path(cls.temp.name)
        tests = Path(__file__).resolve().parent
        root = tests.parents[1]
        for name in ('can.h', 'crc.h', 'usart.h', 'static_mem.h', 'stream_buffer.h',
                     'motor.h', 'wheel.h', 'm3508.h', 'generic_def.h'):
            (temp / name).write_text('#include "firmware_stubs.h"\n')
        cls.executable = temp / 'firmware_yaw'
        result = subprocess.run(['cc', '-std=c11', '-O0', '-Wall', '-Wextra', '-Wno-unused-function',
                                 '-I', str(temp), '-I', str(tests),
                                 '-I', str(root / 'User/Modules/Interface'),
                                 '-I', str(root / 'User/Utils/Interface'),
                                 str(tests / 'firmware_yaw_harness.c'),
                                 str(root / 'User/Utils/Src/pid.c'), '-lm', '-o', str(cls.executable)],
                                capture_output=True, text=True)
        if result.returncode:
            raise AssertionError('Firmware harness did not compile:\n' + result.stderr)
        result = subprocess.run([cls.executable, 'config'], capture_output=True, text=True, check=True)
        cls.kp, cls.ki, cls.out_min, cls.out_max = map(float, result.stdout.split())

    def receive(self, data):
        result = subprocess.run([self.executable], input=data, capture_output=True, check=True)
        return [[float(field) for field in line.split()] for line in result.stdout.decode().splitlines()]

    def rotation(self, mode, target, rate, measurement, ready=1):
        result = subprocess.run([self.executable, str(mode), str(target), str(rate), str(measurement), str(ready)],
                                capture_output=True, text=True, check=True)
        return result.stdout.strip()

    def test_python_velocity_and_angle_frames_reach_real_remote_parser(self):
        rate = app.encode_frame(app.Command(vx=0.8, d_yaw=-0.5))
        angle = app.encode_frame(app.Command(vy=0.8, yaw=math.pi / 2, mode=0))
        stop = app.encode_frame(app.Command())
        commands = self.receive(b'noise\x72' + rate + angle + stop)
        self.assertEqual(len(commands), 3)
        self.assertEqual([cmd[0] for cmd in commands], [1, 0, 1])
        self.assertAlmostEqual(commands[0][1], 0.8, places=6)
        self.assertEqual(commands[0][4], -0.5)
        self.assertAlmostEqual(commands[1][3], math.pi / 2, places=6)
        self.assertEqual(commands[2], [1, 0, 0, 0, 0])

    def test_crc_covers_mode_and_corrupted_frame_recovers(self):
        frame = app.encode_frame(app.Command(yaw=0.5, mode=0))
        corrupt = bytearray(frame)
        corrupt[18] = 1
        self.assertEqual(self.receive(corrupt), [])
        self.assertEqual(self.receive(bytes(corrupt) + frame), [[0, 0, 0, 0.5, 0]])

    def test_bad_long_frame_containing_short_frame_preserves_following_bytes(self):
        stop = app.encode_frame(app.Command())
        angle = app.encode_frame(app.Command(yaw=0.5, mode=0))
        commands = self.receive(b'rm\x00' + stop + angle)
        self.assertEqual(commands, [[1, 0, 0, 0, 0], [0, 0, 0, 0.5, 0]])

    def test_unknown_mode_with_valid_crc_is_rejected(self):
        payload = struct.pack('<4fI', 0, 0, 0.5, 0, 2)
        frame = b'rm' + payload + struct.pack('<I', app.crc32_stm32(payload)) + b'd'
        self.assertEqual(self.receive(frame), [])
        self.assertEqual(self.rotation(2, 0, 0, 0), 'rejected')

    def test_velocity_mode_bypasses_yaw_pid_even_without_imu(self):
        self.assertAlmostEqual(float(self.rotation(1, 2, -0.3, -2, 0)), -0.3, places=6)

    def test_angle_mode_uses_real_yaw_pid_limits_and_wrap(self):
        error = 0.1
        expected = min(self.out_max, max(self.out_min, (self.kp + self.ki * 0.01) * error))
        self.assertAlmostEqual(float(self.rotation(0, error, -0.5, 0)), expected, places=6)
        error = 2.0
        expected = min(self.out_max, max(self.out_min, (self.kp + self.ki * 0.01) * error))
        self.assertAlmostEqual(float(self.rotation(0, error, 0, 0)), expected, places=5)
        target, measurement = math.radians(-179), math.radians(179)
        expected = min(self.out_max, max(self.out_min, (self.kp + self.ki * 0.01) * math.radians(2)))
        self.assertAlmostEqual(float(self.rotation(0, target, 0, measurement)), expected, places=5)

    def test_angle_mode_with_no_sample_returns_zero_rotation(self):
        self.assertEqual(float(self.rotation(0, 2, 0.5, 0, 0)), 0.0)

    def test_angle_output_is_held_between_ten_millisecond_updates(self):
        result = subprocess.run([self.executable, '0', '0.1', '0', '0', '1', '12'],
                                capture_output=True, text=True, check=True)
        rotations = list(map(float, result.stdout.splitlines()))
        self.assertEqual(len(rotations), 12)
        self.assertGreater(rotations[0], 0)
        self.assertEqual(rotations[:10], [rotations[0]] * 10)
        self.assertGreaterEqual(rotations[10], rotations[0])
        self.assertEqual(rotations[10], rotations[11])

    def test_stop_velocity_command_does_not_return_to_yaw_zero(self):
        self.assertEqual(float(self.rotation(1, 0, 0, 1.5)), 0.0)


if __name__ == '__main__':
    unittest.main()
