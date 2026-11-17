import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from fr5_rviz_sync.conversion import degrees_to_joint_positions
from fr5_rviz_sync.model import load_live_description


class ConversionTests(unittest.TestCase):
    def test_real_feedback_and_one_degree_motion(self):
        start = [97.731, -0.285, -134.788, 2.563, 6.635, 26.677]
        finish = start[:]
        finish[5] += 1.0
        converted = degrees_to_joint_positions(start)
        moved = degrees_to_joint_positions(finish)
        self.assertAlmostEqual(converted[2], -2.352494392178117, places=10)
        self.assertAlmostEqual(moved[5] - converted[5], math.pi / 180, places=12)
        self.assertEqual(converted[:5], moved[:5])

    def test_signed_angles_and_revolutions_are_preserved(self):
        result = degrees_to_joint_positions([0, 90, -90, 180, -180, 720])
        for actual, expected in zip(result, [0, math.pi / 2, -math.pi / 2, math.pi, -math.pi, 4 * math.pi]):
            self.assertAlmostEqual(actual, expected)

    def test_bad_feedback_rejected(self):
        for values in ([1, 2], [0] * 7, [float('nan')] * 6, [float('inf')] * 6):
            with self.assertRaises(ValueError):
                degrees_to_joint_positions(values)

    def test_hardware_urdf_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.urdf'
            root = ET.Element('robot', name='fr5')
            for index in range(1, 7):
                ET.SubElement(root, 'joint', name=f'j{index}', type='revolute')
            ET.SubElement(root, 'ros2_control', name='real_hardware')
            ET.ElementTree(root).write(path)
            with self.assertRaises(ValueError):
                load_live_description(path)


if __name__ == '__main__':
    unittest.main()
