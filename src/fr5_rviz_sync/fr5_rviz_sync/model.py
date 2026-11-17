"""Use the official plain URDF, without hardware or controller definitions."""
import xml.etree.ElementTree as ET
from .conversion import JOINT_NAMES


def load_live_description(path):
    root = ET.parse(path).getroot()
    actual = tuple(j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed')
    if actual != JOINT_NAMES or root.find('ros2_control') is not None:
        raise ValueError('Expected the plain official six-axis FR5 V6 URDF')
    for origin in root.findall('./link/collision/origins'):
        origin.tag = 'origin'
    return ET.tostring(root, encoding='unicode')
