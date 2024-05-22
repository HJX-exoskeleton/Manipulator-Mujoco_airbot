import os
from manipulator_mujoco.robots.arm import Arm

# airbot arm with gripper
_VX300S_XML = os.path.join(
    os.path.dirname(__file__),
    # '../assets/robots/vx300s/vx300s.xml',
    '../assets/robots/vx300s/scene.xml',
)

_JOINTS = (
    'waist',
    'shoulder',
    'elbow',
    'forearm_roll',
    'wrist_angle',
    'wrist_rotate',
    'left_finger',
    'right_finger'
)

_EEF_SITE = 'eef_site'

_ATTACHMENT_SITE = 'attachment_site'

class Vx300sArm(Arm):
    def __init__(self, name: str = None):
        super().__init__(_VX300S_XML, _EEF_SITE, _ATTACHMENT_SITE, _JOINTS, name)