import os
from manipulator_mujoco.robots.arm import Arm

# airbot arm with gripper
_AIRBOTARM_XML = os.path.join(
    os.path.dirname(__file__),
    # '../assets/robots/airbot/airbot_play_v3_0_gripper_otd.xml',
    '../assets/robots/airbot/airbot_arm_manipulation_otd.xml',  # 操作——开门
    # '../assets/robots/airbot/scene.xml',
)

_JOINTS = (
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
    'endleft',
    'endright'
)

_EEF_SITE = 'eef_site'

_ATTACHMENT_SITE = 'attachment_site'

class AirBotArm(Arm):
    def __init__(self, name: str = None):
        super().__init__(_AIRBOTARM_XML, _EEF_SITE, _ATTACHMENT_SITE, _JOINTS, name)