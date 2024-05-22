import os
from manipulator_mujoco.robots.arm import Arm

# airbot arm with gripper
_AIRBOTARM_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/airbot_no_actuator/airbot_play_v3_0_otd.xml',
)

_JOINTS = (
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
)

_EEF_SITE = 'eef_site'

_ATTACHMENT_SITE = 'attachment_site'

class AirBotArmNoActuator(Arm):
    def __init__(self, name: str = None):
        super().__init__(_AIRBOTARM_XML, _EEF_SITE, _ATTACHMENT_SITE, _JOINTS, name)