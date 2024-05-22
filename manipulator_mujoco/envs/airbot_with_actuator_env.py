import time
import os
import numpy as np
from dm_control import mjcf
import mujoco.viewer
import gymnasium as gym
from gymnasium import spaces
from manipulator_mujoco.arenas import StandardArena
from manipulator_mujoco.robots import AirBotArmWithActuator, AG95, G2F85
from manipulator_mujoco.props import Primitive, Primitive_cylinder, Primitive_box_1, Primitive_test  # 盒子属性
from manipulator_mujoco.mocaps import Target
from manipulator_mujoco.controllers import OperationalSpaceController


class AirBotWithActuatorEnv(gym.Env):

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": None,
    }  # TODO add functionality to render_fps

    def __init__(self, render_mode=None):
        # TODO come up with an observation space that makes sense
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64
        )

        # TODO come up with an action space that makes sense
        self.action_space = spaces.Box(
            low=-0.1, high=0.1, shape=(6,), dtype=np.float64
        )

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self._render_mode = render_mode

        ############################
        # create MJCF model
        ############################

        # checkerboard floor
        self._arena = StandardArena()

        # mocap target that OSC will try to follow
        self._target = Target(self._arena.mjcf_model)

        # airbot arm
        self._arm = AirBotArmWithActuator()

        # ag95 gripper
        # self._gripper = AG95()

        # robotiq_2f85 gripper
        self._gripper = G2F85()

        # attach gripper to arm
        self._arm.attach_tool(self._gripper.mjcf_model, pos=[0, 0, 0], quat=[0, 0, 0, 1])  # 夹爪位置


        self._box_1 = Primitive_box_1(name_body="box_1", pos_body=[0.1, 0.5, 0.55],  # body
                              diaginertia=[0.002, 0.002, 0.002],  # inertia
                              condim="4", type_geom="box", name_geom="box_geom_1", quat_geom=[0, 0, 0, 1],  # geom
                              mass_geom=0.01, size_geom=[0.02, 0.02, 0.02], rgba_geom=[1, 0.5, 0, 1],  # geom
                              friction=[1, 0.005, 0.0001], solimp=[2, 1, 0.01], solref=[0.01, 1])  # geom


        self._cylinder = Primitive_cylinder(
                              # name_site="hook_test", size_site=[0.003], pos_site=[0.05, 0.33, 0.6],  # site
                              name_body="cylinder", pos_body=[-0.1, 0.5, 0.5],  # body
                              axis_joint=[0, 1, 0],  # joint
                              diaginertia=[0.002, 0.002, 0.002],  # inertia
                              condim="4", type_geom="cylinder", name_geom="cylinder_geom", quat_geom=[0, 0, 0, 1],  # geom
                              mass_geom=0.01, size_geom=[0.02, 0.02, 0.02], rgba_geom=[0.3, 0.5, 0.6, 1],  # geom
                              friction=[1, 0.005, 0.0001], solimp=[2, 1, 0.01], solref=[0.01, 1])  # geom


        # gemo type: [plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh, sdf]
        self._box_test = Primitive_test(
                              # name_site="anchor_test", size_site=[0.005], pos_site=[0.0, 0.13, 0.5],  # site
                              name_body="box_test", pos_body=[0.1, 0.5, 0.5],  # body
                              type_joint="hinge",  # joint type: [free, ball, slide, hinge]
                              axis_joint=[0, 0, 1],  # joint
                              diaginertia=[0.002, 0.002, 0.002],  # inertia
                              condim="4", type_geom="box", name_geom="box_geom_test", quat_geom=[0, 0, 0, 1],  # geom
                              mass_geom=0.01, size_geom=[0.02, 0.02, 0.02], rgba_geom=[0.5, 0.5, 0.2, 1],  # geom
                              friction=[1, 0.05, 0.001], solimp=[2, 1, 0.01], solref=[0.01, 1])  # geom


        # attach box to arena as free joint
        self._arena.attach_free(
            self._box_1.mjcf_model
        )

        self._arena.attach(
            self._cylinder.mjcf_model,
        )

        # attach box
        self._arena.attach(
            self._box_test.mjcf_model,
        )


        # attach arm to arena
        self._arena.attach(
            self._arm.mjcf_model, pos=[0, 0, 0], quat=[0, 0, 0, 1]
        )

        # generate model
        self._physics = mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)

        # set up OSC controller
        self._controller = OperationalSpaceController(
            physics=self._physics,
            joints=self._arm.joints,
            eef_site=self._arm.eef_site,
            min_effort=-150.0,
            max_effort=150.0,
            kp=200,
            ko=200,
            kv=50,
            vmax_xyz=1.0,
            vmax_abg=2.0,
        )

        # for GUI and time keeping
        self._timestep = self._physics.model.opt.timestep
        self._viewer = None
        self._step_start = None

    def _get_obs(self) -> np.ndarray:
        # TODO come up with an observations that makes sense for your RL task
        return np.zeros(6)

    def _get_info(self) -> dict:
        # TODO come up with an info dict that makes sense for your RL task
        return {}

    def reset(self, seed=None, options=None) -> tuple:
        super().reset(seed=seed)

        # reset physics
        with self._physics.reset_context():
            # put arm in a reasonable starting position
            self._physics.bind(self._arm.joints).qpos = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ]
            # put target in a reasonable starting position
            self._target.set_mocap_pose(self._physics, position=[-0.365, 0.16, 0.6], quaternion=[0, 1, 0, 1])  # quaternion=[0, 1, 0, 1]

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, action: np.ndarray) -> tuple:
        # TODO use the action to control the arm

        # get mocap target pose
        target_pose = self._target.get_mocap_pose(self._physics)

        # run OSC controller to move to target pose
        self._controller.run(target_pose)

        # step physics
        self._physics.step()

        # render frame
        if self._render_mode == "human":
            self._render_frame()

        # TODO come up with a reward, termination function that makes sense for your RL task
        observation = self._get_obs()
        reward = 0
        terminated = False
        info = self._get_info()

        return observation, reward, terminated, False, info

    def render(self) -> np.ndarray:
        """
        Renders the current frame and returns it as an RGB array if the render mode is set to "rgb_array".

        Returns:
            np.ndarray: RGB array of the current frame.
        """
        if self._render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self) -> None:
        """
        Renders the current frame and updates the viewer if the render mode is set to "human".
        """
        if self._viewer is None and self._render_mode == "human":
            # launch viewer
            self._viewer = mujoco.viewer.launch_passive(
                self._physics.model.ptr,
                self._physics.data.ptr,
            )
        if self._step_start is None and self._render_mode == "human":
            # initialize step timer
            self._step_start = time.time()

        if self._render_mode == "human":
            # render viewer
            self._viewer.sync()

            # TODO come up with a better frame rate keeping strategy
            time_until_next_step = self._timestep - (time.time() - self._step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            self._step_start = time.time()

        else:  # rgb_array
            return self._physics.render()

    def close(self) -> None:
        """
        Closes the viewer if it's open.
        """
        if self._viewer is not None:
            self._viewer.close()