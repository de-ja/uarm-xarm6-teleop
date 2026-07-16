"""Visible ManiSkill xArm6 follower backend."""

from __future__ import annotations

import numpy as np


class ManiSkillXArm6:
    def __init__(self, scene: str):
        try:
            import gymnasium as gym
            import mani_skill.envs  # noqa: F401
            import sapien
            from mani_skill.utils import sapien_utils
        except ImportError as error:
            raise RuntimeError(
                "ManiSkill dependencies are missing. Install with `pip install -e '.[sim]'`."
            ) from error

        self.env = gym.make(
            scene,
            robot_uids="xarm6_robotiq",
            render_mode="human",
            control_mode="pd_joint_pos",
            sensor_configs=dict(shader_pack="rt-fast"),
            human_render_camera_configs=dict(shader_pack="rt-fast"),
            viewer_camera_configs=dict(shader_pack="rt-fast"),
        )
        self.env.reset(seed=0)

        agent = getattr(self.env.unwrapped, "agent", None)
        viewer = getattr(self.env.unwrapped, "viewer", None)
        if agent is not None and viewer is not None:
            robot_pose = agent.robot.get_pose()
            camera_pose = sapien_utils.look_at([0.0, -1.5, 1.25], robot_pose.p)
            raw_pose = camera_pose.raw_pose.squeeze().cpu().numpy()
            viewer.set_camera_pose(sapien.Pose(raw_pose[:3], raw_pose[3:]))

    def step(self, action: np.ndarray) -> None:
        bounded = np.clip(action, self.env.action_space.low, self.env.action_space.high)
        self.env.step(bounded)
        self.env.render()

    def close(self) -> None:
        self.env.close()

    def __enter__(self) -> "ManiSkillXArm6":
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
