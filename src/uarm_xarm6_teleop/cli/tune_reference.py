"""Interactively pose the simulated xArm6 and report its joint angles."""

from __future__ import annotations

import argparse

import numpy as np

from ..config import load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manually tune the simulated xArm6 reference pose with joint sliders."
    )
    parser.add_argument("--config", help="path to a TOML configuration file")
    parser.add_argument("--scene", help="override the configured ManiSkill scene")
    return parser.parse_args()


def format_reference(degrees: np.ndarray) -> str:
    values = ", ".join(f"{value:.1f}" for value in degrees)
    return f"reference_degrees = [{values}]"


def run() -> None:
    try:
        import gymnasium as gym
        import mani_skill.envs  # noqa: F401
        import sapien
        import torch
        from mani_skill.utils import sapien_utils
    except ImportError as error:
        raise RuntimeError(
            "ManiSkill dependencies are missing. Install with `pip install -e '.[sim]'`."
        ) from error

    args = parse_args()
    config = load_config(args.config)
    scene = args.scene or config.simulation.scene
    env = gym.make(
        scene,
        robot_uids="xarm6_robotiq",
        render_mode="human",
        control_mode="pd_joint_pos",
        sensor_configs=dict(shader_pack="rt-fast"),
        human_render_camera_configs=dict(shader_pack="rt-fast"),
        viewer_camera_configs=dict(shader_pack="rt-fast"),
    )

    final_degrees: np.ndarray | None = None
    try:
        env.reset(seed=0)
        base_env = env.unwrapped
        robot = base_env.agent.robot
        qpos = robot.qpos.clone()
        reference = torch.as_tensor(
            np.deg2rad(config.xarm6.reference_degrees),
            dtype=qpos.dtype,
            device=qpos.device,
        )
        qpos[..., :6] = reference
        robot.set_qpos(qpos)

        if base_env.gpu_sim_enabled:
            base_env.scene._gpu_apply_all()
            base_env.scene.px.gpu_update_articulation_kinematics()
            base_env.scene._gpu_fetch_all()

        # Render once so SAPIEN builds its viewer plugins before auto-selection.
        viewer = base_env.render_human()
        robot_pose = robot.get_pose()
        camera_pose = sapien_utils.look_at([0.0, -1.5, 1.25], robot_pose.p)
        raw_pose = camera_pose.raw_pose.squeeze().cpu().numpy()
        viewer.set_camera_pose(sapien.Pose(raw_pose[:3], raw_pose[3:]))
        # SAPIEN 3.0.1's expanded joint-details panel passes one-element
        # arrays to scalar widgets and crashes. Keep its useful joint sliders
        # while making the optional +/- expansion a no-op.
        def ignore_joint_details(*_args) -> None:
            return None

        for plugin in viewer.plugins:
            if plugin.__class__.__name__ == "ArticulationWindow":
                plugin.set_joint_details = ignore_joint_details
        viewer.select_entity(robot.links[0]._objs[0].entity)

        print("Drag the first six sliders in the Articulation window (joint1-joint6).")
        print("Ignore the gripper sliders and +/- buttons. Close the viewer when done.")
        while not viewer.closed:
            viewer.render()

        raw_robot = robot._objs[0]
        final_degrees = np.rad2deg(raw_robot.get_qpos()[:6])
    finally:
        env.close()

    if final_degrees is not None:
        print("\nCopy this into the [xarm6] section of your config:")
        print(format_reference(final_degrees))


def main() -> None:
    try:
        run()
    except KeyboardInterrupt:
        print("\nStopped without recording a pose.")
    except (RuntimeError, ValueError, OSError) as error:
        raise SystemExit(f"Error: {error}") from error


if __name__ == "__main__":
    main()
