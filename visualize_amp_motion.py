#!/usr/bin/env python3
"""Replay legacy AMP motion files with Isaac Lab / Isaac Sim.

The input format is the JSON format used by the original Isaac Gym AMP
pipeline. Each frame contains 61 values:

    root position (3), root quaternion xyzw (4), joint position (12),
    local foot position (12), body linear velocity (3),
    body angular velocity (3), joint velocity (12), local foot velocity (12).

No trained policy is required. The root and joint states are written directly
to a single Go2 articulation for dataset inspection.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

from isaaclab.app import AppLauncher


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
DEFAULT_MOTION_GLOB = str(SCRIPT_DIR / "go2_motion" / "*.txt")
DEFAULT_URDF = PROJECT_DIR / "legged_gym" / "resources" / "robots" / "go2" / "urdf" / "go2.urdf"

SOURCE_JOINT_NAMES = (
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
)

ROOT_POS = slice(0, 3)
ROOT_QUAT_XYZW = slice(3, 7)
JOINT_POS = slice(7, 19)
ROOT_LINEAR_VEL_BODY = slice(31, 34)
ROOT_ANGULAR_VEL_BODY = slice(34, 37)
JOINT_VEL = slice(37, 49)
MIN_FRAME_SIZE = 49


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visualize Isaac Gym AMP motions in Isaac Lab/Isaac Sim.")
    parser.add_argument(
        "--motion",
        action="append",
        default=[],
        help="Motion file, directory, or glob. Repeat to supply multiple inputs. Default: datasets/go2_motion/*.txt",
    )
    parser.add_argument("--urdf", type=str, default=str(DEFAULT_URDF), help="Go2 URDF used by Isaac Lab.")
    parser.add_argument(
        "--asset-source",
        choices=("isaaclab", "local-urdf"),
        default="isaaclab",
        help="Use Isaac Lab's official Go2 USD (recommended) or convert the project's local URDF.",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument("--loops", type=int, default=1, help="Number of times to play the selected motion list; 0 loops forever.")
    parser.add_argument("--start-motion", type=int, default=0, help="Index of the first motion to play.")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many total frames; 0 has no limit.")
    parser.add_argument("--root-height-offset", type=float, default=0.0, help="Added to the dataset root z position.")
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Remove each clip's root x/y translation while preserving height and orientation.",
    )
    parser.add_argument(
        "--no-real-time",
        action="store_true",
        help="Do not sleep to match FrameDuration (useful for headless validation).",
    )
    parser.add_argument("--camera-distance", type=float, default=2.2)
    parser.add_argument("--camera-height", type=float, default=0.9)
    AppLauncher.add_app_launcher_args(parser)
    return parser


ARGS = build_parser().parse_args()
if ARGS.speed <= 0.0:
    raise ValueError("--speed must be greater than zero")
if ARGS.loops < 0:
    raise ValueError("--loops must be non-negative")

APP_LAUNCHER = AppLauncher(ARGS)
SIMULATION_APP = APP_LAUNCHER.app

# Isaac Lab and Omniverse imports must happen after AppLauncher creates the app.
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

try:  # Isaac Lab pip layouts do not always expose isaaclab_assets on sys.path.
    from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # noqa: E402
except ModuleNotFoundError:  # pragma: no cover - depends on the installation layout
    import isaaclab  # noqa: E402
    import sys  # noqa: E402

    assets_package_root = Path(isaaclab.__file__).resolve().parents[2] / "isaaclab_assets"
    sys.path.insert(0, str(assets_package_root))
    from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # noqa: E402


@dataclass(frozen=True)
class MotionClip:
    path: Path
    frame_duration: float
    frames: np.ndarray

    @property
    def duration(self) -> float:
        return max(0, self.frames.shape[0] - 1) * self.frame_duration


def resolve_motion_paths(inputs: list[str]) -> list[Path]:
    """Expand files, directories, and globs into a stable de-duplicated list."""
    patterns = inputs or [DEFAULT_MOTION_GLOB]
    paths: list[Path] = []
    seen: set[Path] = set()
    for value in patterns:
        candidate = Path(value).expanduser()
        if candidate.is_dir():
            matches = sorted(candidate.glob("*.txt"))
        elif candidate.is_file():
            matches = [candidate]
        else:
            matches = [Path(path) for path in sorted(glob.glob(value))]
        for match in matches:
            resolved = match.resolve()
            if resolved not in seen:
                seen.add(resolved)
                paths.append(resolved)
    return paths


def load_motion(path: Path) -> MotionClip:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)

    if "Frames" not in payload or "FrameDuration" not in payload:
        raise ValueError(f"{path}: expected FrameDuration and Frames fields")
    frames = np.asarray(payload["Frames"], dtype=np.float32)
    if frames.ndim != 2 or frames.shape[0] == 0 or frames.shape[1] < MIN_FRAME_SIZE:
        raise ValueError(f"{path}: expected a non-empty [N, >= {MIN_FRAME_SIZE}] frame array, got {frames.shape}")
    if not np.isfinite(frames).all():
        raise ValueError(f"{path}: frames contain NaN or Inf")
    frame_duration = float(payload["FrameDuration"])
    if not math.isfinite(frame_duration) or frame_duration <= 0.0:
        raise ValueError(f"{path}: FrameDuration must be finite and positive")

    quaternions = frames[:, ROOT_QUAT_XYZW]
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    if np.any(norms < 1.0e-8):
        raise ValueError(f"{path}: contains a zero-length root quaternion")
    frames[:, ROOT_QUAT_XYZW] = quaternions / norms
    return MotionClip(path=path, frame_duration=frame_duration, frames=frames)


def quat_xyzw_to_wxyz(quaternion: np.ndarray) -> np.ndarray:
    return quaternion[[3, 0, 1, 2]]


def quat_rotate_wxyz(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a 3-vector with a normalized wxyz quaternion."""
    xyz = quaternion[1:4]
    uv = np.cross(xyz, vector)
    uuv = np.cross(xyz, uv)
    return vector + 2.0 * (quaternion[0] * uv + uuv)


def create_robot(urdf_path: Path) -> Articulation:
    if ARGS.asset_source == "isaaclab":
        robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Robot")
        robot_cfg.spawn.rigid_props.disable_gravity = True
        return Articulation(robot_cfg)

    if not urdf_path.is_file():
        raise FileNotFoundError(f"Go2 URDF not found: {urdf_path}")

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=str(urdf_path),
            fix_base=False,
            # Keep fixed visual links. Isaac Sim 5.1 can otherwise generate
            # unresolved visual references while merging the Go2 head links.
            merge_fixed_joints=False,
            replace_cylinders_with_capsules=True,
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.4)),
        actuators={
            "replay": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit_sim=0.0,
                velocity_limit_sim=100.0,
                stiffness=0.0,
                damping=0.0,
            )
        },
    )
    return Articulation(robot_cfg)


def joint_indices_for_dataset(robot: Articulation) -> list[int]:
    name_to_index = {name: index for index, name in enumerate(robot.joint_names)}
    missing = [name for name in SOURCE_JOINT_NAMES if name not in name_to_index]
    if missing:
        raise RuntimeError(
            "The Isaac Lab articulation does not contain the dataset joints: " + ", ".join(missing)
        )
    return [name_to_index[name] for name in SOURCE_JOINT_NAMES]


def write_frame(
    robot: Articulation,
    frame: np.ndarray,
    device: torch.device,
    joint_indices: list[int],
    root_xy_origin: np.ndarray,
) -> np.ndarray:
    root_position = frame[ROOT_POS].copy()
    if ARGS.in_place:
        root_position[:2] -= root_xy_origin
    root_position[2] += ARGS.root_height_offset

    root_quaternion = quat_xyzw_to_wxyz(frame[ROOT_QUAT_XYZW])
    linear_velocity = quat_rotate_wxyz(root_quaternion, frame[ROOT_LINEAR_VEL_BODY])
    angular_velocity = quat_rotate_wxyz(root_quaternion, frame[ROOT_ANGULAR_VEL_BODY])

    root_pose = torch.as_tensor(
        np.concatenate((root_position, root_quaternion)), dtype=torch.float32, device=device
    ).unsqueeze(0)
    root_velocity = torch.as_tensor(
        np.concatenate((linear_velocity, angular_velocity)), dtype=torch.float32, device=device
    ).unsqueeze(0)

    joint_position = robot.data.joint_pos.clone()
    joint_velocity = robot.data.joint_vel.clone()
    joint_position[0, joint_indices] = torch.as_tensor(frame[JOINT_POS], dtype=torch.float32, device=device)
    joint_velocity[0, joint_indices] = torch.as_tensor(frame[JOINT_VEL], dtype=torch.float32, device=device)

    robot.write_root_pose_to_sim(root_pose)
    robot.write_root_velocity_to_sim(root_velocity)
    robot.write_joint_state_to_sim(joint_position, joint_velocity)
    return root_position


def main() -> None:
    motion_paths = resolve_motion_paths(ARGS.motion)
    if not motion_paths:
        raise FileNotFoundError(f"No AMP motion files matched: {ARGS.motion or [DEFAULT_MOTION_GLOB]}")
    clips = [load_motion(path) for path in motion_paths]
    print(f"[INFO] Parsed {len(clips)} AMP motion file(s)", flush=True)
    if ARGS.start_motion >= len(clips):
        raise IndexError(f"--start-motion={ARGS.start_motion} but only {len(clips)} motions were found")
    clips = clips[ARGS.start_motion :]

    # Physics time step follows the fastest selected clip. Slower clips still
    # advance one authored state per frame and use wall-clock pacing for display.
    sim_dt = min(clip.frame_duration for clip in clips) / ARGS.speed
    sim = SimulationContext(sim_utils.SimulationCfg(dt=sim_dt, device=ARGS.device))
    print(f"[INFO] Simulation context created: device={ARGS.device}, dt={sim_dt:.6f}", flush=True)
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)
    robot = create_robot(Path(ARGS.urdf).expanduser().resolve())
    asset_description = (
        "Isaac Lab UNITREE_GO2_CFG"
        if ARGS.asset_source == "isaaclab"
        else str(Path(ARGS.urdf).expanduser().resolve())
    )
    print(f"[INFO] Go2 spawned from: {asset_description}", flush=True)
    sim.reset()
    print("[INFO] Simulation reset complete", flush=True)
    robot.update(sim_dt)

    device = torch.device(ARGS.device)
    joint_indices = joint_indices_for_dataset(robot)
    print(f"[INFO] Isaac Lab joints: {robot.joint_names}", flush=True)
    print(f"[INFO] Dataset joint mapping: {dict(zip(SOURCE_JOINT_NAMES, joint_indices))}", flush=True)
    print(f"[INFO] Loaded {len(clips)} motions from {clips[0].path.parent}", flush=True)

    total_frames = 0
    loop_index = 0
    stop = False
    while SIMULATION_APP.is_running() and not stop and (ARGS.loops == 0 or loop_index < ARGS.loops):
        for clip_index, clip in enumerate(clips):
            if not SIMULATION_APP.is_running():
                stop = True
                break
            print(
                f"[{clip_index + 1}/{len(clips)}] Playing {clip.path.name}: "
                f"{clip.frames.shape[0]} frames, {clip.duration:.2f}s",
                flush=True,
            )
            root_xy_origin = clip.frames[0, :2].copy()
            for frame in clip.frames:
                if not SIMULATION_APP.is_running():
                    stop = True
                    break
                wall_start = time.monotonic()
                root_position = write_frame(robot, frame, device, joint_indices, root_xy_origin)
                robot.write_data_to_sim()
                sim.step()
                robot.update(sim_dt)

                if not ARGS.headless:
                    camera_target = root_position.astype(np.float64)
                    camera_eye = camera_target + np.array(
                        [-ARGS.camera_distance, -0.35 * ARGS.camera_distance, ARGS.camera_height], dtype=np.float64
                    )
                    sim.set_camera_view(camera_eye, camera_target)

                total_frames += 1
                if ARGS.max_frames > 0 and total_frames >= ARGS.max_frames:
                    stop = True
                    break
                if not ARGS.no_real_time:
                    target_period = clip.frame_duration / ARGS.speed
                    remaining = target_period - (time.monotonic() - wall_start)
                    if remaining > 0.0:
                        time.sleep(remaining)
            print(f"[INFO] Finished {clip.path.name}", flush=True)
            if stop:
                break
        loop_index += 1

    print(f"[INFO] Playback complete: {total_frames} frames", flush=True)

if __name__ == "__main__":
    try:
        main()
    finally:
        # This viewer does not create Replicator writers. Isaac Sim 5.1 can
        # wait indefinitely during full stage cleanup in long-lived Docker
        # containers, so use its documented immediate-exit path.
        SIMULATION_APP.close(wait_for_replicator=False, skip_cleanup=True)
