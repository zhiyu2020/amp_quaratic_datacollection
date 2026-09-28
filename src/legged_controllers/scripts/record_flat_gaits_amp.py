#!/usr/bin/env python3
"""Record flat-ground NMPC/WBC motions in the AMP JSON format.

The output layout is compatible with ``amp_go2/datasets/record_legged_control_amp.py``:
``root_pos(3), root_quat_xyzw(4), joint_pos(12), foot_pos(12),
root_lin_vel(3), root_ang_vel(3), joint_vel(12), foot_vel(12)``.

Unlike the original script, gait definitions are read from the controller's
``gait.info``.  This keeps the recorder in sync with every gait configured for
the selected robot instead of maintaining a second, partial gait table.
"""

import argparse
import json
import math
import os
import re
import time
from typing import Dict, List, Optional, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import mode_schedule
from sensor_msgs.msg import JointState


MODE_NAME_TO_NUMBER = {
    "FLY": 0, "RH": 1, "LH": 2, "LH_RH": 3, "RF": 4, "RF_RH": 5,
    "RF_LH": 6, "RF_LH_RH": 7, "LF": 8, "LF_RH": 9, "LF_LH": 10,
    "LF_LH_RH": 11, "LF_RF": 12, "LF_RF_RH": 13, "LF_RF_LH": 14,
    "STANCE": 15,
}
DEFAULT_JOINTS = "LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE"
DEFAULT_FEET = "LF_FOOT,RF_FOOT,LH_FOOT,RH_FOOT"


def parse_list(value: str) -> List[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def extract_block(text: str, name: str) -> Optional[str]:
    match = re.search(r"(^|\n)\s*" + re.escape(name) + r"\s*\{", text)
    if match is None:
        return None
    start = text.find("{", match.start())
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1:index]
    return None


def indexed_values(block: str) -> List[str]:
    values = []
    for line in block.splitlines():
        line = line.split(";", 1)[0].strip()
        match = re.search(r"\[\s*\d+\s*\]\s+([A-Za-z0-9_.+-]+)", line)
        if match is not None:
            values.append(match.group(1))
    return values


def load_gaits(gait_file: str) -> Dict[str, Tuple[List[float], List[int]]]:
    with open(gait_file, "r", encoding="utf-8") as handle:
        text = handle.read()
    names_block = extract_block(text, "list")
    if names_block is None:
        raise RuntimeError("Could not find gait list in {}".format(gait_file))

    gaits = {}
    for name in indexed_values(names_block):
        gait_block = extract_block(text, name)
        if gait_block is None:
            continue
        modes_block = extract_block(gait_block, "modeSequence")
        times_block = extract_block(gait_block, "switchingTimes")
        if modes_block is None or times_block is None:
            continue
        mode_names = indexed_values(modes_block)
        unknown = [mode for mode in mode_names if mode not in MODE_NAME_TO_NUMBER]
        if unknown:
            raise RuntimeError("Unknown mode(s) {} in gait '{}'".format(unknown, name))
        event_times = [float(value) for value in indexed_values(times_block)]
        if len(event_times) != len(mode_names) + 1:
            raise RuntimeError("Gait '{}' has {} event times for {} modes".format(name, len(event_times), len(mode_names)))
        gaits[name] = (event_times, [MODE_NAME_TO_NUMBER[mode] for mode in mode_names])
    if not gaits:
        raise RuntimeError("No valid gait definitions found in {}".format(gait_file))
    return gaits


def normalize_quat(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / norm, qy / norm, qz / norm, qw / norm) if norm > 1e-12 else (0.0, 0.0, 0.0, 1.0)


def world_to_body(vector: List[float], quaternion: List[float]) -> List[float]:
    """Rotate a world-frame vector by the inverse xyzw base quaternion."""
    qx, qy, qz, qw = normalize_quat(*quaternion)
    # R(q)^T * v, written out to avoid a numpy dependency in a ROS node.
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return [
        r00 * vector[0] + r10 * vector[1] + r20 * vector[2],
        r01 * vector[0] + r11 * vector[1] + r21 * vector[2],
        r02 * vector[0] + r12 * vector[1] + r22 * vector[2],
    ]


class FlatAmpRecorder:
    def __init__(self, args: argparse.Namespace, gaits: Dict[str, Tuple[List[float], List[int]]]):
        self.args = args
        self.gaits = gaits
        self.odom = None
        self.link_states = None
        self.joint_state = None
        self.joint_index = None
        self.frames: List[List[float]] = []
        self.root_pos0 = None
        self.last_foot_pos = None
        self.last_stamp = None
        self.recording = False

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self._odom_callback, queue_size=1)
        self.link_states_sub = rospy.Subscriber(args.gazebo_link_states_topic, LinkStates, self._link_states_callback, queue_size=1)
        self.joint_sub = rospy.Subscriber(args.joint_topic, JointState, self._joint_callback, queue_size=1)
        self.gait_pub = rospy.Publisher(args.gait_topic, mode_schedule, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
        self.sample_timer = rospy.Timer(rospy.Duration(1.0 / args.rate), self._sample)
        self.cmd_timer = rospy.Timer(rospy.Duration(1.0 / args.cmd_rate), self._publish_command)

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def _link_states_callback(self, msg: LinkStates) -> None:
        self.link_states = msg

    def _joint_callback(self, msg: JointState) -> None:
        self.joint_state = msg
        if self.joint_index is None:
            names = {name: index for index, name in enumerate(msg.name)}
            missing = [name for name in self.args.joint_names if name not in names]
            if missing:
                rospy.logwarn_throttle(2.0, "Waiting for joints: %s", ", ".join(missing))
                return
            self.joint_index = [names[name] for name in self.args.joint_names]

    def _publish_command(self, _event) -> None:
        if not self.recording and not self.args.command_during_warmup:
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = self.args.cmd_vel
        self.cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_gait(self, gait_name: str) -> None:
        event_times, modes = self.gaits[gait_name]
        msg = mode_schedule()
        msg.eventTimes = event_times
        msg.modeSequence = modes
        end_time = rospy.get_time() + self.args.gait_publish_duration
        rate = rospy.Rate(1.0 / self.args.gait_publish_period)
        while not rospy.is_shutdown() and rospy.get_time() < end_time:
            self.gait_pub.publish(msg)
            rate.sleep()
        rospy.loginfo("Published gait '%s'", gait_name)

    def wait_until_ready(self, timeout: float = 10.0) -> None:
        """Fail early instead of silently writing an empty motion file."""
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            if self.joint_index is not None and self._gazebo_state() is not None:
                return
            if self.joint_index is not None and self.odom is not None:
                try:
                    self.tf_buffer.lookup_transform(
                        self.args.base_frame, self.args.foot_frames[0], rospy.Time(0), rospy.Duration(0.02))
                    return
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                    pass
            if time.monotonic() >= deadline:
                missing = []
                if self.odom is None:
                    missing.append("no messages on {}".format(self.args.odom_topic))
                if self.joint_index is None:
                    missing.append("the requested joints are absent from {}".format(self.args.joint_topic))
                if self._gazebo_state() is None:
                    missing.append("no complete base/foot links on {} for model '{}'".format(
                        self.args.gazebo_link_states_topic, self.args.gazebo_model))
                if self.odom is not None:
                    missing.append("no TF {} -> {}".format(self.args.base_frame, self.args.foot_frames[0]))
                raise RuntimeError(
                    "Recorder inputs were not ready: {}".format("; ".join(missing)))
            rate.sleep()

    def _gazebo_state(self):
        """Return base and feet from Gazebo link states, or None if unavailable."""
        if self.link_states is None:
            return None
        indices = {name: index for index, name in enumerate(self.link_states.name)}

        def index_for(link: str):
            exact = "{}::{}".format(self.args.gazebo_model, link)
            if exact in indices:
                return indices[exact]
            suffix = "::{}".format(link)
            matches = [index for name, index in indices.items() if name.endswith(suffix)]
            return matches[0] if len(matches) == 1 else None

        base_index = index_for(self.args.base_frame)
        foot_indices = [index_for(frame) for frame in self.args.foot_frames]
        if base_index is None or any(index is None for index in foot_indices):
            return None
        return base_index, foot_indices

    def _sample(self, _event) -> None:
        if (not self.recording or self.joint_state is None or self.joint_index is None
                or (self.odom is None and self._gazebo_state() is None)):
            return
        gazebo_state = self._gazebo_state()
        if gazebo_state is not None:
            base_index, foot_indices = gazebo_state
            pose = self.link_states.pose[base_index]
            base_pos = [pose.position.x, pose.position.y, pose.position.z]
            root_rot = list(normalize_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            foot_positions = []
            for foot_index in foot_indices:
                foot = self.link_states.pose[foot_index].position
                relative = [foot.x - base_pos[0], foot.y - base_pos[1], foot.z - base_pos[2]]
                foot_positions.extend(world_to_body(relative, root_rot))
            velocity = self.link_states.twist[base_index]
            lin_vel = world_to_body([velocity.linear.x, velocity.linear.y, velocity.linear.z], root_rot)
            ang_vel = world_to_body([velocity.angular.x, velocity.angular.y, velocity.angular.z], root_rot)
            stamp = rospy.Time.now()
            root_pos = base_pos
        else:
            foot_positions = []
            for frame in self.args.foot_frames:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.args.base_frame, frame, rospy.Time(0), rospy.Duration(0.02))
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                    return
                translation = transform.transform.translation
                foot_positions.extend([translation.x, translation.y, translation.z])
            stamp = self.odom.header.stamp
            if stamp.to_sec() <= 0.0:
                stamp = rospy.Time.now()
            pose = self.odom.pose.pose
            root_pos = [pose.position.x, pose.position.y, pose.position.z]
            root_rot = list(normalize_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            velocity = self.odom.twist.twist
            lin_vel = [velocity.linear.x, velocity.linear.y, velocity.linear.z]
            ang_vel = [velocity.angular.x, velocity.angular.y, velocity.angular.z]
        if self.args.zero_root or self.args.zero_root_xy:
            if self.root_pos0 is None:
                self.root_pos0 = list(root_pos)
            root_pos[0] -= self.root_pos0[0]
            root_pos[1] -= self.root_pos0[1]
            if self.args.zero_root:
                root_pos[2] -= self.root_pos0[2]
        root_pos[2] += self.args.root_z_offset

        joint_pos = [float(self.joint_state.position[index]) for index in self.joint_index]
        joint_vel = [float(self.joint_state.velocity[index]) if index < len(self.joint_state.velocity) else 0.0
                     for index in self.joint_index]
        foot_vel = [0.0] * 12
        if self.last_foot_pos is not None and self.last_stamp is not None:
            dt = (stamp - self.last_stamp).to_sec()
            if dt > 1e-6:
                foot_vel = [(foot_positions[i] - self.last_foot_pos[i]) / dt for i in range(12)]
        self.last_foot_pos, self.last_stamp = list(foot_positions), stamp

        frame = root_pos + root_rot + joint_pos + foot_positions
        frame += lin_vel + ang_vel + joint_vel + foot_vel
        if len(frame) != 61:
            rospy.logerr("Invalid AMP frame length %d (expected 61)", len(frame))
            return
        self.frames.append(frame)

    def record_gait(self, gait_name: str, output_path: str) -> None:
        self._publish_zero()
        self.publish_gait(gait_name)
        rospy.loginfo("Settling gait '%s' for %.1f s", gait_name, self.args.gait_warmup)
        rospy.sleep(self.args.gait_warmup)
        self.wait_until_ready()
        self.frames, self.root_pos0 = [], None
        self.last_foot_pos, self.last_stamp = None, None
        self.recording = True
        rospy.loginfo("Recording %s for %.2f s to %s", gait_name, self.args.duration, output_path)
        rospy.sleep(self.args.duration)
        self.recording = False
        self._publish_zero()
        self._write(output_path, gait_name)

    def _write(self, output_path: str, gait_name: str) -> None:
        directory = os.path.dirname(os.path.abspath(output_path))
        os.makedirs(directory, exist_ok=True)
        data = {
            "LoopMode": self.args.loop_mode,
            "FrameDuration": 1.0 / self.args.rate,
            "EnableCycleOffsetPosition": self.args.enable_cycle_offset_position,
            "EnableCycleOffsetRotation": self.args.enable_cycle_offset_rotation,
            "MotionWeight": self.args.motion_weight,
            "Source": "legged_mpc_amp flat-ground recorder",
            "Gait": gait_name,
            "Frames": self.frames,
        }
        with open(output_path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
            handle.write("\n")
        rospy.loginfo("Saved %d frames to %s", len(self.frames), output_path)

    def shutdown(self) -> None:
        self.recording = False
        self._publish_zero()
        self.sample_timer.shutdown()
        self.cmd_timer.shutdown()


def default_gait_file() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "go2", "gait.info"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Record configured legged_mpc_amp flat-ground gaits as AMP JSON motions.")
    parser.add_argument("--out", help="Output JSON path for one gait.")
    parser.add_argument("--out-dir", help="Output directory when --gait all is used.")
    parser.add_argument("--gait", default="trot", help="Gait from gait.info, or 'all' to record every configured gait.")
    parser.add_argument("--gait-file", default=default_gait_file())
    parser.add_argument("--list-gaits", action="store_true")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--gait-warmup", type=float, default=3.0)
    parser.add_argument("--gait-publish-duration", type=float, default=1.0)
    parser.add_argument("--gait-publish-period", type=float, default=0.1)
    parser.add_argument("--cmd-vel", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("VX", "VY", "WZ"))
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--cmd-rate", type=float, default=20.0)
    parser.add_argument("--command-during-warmup", action="store_true")
    parser.add_argument("--odom-topic", default="/ground_truth/state")
    parser.add_argument("--gazebo-link-states-topic", default="/gazebo/link_states",
                        help="Gazebo fallback source for root and foot poses.")
    parser.add_argument("--gazebo-model", default="go2",
                        help="Gazebo model name; used with --gazebo-link-states-topic.")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--gait-topic", default="/legged_robot_mpc_mode_schedule")
    parser.add_argument("--base-frame", default="base")
    parser.add_argument("--foot-frames", type=parse_list, default=parse_list(DEFAULT_FEET))
    parser.add_argument("--joint-names", type=parse_list, default=parse_list(DEFAULT_JOINTS))
    parser.add_argument("--zero-root", action="store_true")
    parser.add_argument("--zero-root-xy", action="store_true")
    parser.add_argument("--root-z-offset", type=float, default=0.0)
    parser.add_argument("--loop-mode", default="Wrap")
    parser.add_argument("--motion-weight", type=float, default=1.0)
    parser.add_argument("--enable-cycle-offset-position", action="store_true")
    parser.add_argument("--enable-cycle-offset-rotation", action="store_true")
    args = parser.parse_args()

    if args.rate <= 0.0 or args.cmd_rate <= 0.0 or args.duration <= 0.0:
        parser.error("--rate, --cmd-rate and --duration must be positive")
    if args.gait_publish_duration <= 0.0 or args.gait_publish_period <= 0.0 or args.gait_warmup < 0.0:
        parser.error("gait publish values must be positive and --gait-warmup must not be negative")
    if len(args.joint_names) != 12 or len(args.foot_frames) != 4:
        parser.error("AMP layout requires exactly 12 joint names and 4 foot frames")
    gaits = load_gaits(args.gait_file)
    if args.list_gaits:
        print("\n".join(gaits))
        return
    if args.gait != "all" and args.gait not in gaits:
        parser.error("unknown gait '{}'; available: {}".format(args.gait, ", ".join(gaits)))
    if args.gait == "all" and not args.out_dir:
        parser.error("--gait all requires --out-dir")
    if args.gait != "all" and not args.out:
        parser.error("--out is required when recording one gait")

    rospy.init_node("record_flat_gaits_amp", anonymous=True)
    recorder = FlatAmpRecorder(args, gaits)
    try:
        gait_names = list(gaits) if args.gait == "all" else [args.gait]
        for gait_name in gait_names:
            output_path = os.path.join(args.out_dir, "go2_{}.txt".format(gait_name)) if args.gait == "all" else args.out
            recorder.record_gait(gait_name, output_path)
            if args.gait == "all" and not rospy.is_shutdown():
                rospy.sleep(1.0)
    finally:
        recorder.shutdown()


if __name__ == "__main__":
    main()
