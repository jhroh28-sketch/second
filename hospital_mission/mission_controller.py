#!/usr/bin/env python3
import math
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String


class MissionController(Node):
    """Simple state machine driven mission controller using Nav2 Simple Commander."""

    IDLE = 'IDLE'
    GO_TO_STORAGE = 'GO_TO_STORAGE'
    WAIT_PICK_DONE = 'WAIT_PICK_DONE'
    GO_TO_PATIENT = 'GO_TO_PATIENT'
    WAIT_DROP_DONE = 'WAIT_DROP_DONE'
    EMERGENCY = 'EMERGENCY'
    DONE = 'DONE'

    def __init__(self) -> None:
        super().__init__('mission_controller')

        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('poses_file', self._default_pose_file())
        self.declare_parameter('emergency_return_state', 'IDLE')

        use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.navigator = BasicNavigator()
        self.navigator.set_parameters([
            Parameter('use_sim_time', value=use_sim_time),
        ])

        self.pose_config = self._load_pose_config(Path(str(self.get_parameter('poses_file').value)))
        self.return_state_after_emergency = str(
            self.get_parameter('emergency_return_state').value
        ).strip().upper()
        if self.return_state_after_emergency not in [self.IDLE, self.DONE]:
            self.get_logger().warn(
                "emergency_return_state must be IDLE or DONE. Falling back to IDLE."
            )
            self.return_state_after_emergency = self.IDLE

        self.yaw_unit = str(self.pose_config.get('yaw_unit', 'rad')).strip().lower()
        if self.yaw_unit not in ('rad', 'deg'):
            raise RuntimeError("Invalid yaw_unit in YAML. Use 'rad' or 'deg'.")

        self.initial_pose = self._build_pose_stamped(self.pose_config['initial_pose'])
        self.storage_pose = self._build_pose_stamped(self.pose_config['storage_pose'])
        self.patient_pose = self._build_pose_stamped(self.pose_config['patient_pose'])
        self.emergency_pose = self._build_pose_stamped(self.pose_config['emergency_pose'])
        through_pose_dicts = self.pose_config.get('through_poses', [])
        self.through_poses = [self._build_pose_stamped(pose) for pose in through_pose_dicts]

        self.state = self.IDLE
        self.nav_in_progress = False
        self.last_feedback_log_sec = None

        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.mission_sub = self.create_subscription(String, '/mission_cmd', self._on_mission_cmd, 10)
        self.emergency_sub = self.create_subscription(Bool, '/emergency', self._on_emergency, 10)

        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.main_timer = self.create_timer(0.2, self._spin_state_machine)

        self.get_logger().info('Setting initial pose and waiting for Nav2 to become active...')
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active. Waiting for mission start command.')

    def _default_pose_file(self) -> str:
        try:
            share_dir = Path(get_package_share_directory('hospital_mission'))
            return str(share_dir / 'config' / 'poses.yaml')
        except PackageNotFoundError:
            return str((Path(__file__).resolve().parents[1] / 'config' / 'poses.yaml'))

    def _load_pose_config(self, path: Path) -> Dict:
        if not path.exists():
            raise RuntimeError(
                f"Pose YAML not found: {path}. Set parameter 'poses_file' to a valid YAML path."
            )

        try:
            with path.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise RuntimeError(f'Failed to parse YAML file {path}: {exc}') from exc

        if not isinstance(data, dict):
            raise RuntimeError('Pose YAML must be a dictionary at the top-level.')

        required_keys = ['initial_pose', 'storage_pose', 'patient_pose', 'emergency_pose']
        missing = [key for key in required_keys if key not in data]
        if missing:
            raise RuntimeError(f"Pose YAML missing required keys: {', '.join(missing)}")

        for pose_key in required_keys:
            for field in ['x', 'y', 'yaw']:
                if field not in data[pose_key]:
                    raise RuntimeError(f"'{pose_key}' is missing '{field}'")

        if 'through_poses' in data and not isinstance(data['through_poses'], list):
            raise RuntimeError("'through_poses' must be a list of pose dictionaries.")

        return data

    def _yaw_to_rad(self, yaw_value: float) -> float:
        return math.radians(yaw_value) if self.yaw_unit == 'deg' else yaw_value

    def _build_pose_stamped(self, pose_dict: Dict) -> PoseStamped:
        yaw = self._yaw_to_rad(float(pose_dict['yaw']))
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_dict['x'])
        pose.pose.position.y = float(pose_dict['y'])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _on_mission_cmd(self, msg: String) -> None:
        cmd = msg.data.strip().lower()

        if cmd == 'start':
            if self.state in [self.IDLE, self.DONE]:
                self.get_logger().info('Received start command. Starting mission.')
                self._start_go_to_storage()
            else:
                self.get_logger().info(f'Ignoring start command while in state {self.state}.')
        elif cmd == 'pick_done':
            if self.state == self.WAIT_PICK_DONE:
                self.get_logger().info('Received pick_done. Heading to patient.')
                self._start_go_to_patient()
            else:
                self.get_logger().info(f'Ignoring pick_done while in state {self.state}.')
        elif cmd == 'drop_done':
            if self.state == self.WAIT_DROP_DONE:
                self.get_logger().info('Received drop_done. Mission complete.')
                self.state = self.DONE
            else:
                self.get_logger().info(f'Ignoring drop_done while in state {self.state}.')
        elif cmd == 'reset':
            self.get_logger().warn('Received reset. Returning to IDLE safely.')
            self._cancel_navigation_if_needed()
            self.state = self.IDLE
        else:
            self.get_logger().warn(f'Unknown mission command: {cmd}')

    def _on_emergency(self, msg: Bool) -> None:
        if not msg.data:
            return

        self.get_logger().error('Emergency signal received! Cancelling current task and moving to emergency pose.')
        self._cancel_navigation_if_needed()
        self.state = self.EMERGENCY
        self._start_navigation([self.emergency_pose], use_through=False)

    def _start_go_to_storage(self) -> None:
        self.state = self.GO_TO_STORAGE
        targets = self.through_poses + [self.storage_pose]
        self._start_navigation(targets, use_through=len(targets) > 1)

    def _start_go_to_patient(self) -> None:
        self.state = self.GO_TO_PATIENT
        self._start_navigation([self.patient_pose], use_through=False)

    def _start_navigation(self, poses: List[PoseStamped], use_through: bool) -> None:
        for pose in poses:
            pose.header.stamp = self.navigator.get_clock().now().to_msg()

        if use_through:
            self.navigator.goThroughPoses(poses)
        else:
            self.navigator.goToPose(poses[0])

        self.nav_in_progress = True
        self.last_feedback_log_sec = None

    def _spin_state_machine(self) -> None:
        if not self.nav_in_progress:
            return

        feedback = self.navigator.getFeedback()
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if feedback is not None:
            if self.last_feedback_log_sec is None or (now_sec - self.last_feedback_log_sec) > 2.0:
                self.get_logger().info(
                    f'[{self.state}] distance_remaining: {feedback.distance_remaining:.2f} m'
                )
                self.last_feedback_log_sec = now_sec

        if not self.navigator.isTaskComplete():
            return

        self.nav_in_progress = False
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Navigation succeeded in state {self.state}.')
            if self.state == self.GO_TO_STORAGE:
                self.state = self.WAIT_PICK_DONE
            elif self.state == self.GO_TO_PATIENT:
                self.state = self.WAIT_DROP_DONE
            elif self.state == self.EMERGENCY:
                self.state = self.return_state_after_emergency
                self.get_logger().warn(f'Emergency goal reached. Switching to {self.state}.')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'Navigation canceled in state {self.state}.')
        elif result == TaskResult.FAILED:
            self.get_logger().error(f'Navigation failed in state {self.state}.')
            self.state = self.IDLE
        else:
            self.get_logger().warn(f'Navigation ended with unknown result in state {self.state}.')
            self.state = self.IDLE

    def _cancel_navigation_if_needed(self) -> None:
        if self.nav_in_progress:
            self.navigator.cancelTask()
            self.nav_in_progress = False

    def _publish_status(self) -> None:
        msg = String()
        msg.data = f'state={self.state}; nav_in_progress={self.nav_in_progress}'
        self.status_pub.publish(msg)

    def shutdown(self) -> None:
        self.get_logger().info('Shutting down mission controller safely...')
        self._cancel_navigation_if_needed()
        self.navigator.lifecycleShutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionController()
        rclpy.spin(node)
    except RuntimeError as exc:
        print(f'[mission_controller] Initialization error: {exc}')
    except KeyboardInterrupt:
        print('[mission_controller] KeyboardInterrupt received.')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
