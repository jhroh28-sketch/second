import math
import os
from typing import Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
import yaml


class MissionController(Node):
    STATE_IDLE = 'IDLE'
    STATE_GO_TO_STORAGE = 'GO_TO_STORAGE'
    STATE_WAIT_PICK_DONE = 'WAIT_PICK_DONE'
    STATE_GO_TO_PATIENT = 'GO_TO_PATIENT'
    STATE_WAIT_DROP_DONE = 'WAIT_DROP_DONE'
    STATE_EMERGENCY = 'EMERGENCY'
    STATE_DONE = 'DONE'

    REQUIRED_POSE_KEYS = ['initial_pose', 'storage_pose', 'patient_pose', 'emergency_pose']

    def __init__(self) -> None:
        super().__init__('mission_controller')

        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('poses_file', '')
        self.declare_parameter('status_publish_period', 1.0)
        self.declare_parameter('emergency_return_state', 'IDLE')

        use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])

        self.navigator = BasicNavigator('mission_navigator')
        self.navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])

        poses_file = str(self.get_parameter('poses_file').value).strip()
        if not poses_file:
            poses_file = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config',
                'poses.yaml',
            )

        self.poses = self._load_poses(poses_file)
        self.yaw_unit = str(self.poses.get('yaw_unit', 'rad')).lower()
        if self.yaw_unit not in ('rad', 'deg'):
            raise ValueError("poses.yaml: yaw_unit must be either 'rad' or 'deg'.")

        self.state = self.STATE_IDLE
        self.last_feedback_log_ns = 0
        self.current_nav_label = ''

        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.mission_sub = self.create_subscription(String, '/mission_cmd', self._on_mission_cmd, 10)
        self.emergency_sub = self.create_subscription(Bool, '/emergency', self._on_emergency, 10)

        timer_period = float(self.get_parameter('status_publish_period').value)
        self.timer = self.create_timer(timer_period, self._on_timer)

        self._set_initial_pose_and_wait_nav2()
        self._publish_status('Mission controller ready. Waiting for start command.')

    def _load_poses(self, yaml_path: str) -> Dict:
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(
                f'poses.yaml not found at: {yaml_path}. Set poses_file parameter or add config/poses.yaml.'
            )

        with open(yaml_path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        if not isinstance(data, dict):
            raise ValueError('poses.yaml must contain a top-level dictionary.')

        for key in self.REQUIRED_POSE_KEYS:
            if key not in data:
                raise ValueError(f'poses.yaml is missing required key: {key}')
            for field in ('x', 'y', 'yaw'):
                if field not in data[key]:
                    raise ValueError(f'poses.yaml key {key} is missing field: {field}')

        through_poses = data.get('through_poses', [])
        if through_poses and not isinstance(through_poses, list):
            raise ValueError('poses.yaml through_poses must be a list of poses.')

        return data

    def _set_initial_pose_and_wait_nav2(self) -> None:
        init_pose = self._to_pose_stamped(self.poses['initial_pose'])
        self.navigator.setInitialPose(init_pose)
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')

    def _yaw_to_rad(self, yaw_value: float) -> float:
        return math.radians(yaw_value) if self.yaw_unit == 'deg' else yaw_value

    def _to_pose_stamped(self, pose_cfg: Dict) -> PoseStamped:
        yaw = self._yaw_to_rad(float(pose_cfg['yaw']))

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_cfg['x'])
        pose.pose.position.y = float(pose_cfg['y'])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_status(self, event: str = '') -> None:
        msg = String()
        msg.data = f'state={self.state}'
        if event:
            msg.data += f' | event={event}'
        self.status_pub.publish(msg)

    def _transition(self, new_state: str, event: str) -> None:
        self.state = new_state
        self.get_logger().info(f'{event} -> {new_state}')
        self._publish_status(event)

    def _start_nav_to_storage(self) -> None:
        through_cfg = self.poses.get('through_poses', [])
        if through_cfg:
            route: List[PoseStamped] = [self._to_pose_stamped(p) for p in through_cfg]
            route.append(self._to_pose_stamped(self.poses['storage_pose']))
            self.navigator.goThroughPoses(route)
            self.current_nav_label = 'through_poses + storage_pose'
        else:
            self.navigator.goToPose(self._to_pose_stamped(self.poses['storage_pose']))
            self.current_nav_label = 'storage_pose'
        self._transition(self.STATE_GO_TO_STORAGE, f'Navigation started to {self.current_nav_label}')

    def _start_nav_to_patient(self) -> None:
        self.navigator.goToPose(self._to_pose_stamped(self.poses['patient_pose']))
        self.current_nav_label = 'patient_pose'
        self._transition(self.STATE_GO_TO_PATIENT, 'Navigation started to patient_pose')

    def _start_emergency_nav(self) -> None:
        self.navigator.goToPose(self._to_pose_stamped(self.poses['emergency_pose']))
        self.current_nav_label = 'emergency_pose'
        self._transition(self.STATE_EMERGENCY, 'Emergency navigation started')

    def _cancel_if_running(self) -> None:
        if not self.navigator.isTaskComplete():
            self.navigator.cancelTask()
            self.get_logger().warn('Active Nav2 task canceled.')

    def _on_mission_cmd(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received /mission_cmd: {cmd}')

        if cmd == 'reset':
            self._cancel_if_running()
            self._transition(self.STATE_IDLE, 'Mission reset')
            return

        if cmd == 'start' and self.state == self.STATE_IDLE:
            self._start_nav_to_storage()
            return

        if cmd == 'pick_done' and self.state == self.STATE_WAIT_PICK_DONE:
            self._start_nav_to_patient()
            return

        if cmd == 'drop_done' and self.state == self.STATE_WAIT_DROP_DONE:
            self._transition(self.STATE_DONE, 'Drop confirmed')
            return

        self.get_logger().warn(f'Ignored command {cmd} in state {self.state}')

    def _on_emergency(self, msg: Bool) -> None:
        if not msg.data:
            return

        self.get_logger().error('Emergency signal received. Canceling current task.')
        self._cancel_if_running()
        self._start_emergency_nav()

    def _handle_nav_feedback(self) -> None:
        feedback = self.navigator.getFeedback()
        if feedback is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_feedback_log_ns < 2_000_000_000:
            return

        self.last_feedback_log_ns = now_ns
        distance_remaining = getattr(feedback, 'distance_remaining', None)
        if distance_remaining is not None:
            self.get_logger().info(
                f'[{self.state}] target={self.current_nav_label}, distance_remaining={distance_remaining:.2f} m'
            )

    def _on_timer(self) -> None:
        self._publish_status()

        if self.state not in (self.STATE_GO_TO_STORAGE, self.STATE_GO_TO_PATIENT, self.STATE_EMERGENCY):
            return

        self._handle_nav_feedback()
        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Navigation succeeded at {self.current_nav_label}.')
            if self.state == self.STATE_GO_TO_STORAGE:
                self._transition(self.STATE_WAIT_PICK_DONE, 'Arrived at storage')
            elif self.state == self.STATE_GO_TO_PATIENT:
                self._transition(self.STATE_WAIT_DROP_DONE, 'Arrived at patient')
            elif self.state == self.STATE_EMERGENCY:
                return_state = str(self.get_parameter('emergency_return_state').value).upper()
                if return_state == self.STATE_DONE:
                    self._transition(self.STATE_DONE, 'Emergency goal reached')
                else:
                    self._transition(self.STATE_IDLE, 'Emergency goal reached')
            return

        if result == TaskResult.CANCELED:
            self._transition(self.STATE_IDLE, 'Navigation canceled')
        else:
            self._transition(self.STATE_IDLE, 'Navigation failed')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionController()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(node.navigator)
        executor.spin()
    except (FileNotFoundError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'[mission_controller] {exc}')
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._cancel_if_running()
            node.destroy_node()
            node.navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
