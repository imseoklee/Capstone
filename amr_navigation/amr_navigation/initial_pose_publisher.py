
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

from amr_navigation.capstone_logic import (
    NAV2_NAMESPACE_BY_ROBOT,
    ROBOT_STRUCTURE_FILE,
    load_robot_specs,
)


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        self.declare_parameter("robot_structure_file", ROBOT_STRUCTURE_FILE)
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("map_offset_x", 0.0)
        self.declare_parameter("map_offset_y", 0.0)
        self.declare_parameter("default_yaw", 0.0)
        self.declare_parameter("iw_hub_ros_yaw", 0.0)
        self.declare_parameter("iw_hub_ros_01_yaw", 0.0)
        self.declare_parameter("publish_delay_sec", 6.0)
        self.declare_parameter("repeat_count", 20)
        self.declare_parameter("repeat_period_sec", 1.0)
        self.declare_parameter("subscriber_wait_timeout_sec", 10.0)

        self.robot_structure_file = str(self.get_parameter("robot_structure_file").value)
        self.goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self.map_offset_x = float(self.get_parameter("map_offset_x").value)
        self.map_offset_y = float(self.get_parameter("map_offset_y").value)
        self.default_yaw = float(self.get_parameter("default_yaw").value)
        self.yaw_by_namespace = {
            "iw_hub_ros": float(self.get_parameter("iw_hub_ros_yaw").value),
            "iw_hub_ros_01": float(self.get_parameter("iw_hub_ros_01_yaw").value),
        }
        self.repeat_count = int(self.get_parameter("repeat_count").value)
        self.repeat_period_sec = float(self.get_parameter("repeat_period_sec").value)
        self.subscriber_wait_timeout_sec = float(
            self.get_parameter("subscriber_wait_timeout_sec").value
        )

        self.robots = load_robot_specs(self.robot_structure_file)
        self._pose_publishers = {}
        self.remaining_repeats = self.repeat_count
        self._subscriber_wait_elapsed_sec = 0.0

        for robot_name, namespace in NAV2_NAMESPACE_BY_ROBOT.items():
            self._pose_publishers[namespace] = self.create_publisher(
                PoseWithCovarianceStamped, f"/{namespace}/initialpose", 10
            )
            if robot_name in self.robots:
                robot = self.robots[robot_name]
                self.get_logger().info(
                    "Prepared initial pose for %s from %s at (%.3f, %.3f, %.3f)"
                    % (namespace, robot.prim_path, robot.pose.x, robot.pose.y, robot.pose.z)
                )
            else:
                self.get_logger().warning(
                    "Robot %s was not found in %s"
                    % (robot_name, self.robot_structure_file)
                )

        self.publish_delay_sec = float(self.get_parameter("publish_delay_sec").value)
        self._subscriber_wait_timer = self.create_timer(
            0.5, self._wait_for_initialpose_subscribers
        )
        self._start_timer = None
        self._repeat_timer = None
        self.get_logger().info("Initial pose publisher node started.")
        self.get_logger().info(
            "Waiting for /initialpose subscribers before publishing initial poses."
        )

    def _wait_for_initialpose_subscribers(self) -> None:
        ready_namespaces = [
            namespace
            for namespace, publisher in self._pose_publishers.items()
            if publisher.get_subscription_count() > 0
        ]
        required_namespaces = list(self._pose_publishers.keys())
        if len(ready_namespaces) == len(required_namespaces):
            if self._subscriber_wait_timer is not None:
                self._subscriber_wait_timer.cancel()
                self._subscriber_wait_timer = None
            self.get_logger().info(
                "Initialpose subscribers are ready for: %s"
                % ", ".join(sorted(ready_namespaces))
            )
            self._start_timer = self.create_timer(
                self.publish_delay_sec, self._start_repeating_publish
            )
            self.get_logger().info(
                "Publishing initial poses after %.1f seconds."
                % self.publish_delay_sec
            )
            return

        self._subscriber_wait_elapsed_sec += 0.5
        if self._subscriber_wait_elapsed_sec >= self.subscriber_wait_timeout_sec:
            if self._subscriber_wait_timer is not None:
                self._subscriber_wait_timer.cancel()
                self._subscriber_wait_timer = None
            self.get_logger().warning(
                "Timed out waiting for all /initialpose subscribers. "
                "Proceeding with available subscribers: %s"
                % ", ".join(sorted(ready_namespaces))
            )
            self._start_timer = self.create_timer(
                self.publish_delay_sec, self._start_repeating_publish
            )
            return

    def _start_repeating_publish(self) -> None:
        if self._start_timer is not None:
            self._start_timer.cancel()
            self._start_timer = None

        self._publish_once()
        if self.repeat_count > 1:
            self.remaining_repeats = self.repeat_count - 1
            self._repeat_timer = self.create_timer(
                self.repeat_period_sec, self._repeat_publish_callback
            )

    def _repeat_publish_callback(self) -> None:
        if self.remaining_repeats <= 0:
            if self._repeat_timer is not None:
                self._repeat_timer.cancel()
                self._repeat_timer = None
            self.get_logger().info("Finished publishing initial poses.")
            return

        self._publish_once()
        self.remaining_repeats -= 1

    def _publish_once(self) -> None:
        for robot_name, namespace in NAV2_NAMESPACE_BY_ROBOT.items():
            robot = self.robots.get(robot_name)
            if robot is None:
                continue

            yaw = self.yaw_by_namespace.get(namespace, self.default_yaw)
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.goal_frame_id
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.pose.pose.position.x = robot.pose.x + self.map_offset_x
            msg.pose.pose.position.y = robot.pose.y + self.map_offset_y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
            msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.06853891945200942
            self._pose_publishers[namespace].publish(msg)
            self.get_logger().info(
                "Published /%s/initialpose -> (%.3f, %.3f), yaw=%.3f"
                % (
                    namespace,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    yaw,
                )
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
