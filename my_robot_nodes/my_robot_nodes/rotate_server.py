import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import Rotate


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")

        self.declare_parameter("turn_speed", 0.5)

        self.callback_group = ReentrantCallbackGroup()

        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            Rotate,
            "/rotate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.current_yaw = None
        self.last_yaw = None
        self.accumulated_yaw = 0.0
        self.odom_event = threading.Event()

        self.get_logger().info("Rotate action server started")

    def get_yaw(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    def odom_callback(self, msg):
        yaw = self.get_yaw(msg.pose.pose.orientation)

        if self.last_yaw is not None:
            delta = math.atan2(
                math.sin(yaw - self.last_yaw),
                math.cos(yaw - self.last_yaw)
            )
            self.accumulated_yaw += delta

        self.last_yaw = yaw
        self.current_yaw = yaw
        self.odom_event.set()

    def goal_callback(self, goal_request):
        if abs(goal_request.angle) < 1e-6:
            self.get_logger().warning("Rejecting zero-angle goal")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Rotation cancellation requested")
        return CancelResponse.ACCEPT

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing rotation goal")

        self.accumulated_yaw = 0.0
        self.last_yaw = None
        self.odom_event.clear()

        target_angle = goal_handle.request.angle
        direction = 1.0 if target_angle > 0.0 else -1.0

        turn_speed = self.get_parameter("turn_speed").value

        feedback_msg = Rotate.Feedback()
        result = Rotate.Result()

        twist = Twist()
        twist.angular.z = direction * turn_speed

        self.odom_event.wait(timeout=2.0)

        if self.current_yaw is None:
            self.stop_robot()
            goal_handle.abort()
            result.turned = 0.0
            return result

        self.cmd_vel_publisher.publish(twist)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.turned = float(self.accumulated_yaw)
                return result

            turned = self.accumulated_yaw

            feedback_msg.turned = float(turned)
            goal_handle.publish_feedback(feedback_msg)

            if direction * turned >= abs(target_angle):
                break

            self.odom_event.wait(timeout=0.1)
            self.odom_event.clear()

        self.stop_robot()

        result.turned = float(self.accumulated_yaw)
        goal_handle.succeed()

        self.get_logger().info(
            f"Rotation completed: {result.turned:.3f} radians"
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    node = RotateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
