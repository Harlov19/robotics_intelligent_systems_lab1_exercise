#!/usr/bin/env python3

"""Action server that drives the robot forward for a requested distance.

The robot uses wheel odometry to measure distance and maintains its initial
heading. Movement is stopped if an obstacle is detected.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from lab_interfaces.msg import ObstacleInfo


def yaw_from_quaternion(q):
    """Calculate yaw (rotation around the vertical axis) from a quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def angle_difference(target, current):
    """Return the shortest angular difference, in radians."""
    return math.atan2(
        math.sin(target - current),
        math.cos(target - current)
    )


class DriveDistanceServer(Node):

    def __init__(self):
        super().__init__("drive_distance_server")

        self.declare_parameter("speed", 0.3)
        self.declare_parameter("heading_gain", 1.0)
        self.declare_parameter("odom_timeout", 10.0)

        self.position = None
        self.yaw = None
        self.too_close = False

        self.callback_group = ReentrantCallbackGroup()

        self.cmd_vel = self.create_publisher(
            Twist, "cmd_vel", 10
        )

        self.create_subscription(
            Odometry,
            "odom",
            self.on_odom,
            10,
            callback_group=self.callback_group
        )

        self.create_subscription(
            ObstacleInfo,
            "obstacle",
            self.on_obstacle,
            10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            DriveDistance,
            "drive_distance",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.callback_group
        )

        self.get_logger().info("Drive distance action server started")

    def on_odom(self, msg):
        self.position = msg.pose.pose.position
        self.yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

    def on_obstacle(self, msg):
        self.too_close = msg.too_close

    def on_goal(self, goal):
        if not math.isfinite(goal.distance) or goal.distance <= 0.0:
            self.get_logger().warning(
                f"Rejecting invalid distance goal: {goal.distance}"
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepted drive goal: {goal.distance:.2f} m"
        )
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.get_logger().info("Drive cancellation requested")
        return CancelResponse.ACCEPT

    def stop_robot(self):
        """Publish zero velocity."""
        self.cmd_vel.publish(Twist())

    def execute(self, goal_handle):
        goal = goal_handle.request.distance

        result = DriveDistance.Result()

        # Wait for the first odometry message before recording the start pose.
        self.get_logger().info("Waiting for initial odometry...")

        timeout = self.get_parameter("odom_timeout").value
        start_time = time.monotonic()

        while rclpy.ok() and (
            self.position is None or self.yaw is None
        ):
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                self.get_logger().info(
                    "Drive goal canceled while waiting for odometry"
                )
                return result

            if time.monotonic() - start_time > timeout:
                self.stop_robot()
                goal_handle.abort()
                self.get_logger().error(
                    "Timed out waiting for initial odometry"
                )
                return result

            time.sleep(0.05)

        if not rclpy.ok():
            self.stop_robot()
            goal_handle.abort()
            return result

        # Record the initial position and heading.
        start_x = self.position.x
        start_y = self.position.y
        start_yaw = self.yaw

        self.get_logger().info(
            f"Driving forward {goal:.2f} metres"
        )

        speed = self.get_parameter("speed").value
        heading_gain = self.get_parameter("heading_gain").value

        if not math.isfinite(speed) or speed <= 0.0:
            self.stop_robot()
            goal_handle.abort()
            self.get_logger().error("Invalid drive speed parameter")
            return result

        feedback = DriveDistance.Feedback()
        twist = Twist()

        twist.linear.x = speed

        while rclpy.ok():

            # Handle cancellation.
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                self.get_logger().info(
                    f"Drive canceled after {result.travelled:.2f} m"
                )
                return result

            # Check that odometry is available.
            if self.position is None or self.yaw is None:
                self.stop_robot()
                goal_handle.abort()
                self.get_logger().error(
                    "Odometry became unavailable during movement"
                )
                return result

            # Calculate distance travelled from the initial position.
            travelled = math.hypot(
                self.position.x - start_x,
                self.position.y - start_y
            )

            result.travelled = travelled

            # Stop if an obstacle is detected.
            if self.too_close:
                self.stop_robot()
                goal_handle.abort()
                self.get_logger().warning(
                    f"Obstacle detected; stopped after {travelled:.2f} m"
                )
                return result

            # Stop when the requested distance is reached.
            if travelled >= goal:
                break

            # Correct heading to maintain the initial orientation.
            heading_error = angle_difference(
                start_yaw,
                self.yaw
            )

            twist.angular.z = heading_gain * heading_error

            self.cmd_vel.publish(twist)

            feedback.travelled = travelled
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        # Stop and report success.
        self.stop_robot()

        result.travelled = travelled
        result.reached = True

        goal_handle.succeed()

        self.get_logger().info(
            f"Drive completed successfully. "
            f"Requested: {goal:.2f} m, "
            f"travelled: {travelled:.2f} m"
        )

        return result


def main():
    rclpy.init()

    node = DriveDistanceServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
