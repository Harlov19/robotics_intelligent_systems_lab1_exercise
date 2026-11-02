import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")

        # Square side length in metres
        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(
            self,
            DriveDistance,
            "/drive_distance"
        )

        self.rotate_client = ActionClient(
            self,
            Rotate,
            "/rotate"
        )

        self.get_logger().info("Square driver started")

    def send_drive_goal(self, distance):
        goal = DriveDistance.Goal()
        goal.distance = float(distance)

        self.get_logger().info(
            f"Driving forward {distance:.2f} metres"
        )

        self.drive_client.wait_for_server()

        future = self.drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Drive goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()
        result = response.result

        self.get_logger().info(
            f"Travelled: {result.travelled:.3f} m, "
            f"reached: {result.reached}"
        )

        return (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and result.reached
        )

    def send_rotate_goal(self, angle):
        goal = Rotate.Goal()
        goal.angle = float(angle)

        self.get_logger().info(
            f"Rotating {angle:.3f} radians"
        )

        self.rotate_client.wait_for_server()

        future = self.rotate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Rotate goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()
        result = response.result

        self.get_logger().info(
            f"Turned: {result.turned:.3f} radians"
        )

        return response.status == GoalStatus.STATUS_SUCCEEDED

    def drive_square(self):
        side_length = self.get_parameter("side_length").value
        right_angle = 1.5708

        for side in range(4):

            side_number = side + 1

            self.get_logger().info(
                f"Starting square side {side_number}/4"
            )

            # Drive forward
            if not self.send_drive_goal(side_length):
                self.get_logger().error(
                    f"Square stopped: drive goal failed "
                    f"on side {side_number}/4"
                )
                return

            # Rotate 90 degrees to the left
            if not self.send_rotate_goal(right_angle):
                self.get_logger().error(
                    f"Square stopped: rotation goal failed "
                    f"on side {side_number}/4"
                )
                return

        self.get_logger().info(
            "Square completed successfully"
        )


def main(args=None):
    rclpy.init(args=args)

    node = SquareDriver()

    try:
        node.drive_square()

    except KeyboardInterrupt:
        node.get_logger().info("Square driver interrupted")

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
