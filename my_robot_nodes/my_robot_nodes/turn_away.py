
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed
from std_srvs.srv import SetBool


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")

        # Step 3: Declare movement parameters
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)

        # Step 5: Maximum allowed turning speed
        self.declare_parameter("max_turn_speed", 1.5)

        # Step 5: Node starts disabled
        self.enabled = False

        # Publisher: velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Step 4: Publisher: robot status
        self.status_publisher = self.create_publisher(
            RobotStatus,
            "/robot_status",
            10
        )

        # Subscriber: obstacle information
        self.obstacle_subscription = self.create_subscription(
            ObstacleInfo,
            "/obstacle",
            self.obstacle_callback,
            10
        )

        # Step 5: Enable/disable service
        self.enable_service = self.create_service(
            SetBool,
            "/turn_away_enable",
            self.on_enable
        )

        # Step 5: Change turning speed service
        self.set_turn_speed_service = self.create_service(
            SetTurnSpeed,
            "/set_turn_speed",
            self.on_set_turn_speed
        )

        self.get_logger().info(
            "Turn away node started (currently disabled)"
        )

    def obstacle_callback(self, msg):
        # Create status message
        status = RobotStatus()
        status.obstacle_distance = msg.distance

        # Step 5: Do not publish velocity commands while disabled
        if not self.enabled:
            status.state = "stopped"
            self.status_publisher.publish(status)
            return

        # Create velocity command
        twist = Twist()

        if not msg.too_close:
            # Drive forward
            forward_speed = self.get_parameter(
                "forward_speed"
            ).value

            twist.linear.x = forward_speed
            twist.angular.z = 0.0

            status.state = "forward"

        else:
            # Stop forward movement and turn away
            twist.linear.x = 0.0

            turn_speed = self.get_parameter(
                "turn_speed"
            ).value

            if msg.angle > 0:
                # Obstacle on the left: turn right
                twist.angular.z = -turn_speed
            else:
                # Obstacle on the right or straight ahead: turn left
                twist.angular.z = turn_speed

            status.state = "turning"

        # Publish velocity and status
        self.cmd_vel_publisher.publish(twist)
        self.status_publisher.publish(status)

    def on_enable(self, request, response):
        if request.data:
            # Enable the node
            self.enabled = True

            response.success = True
            response.message = "Turn away enabled"

        else:
            # Publish one zero velocity when switching off
            if self.enabled:
                self.stop()

            self.enabled = False

            response.success = True
            response.message = "Turn away disabled"

        self.get_logger().info(response.message)

        return response

    def on_set_turn_speed(self, request, response):
        # Get maximum permitted turning speed
        max_speed = self.get_parameter(
            "max_turn_speed"
        ).value

        requested_speed = request.turn_speed

        # Validate requested speed
        if not 0.0 < requested_speed <= max_speed:
            response.success = False
            response.message = (
                f"Turn speed must be above 0 and at most "
                f"{max_speed} rad/s"
            )

        else:
            # Store accepted speed as the turn_speed parameter
            self.set_parameters([
                Parameter(
                    "turn_speed",
                    value=float(requested_speed)
                )
            ])

            response.success = True
            response.message = (
                f"Turn speed set to {requested_speed:.2f} rad/s"
            )

        self.get_logger().info(response.message)

        return response

    def stop(self):
        # Publish a zero velocity command
        self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    node = TurnAway()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop the robot if the node is still enabled
        if node.enabled:
            node.stop()

        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

