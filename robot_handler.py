import rospy
import json
from std_msgs.msg import String
from kinova_msgs.msg import PoseVelocity
from std_srvs.srv import Empty
from enum import Enum

# Enum for states
class RobotState(Enum):
    IDLE = 1
    VELOCITY = 2
    HOME = 3
    GEOFENCE = 4

class KinovaStateMachine:
    def __init__(self):
        rospy.init_node('kinova_state_machine', anonymous=True)

        # Initial state is IDLE
        self.current_state = RobotState.IDLE

        # Store velocity for continuous publishing
        self.velocity = None

        # Subscribe to the command topic
        self.command_sub = rospy.Subscriber("send_robot_command", String, self.command_callback)

        # Publisher for velocity commands
        self.velocity_pub = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity", PoseVelocity, queue_size=10)

        # Rate to publish at 100 Hz
        self.rate = rospy.Rate(100)  # 100 Hz

        rospy.loginfo("Kinova State Machine Node Initialized")

        # Start the continuous velocity publishing loop
        self.publish_velocity()

    def command_callback(self, msg):
        """Handles incoming JSON commands and performs appropriate actions."""
        try:
            command_data = json.loads(msg.data)  # Parse JSON
            cmd_type = command_data.get("cmd_type")

            if cmd_type == "velocity":
                self.current_state = RobotState.VELOCITY
                self.velocity = command_data.get("velocity")  # Update velocity
                rospy.loginfo("Received velocity command: %s", self.velocity)

            elif cmd_type == "home":
                self.current_state = RobotState.HOME
                rospy.wait_for_service("/j2n6s300_driver/in/home_arm")
                home_service = rospy.ServiceProxy("/j2n6s300_driver/in/home_arm", Empty)
                home_service()
                rospy.loginfo("Robot moved to home position")
                self.current_state = RobotState.IDLE  # Move back to IDLE after homing

            elif cmd_type == "geofence_on":
                self.current_state = RobotState.GEOFENCE
                rospy.wait_for_service("/j2n6s300_driver/in/start_force_control")
                geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/start_force_control", Empty)
                geofence_service()
                rospy.loginfo("Geofence enabled")

            elif cmd_type == "geofence_off":
                self.current_state = RobotState.IDLE
                rospy.wait_for_service("/j2n6s300_driver/in/stop_force_control")
                geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/stop_force_control", Empty)
                geofence_service()
                rospy.loginfo("Geofence disabled, returning to home mode")

            else:
                rospy.logwarn("Unknown command type received: %s", cmd_type)

        except json.JSONDecodeError:
            rospy.logwarn("Invalid JSON received: %s", msg.data)

    def publish_velocity(self):
        """Continuously publish velocity updates at 100 Hz if in VELOCITY state."""
        while not rospy.is_shutdown():
            if self.current_state == RobotState.VELOCITY and self.velocity:
                vel_msg = PoseVelocity()
                vel_msg.twist_linear_x = self.velocity.get("twist_linear_x", 0.0)
                vel_msg.twist_linear_y = self.velocity.get("twist_linear_y", 0.0)
                vel_msg.twist_linear_z = self.velocity.get("twist_linear_z", 0.0)
                vel_msg.twist_angular_x = self.velocity.get("twist_angular_x", 0.0)
                vel_msg.twist_angular_y = self.velocity.get("twist_angular_y", 0.0)
                vel_msg.twist_angular_z = self.velocity.get("twist_angular_z", 0.0)

                self.velocity_pub.publish(vel_msg)
                rospy.loginfo("Published velocity: %s", vel_msg)

            self.rate.sleep()  # Maintain 100 Hz publishing rate

if __name__ == "__main__":
    KinovaStateMachine()
    rospy.spin()
