#!/usr/bin/env python3

import rospy
import threading
import json
from enum import Enum
from std_msgs.msg import String
from kinova_msgs.srv import HomeArm
from kinova_msgs.msg import PoseVelocity


# Enum to represent the different states
class RobotState(Enum):
    IDLE = 1
    VELOCITY = 2
    HOME = 3
    GEOFENCE = 4


class KinovaStateMachine:
    def __init__(self):
        rospy.init_node("kinova_state_machine", anonymous=True)

        # Subscriber to listen for robot commands
        self.command_sub = rospy.Subscriber("send_robot_command", String, self.command_callback)

        # Publisher for velocity commands
        self.velocity_pub = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity", PoseVelocity, queue_size=10)

        # Service client for homing and geofence control
        rospy.wait_for_service("/j2n6s300_driver/in/home_arm")
        rospy.wait_for_service("/j2n6s300_driver/in/start_force_control")
        rospy.wait_for_service("/j2n6s300_driver/in/stop_force_control")
        self.home_service = rospy.ServiceProxy("/j2n6s300_driver/in/home_arm", HomeArm)
        self.start_geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/start_force_control", HomeArm)
        self.stop_geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/stop_force_control", HomeArm)

        # Initialize state
        self.current_state = RobotState.IDLE
        self.velocity_command = PoseVelocity()
        self.lock = threading.Lock()  # Prevent race conditions

        # Start velocity publishing thread
        self.velocity_thread = threading.Thread(target=self.velocity_publisher)
        self.velocity_thread.daemon = True
        self.velocity_thread.start()

        rospy.loginfo("Kinova State Machine Node Ready")
        rospy.spin()

    def command_callback(self, msg):
        """Handles incoming commands and switches states accordingly."""
        try:
            command = json.loads(msg.data)  # Parse JSON input
            cmd_type = command.get("cmd_type", "").lower()

            if cmd_type == "velocity":
                self.enter_velocity_mode(command)
            elif cmd_type == "home":
                self.enter_home_mode()
            elif cmd_type == "geofence_on":
                self.enter_geofence_mode()
            elif cmd_type == "geofence_off":
                self.exit_geofence_mode()
            else:
                rospy.logwarn("Unknown command type received: %s", cmd_type)

        except Exception as e:
            rospy.logerr("Failed to process command: %s", str(e))

    def enter_velocity_mode(self, command):
        """Updates velocity command and switches to velocity mode."""
        with self.lock:
            self.velocity_command.twist_linear_x = command.get("x", 0.0)
            self.velocity_command.twist_linear_y = command.get("y", 0.0)
            self.velocity_command.twist_linear_z = command.get("z", 0.0)
            self.velocity_command.twist_angular_x = command.get("thetax", 0.0)
            self.velocity_command.twist_angular_y = command.get("thetay", 0.0)
            self.velocity_command.twist_angular_z = command.get("thetaz", 0.0)
            self.current_state = RobotState.VELOCITY  # Set state to velocity mode

        rospy.loginfo("Updated velocity command: %s", self.velocity_command)

    def enter_home_mode(self):
        """Handles home state by calling the home service."""
        with self.lock:
            self.current_state = RobotState.HOME  # Set state to home mode

        try:
            rospy.loginfo("Calling home service...")
            self.home_service()  # Call home service
            rospy.loginfo("Robot is homing.")
        except rospy.ServiceException as e:
            rospy.logerr("Home service call failed: %s", str(e))

    def enter_geofence_mode(self):
        """Starts geofence mode by calling the start_force_control service."""
        with self.lock:
            self.current_state = RobotState.GEOFENCE  # Set state to geofence mode

        try:
            rospy.loginfo("Starting geofence mode...")
            self.start_geofence_service()  # Start geofence
            rospy.loginfo("Geofence mode active.")
        except rospy.ServiceException as e:
            rospy.logerr("Geofence service call failed: %s", str(e))

    def exit_geofence_mode(self):
        """Exits geofence mode by calling the stop_force_control service and enters home mode."""
        with self.lock:
            self.current_state = RobotState.HOME  # Set state to home mode

        try:
            rospy.loginfo("Stopping geofence mode...")
            self.stop_geofence_service()  # Stop geofence
            self.home_service()  # Move the robot home
            rospy.loginfo("Exiting geofence mode and homing the robot.")
        except rospy.ServiceException as e:
            rospy.logerr("Geofence service call failed: %s", str(e))

    def velocity_publisher(self):
        """Continuously publishes velocity commands in velocity mode."""
        rate = rospy.Rate(100)  # 100 Hz publishing rate
        while not rospy.is_shutdown():
            with self.lock:
                if self.current_state == RobotState.VELOCITY:
                    self.velocity_pub.publish(self.velocity_command)
            rate.sleep()

if __name__ == "__main__":
    try:
        KinovaStateMachine()
    except rospy.ROSInterruptException:
        pass
