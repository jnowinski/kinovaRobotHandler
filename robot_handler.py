#!/usr/bin/env python3

"""
CUSTOM ROS NODE FOR HANDLING ROBOT STATE AND CONTROL
"""

import rospy
import json
import threading
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import String, Empty as EmptyMsg
from kinova_msgs.msg import PoseVelocity
from kinova_msgs.srv import HomeArm, Start, Stop
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
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
        self.geofence_points = []

        # Store velocity for continuous publishing
        self.velocity = None
        self.current_pose = None  # Store latest tool pose

        # Locks for synchronizing access to velocity and pose
        self.velocity_lock = threading.Lock()
        self.pose_lock = threading.Lock()

        #Initialize Home, start, and stop geofence service proxies
        rospy.wait_for_service("/j2n6s300_driver/in/home_arm")
        rospy.wait_for_service("/j2n6s300_driver/in/start_force_control")
        rospy.wait_for_service("/j2n6s300_driver/in/stop_force_control")
        
        self.home_service = rospy.ServiceProxy("/j2n6s300_driver/in/home_arm", HomeArm)
        self.start_geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/start_force_control", Start)
        self.stop_geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/stop_force_control", Stop)

        # Subscribe to the custom node topics
        self.command_sub_home = rospy.Subscriber("send_home", EmptyMsg, self.home_callback)
        self.command_sub_velocity = rospy.Subscriber("update_velocity", PoseVelocity, self.velocity_callback)
        self.command_sub_geofence_start = rospy.Subscriber("start_geofence", EmptyMsg, self.geofence_start_callback)
        self.command_sub_geofence_stop = rospy.Subscriber("stop_geofence", EmptyMsg, self.geofence_stop_callback)
        self.command_sub_add_point = rospy.Subscriber("add_point", EmptyMsg, self.add_point_callback)
        self.command_sub_reset_geofence = rospy.Subscriber("reset_geofence", EmptyMsg, self.reset_geofence_callback)

        # Subscribe to robot tool pose updates
        self.pose_sub = rospy.Subscriber("/j2n6s300_driver/out/tool_pose", PoseStamped, self.tool_pose_callback)

        # Publisher for velocity commands
        self.velocity_pub = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity", PoseVelocity, queue_size=10)
        # Publisher for returning geofence points
        self.point_pub = rospy.Publisher("get_point", Point, queue_size = 3)
        # Rate to publish at: 100 Hz
        self.rate = rospy.Rate(100)  # 100 Hz

        rospy.loginfo("Kinova State Machine Node Initialized")

        # Start the continuous velocity publishing loop
        self.publish_velocity()

    def tool_pose_callback(self, msg):
        """Updates the current pose of the robot arm."""
        with self.pose_lock:
            self.current_pose = {
                "X": msg.pose.position.x,
                "Y": msg.pose.position.y,
                "Z": msg.pose.position.z,
                "ThetaX": msg.pose.orientation.x,
                "ThetaY": msg.pose.orientation.y,
                "ThetaZ": msg.pose.orientation.z,
                "ThetaW": msg.pose.orientation.w
            }

    def add_point_callback(self, msg):
        """Adds a point to the geofencing boundary, publishes this point to the server on the get_point topic"""
        with self.pose_lock:
            if self.current_pose is None:
                rospy.logwarn("No current pose available, ignoring point addition.")
                return
            x, y = self.current_pose["X"], self.current_pose["Y"]

        self.geofence_points.append([x, y])

        # Create the point message to send to server
        point = Point()
        point.x = x
        point.y = y
        point.z = 0

        self.point_pub.publish(point)
        rospy.loginfo(f"Point: {x}, {y} added to geofence boundary")

    def reset_geofence_callback(self, msg):
        self.geofence_points.clear()

    def home_callback(self, msg):
        """Sets the robot mode to home and homes the robot."""
        rospy.loginfo("Received home command")
        self.current_state = RobotState.HOME
        self.home_service()
        rospy.loginfo("Robot moved to home position")
        self.current_state = RobotState.IDLE  # Move back to IDLE after homing

    def velocity_callback(self, msg):
        """Sets the robot mode to velocity and updates the velocity variable."""
        rospy.loginfo("Received velocity command: %s", msg)
        with self.velocity_lock:
            self.velocity = msg
        self.current_state = RobotState.VELOCITY

    def geofence_start_callback(self, msg):
        """Sets the robot mode to geofence."""
        rospy.loginfo("Received geofence start command")
        self.current_state = RobotState.GEOFENCE
        self.start_geofence_service()
        rospy.loginfo("Geofence enabled")

    def geofence_stop_callback(self, msg):
        """Sets the robot mode to idle and stops geofence."""
        rospy.loginfo("Received geofence stop command")
        self.current_state = RobotState.IDLE
        self.stop_geofence_service()
        rospy.loginfo("Geofence disabled, returning to idle mode")

    def publish_velocity(self):
        """Continuously publish velocity updates at 100 Hz if in VELOCITY state."""
        while not rospy.is_shutdown():
            if self.current_state == RobotState.VELOCITY:
                if not self.velocity or not self.current_pose:
                    continue
                with self.current_pose.lock:
                        qx = self.current_pose["ThetaX"]
                        qy = self.current_pose["ThetaY"]
                        qz = self.current_pose["ThetaZ"]
                        qw = self.current_pose["ThetaW"]
                with self.velocity_lock:
                        v_base = np.array([
                            self.velocity.twist_linear_x,
                            self.velocity.twist_linear_y,
                            self.velocity.twist_linear_z
                        ])
                # Convert quaternion to rotation matrix (Optimized)
                R_ee_base = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()  # 3x3 rotation matrix
                #Get velocity in terms of end effector basis
                v_ee = R_ee_base.T @ v_base

                # Construct and publish the transformed velocity message
                vel_msg = PoseVelocity()
                vel_msg.twist_linear_x = v_ee[0]
                vel_msg.twist_linear_y = v_ee[1]
                vel_msg.twist_linear_z = v_ee[2]
                vel_msg.twist_angular_x = self.velocity.twist_angular_x  # Angular velocity remains unchanged
                vel_msg.twist_angular_y = self.velocity.twist_angular_y
                vel_msg.twist_angular_z = self.velocity.twist_angular_z

                self.velocity_pub.publish(vel_msg)


if __name__ == "__main__":
    KinovaStateMachine()
    rospy.spin()
