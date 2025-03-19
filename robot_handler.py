import rospy
import json
import threading
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

        # Lock for synchronizing access to velocity
        self.velocity_lock = threading.Lock()

        # Subscribe to the command topics
        self.command_sub_home = rospy.Subscriber("send_home", String, self.home_callback)
        self.command_sub_velocity = rospy.Subscriber("update_velocity", PoseVelocity, self.velocity_callback)
        self.command_sub_geofence_start = rospy.Subscriber("start_geofence", String, self.geofence_start_callback)
        self.command_sub_geofence_stop = rospy.Subscriber("stop_geofence", String, self.geofence_stop_callback)

        # Publisher for velocity commands
        self.velocity_pub = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity", PoseVelocity, queue_size=10)

        # Rate to publish at 100 Hz
        self.rate = rospy.Rate(100)  # 100 Hz

        rospy.loginfo("Kinova State Machine Node Initialized")

        # Start the continuous velocity publishing loop
        self.publish_velocity()

    def home_callback(self, msg):
        """Sets the robot mode to home and homes the robot."""
        rospy.loginfo("Received home command")
        self.current_state = RobotState.HOME
        rospy.wait_for_service("/j2n6s300_driver/in/home_arm")
        home_service = rospy.ServiceProxy("/j2n6s300_driver/in/home_arm", Empty)
        home_service()
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
        rospy.wait_for_service("/j2n6s300_driver/in/start_force_control")
        geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/start_force_control", Empty)
        geofence_service()
        rospy.loginfo("Geofence enabled")

    def geofence_stop_callback(self, msg):
        """Sets the robot mode to idle and stops geofence."""
        rospy.loginfo("Received geofence stop command")
        self.current_state = RobotState.IDLE
        rospy.wait_for_service("/j2n6s300_driver/in/stop_force_control")
        geofence_service = rospy.ServiceProxy("/j2n6s300_driver/in/stop_force_control", E
