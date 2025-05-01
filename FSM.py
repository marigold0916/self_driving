# fsm_controller.py

import rospy
from enum import Enum
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import numpy as np


class FSMState(Enum):
    INIT = 0
    ACCELERATING = 1
    NORMAL = 2
    SLOWING = 3
    BRAKING = 4
    ERROR = 5
    STOPPED = 6
    RECOVERING = 7
    FINISHED = 8


class FSMController:
    def __init__(self, waypoints):
        self.state = FSMState.INIT
        self.target_velocity = rospy.get_param('~target_velocity', 2.0)
        self.max_velocity = rospy.get_param('~max_velocity', 3.0)
        self.min_velocity = rospy.get_param('~min_velocity', 0.0)
        self.curvature_threshold = rospy.get_param('~curvature_threshold', 0.05)
        self.hard_turn_threshold = rospy.get_param('~hard_turn_threshold', 0.15)

        self.velocity = 0.0
        self.prev_velocity = None
        self.yaw = 0.0
        self.pitch = 0.0
        self.z_actual = 0.0
        self.cmd_vel_input = 0.0
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.waypoints = waypoints
        self.goal = waypoints[-1]  # assume goal is last point

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        self.current_pose = None
 
    def odom_callback(self, msg):
        self.velocity = msg.twist.twist.linear.x
        self.check_for_error_conditions()

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        orientation = self.current_pose.orientation
        self.yaw, self.pitch, _ = self.quaternion_to_euler(orientation)
        self.z_actual = self.current_pose.position.z

    def quaternion_to_euler(self, q):
        import tf.transformations as tf_trans
        quaternion = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf_trans.euler_from_quaternion(quaternion)
        return yaw, pitch, roll

    def find_closest_index(self, position):
        positions = self.waypoints[:, :3]
        dists = np.linalg.norm(positions - position, axis=1)
        return np.argmin(dists)

    def check_for_error_conditions(self):
        if self.state == FSMState.ACCELERATING:
            if self.prev_velocity is not None:
                if self.velocity - self.prev_velocity < 0.01:
                    self.state = FSMState.ERROR
            self.prev_velocity = self.velocity

        if abs(self.pitch) > 0.3 or abs(self.yaw) > 2.0:
            self.state = FSMState.ERROR

        # Z-axis deviation from path
        if self.current_pose is not None:
            idx = self.find_closest_index(np.array([
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z]))
            z_ref = self.waypoints[idx][2]
            if abs(self.z_actual - z_ref) > 0.5:
                self.state = FSMState.ERROR

        # TODO: add control input timeout check here

    def heading_aligned_with_path(self):
        if self.current_pose is None:
            return False
        idx = self.find_closest_index(np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z]))
        if idx >= len(self.waypoints) - 1:
            return True
        path_vec = self.waypoints[idx + 1][:2] - self.waypoints[idx][:2]
        heading_vec = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        dot = np.dot(path_vec, heading_vec)
        angle = math.acos(dot / (np.linalg.norm(path_vec) * np.linalg.norm(heading_vec)))
        return abs(angle) < np.deg2rad(15)

    def update(self, curvature):
        if self.state == FSMState.INIT:
            if self.heading_aligned_with_path():
                rospy.loginfo("FSM: INIT → ACCELERATING")
                self.state = FSMState.ACCELERATING

        elif self.state == FSMState.ERROR:
            rospy.loginfo("FSM: ERROR → STOPPED")
            self.state = FSMState.STOPPED

        elif self.state == FSMState.STOPPED:
            if abs(self.pitch) < 0.1 and abs(self.yaw) < 0.5 and self.velocity == 0.0:
                rospy.loginfo("FSM: STOPPED → RECOVERING")
                self.state = FSMState.RECOVERING

        elif self.state == FSMState.RECOVERING:
            if self.heading_aligned_with_path():
                rospy.loginfo("FSM: RECOVERING → ACCELERATING")
                self.state = FSMState.ACCELERATING

        elif np.linalg.norm(self.waypoints[-1][:2] -
                            np.array([self.current_pose.position.x,
                                      self.current_pose.position.y])) < 2.0:
            rospy.loginfo("FSM: GOAL REACHED → FINISHED")
            self.state = FSMState.FINISHED

        elif curvature > self.hard_turn_threshold:
            self.state = FSMState.BRAKING

        elif curvature > self.curvature_threshold:
            self.state = FSMState.SLOWING

        elif self.state not in [FSMState.SLOWING, FSMState.BRAKING]:
            self.state = FSMState.NORMAL

        twist = Twist()
        if self.state == FSMState.ACCELERATING:
            twist.linear.x = min(self.velocity + 0.1, self.target_velocity)
        elif self.state == FSMState.NORMAL:
            twist.linear.x = self.target_velocity
        elif self.state == FSMState.SLOWING:
            twist.linear.x = max(self.target_velocity * 0.7, self.min_velocity)
        elif self.state == FSMState.BRAKING:
            twist.linear.x = max(self.target_velocity * 0.4, self.min_velocity)
        elif self.state == FSMState.STOPPED:
            twist.linear.x = 0.0
        elif self.state == FSMState.RECOVERING:
            twist.linear.x = -0.5
        elif self.state == FSMState.FINISHED:
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('fsm_controller')
    path_file = rospy.get_param('~path_file', '/home/user/path/C-track_outdoor.txt')
    waypoints = np.loadtxt(path_file)
    fsm = FSMController(waypoints)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        example_curvature = 0.0  # replace with real-time curvature calculation
        fsm.update(example_curvature)
        rate.sleep()
