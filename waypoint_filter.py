import rospy
import numpy as np
from nab_msgs.msg import PoseStamped
from geometry_msgs import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math

class WaypointFilter:
    def __init__(self, path_file)
        self.waypoints = self.load_waypoint(path_file)
        self.pub = rospy.Publisher('/filtered_path', Path, queue_size = 1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.pose = None

        self.start = np.array([84.53736, -84.336145, 0.812])
        self.m_point = np.array([124.680649, 92.371916, 12.483801])
        self.n_point = np.array([162.620560, 53.963106, 10.787445])