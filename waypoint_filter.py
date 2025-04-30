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
        self.goal = np.array([79.966988, -81.913347, 0.978627])

        # Find closest indices
        self.start_idx = self.find_closest_index(self.start)
        self.m_idx = self.find_closest_index(self.m_point)
        self.n_idx = self.find_closest_index(self.n_point)
        self.goal_idx = self.find_closest_index(self.goal)

        def load_waypoints(self, file_path):
            data = np.loadtxt(file_path)
            return data
        
        def find_closet_index(self,target):
            dists = np.linalg.norm(self.waypoints[:, :3] - target, axis=1)
            return np.argmin(dists)
        
        def pose_callback(self, msg):
            self.pose = msg.pose.pose
            self.publish_filtered_path()

        def get_current_index(self):
            if self.pose is None:
                return None
            pos = np.array([self.pose.position.x, self.pose.position.y, self.pose.postion.z])
            dists =np.linalg.norm(self.waypoints[:,:3] - pos, axis=1)
            return np.argmin(dists)
        
        def publish_filtered_path(self):
            idx = self.get_current_index()
            if idx is None:
                return
        
        if idx < self.m_idx:
            path_segment = self.waypoints[:self.goal_idx -20]
        else:
            path_segment = self.waypoints[self.m_idx + 5:]

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for pt in path_segment:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = pt[2]
            pose.pose.orientation.w =1.0
            path_msg.poses.append(pose)
        self.pub.publish(path_msg)
if __name__ == "__main__":
    rospy.init_node('filter_waypoint_node')
    path_file = rospy.get_param('~path_file', '/home/user/path/C-track_outdoor.txt')
    filter_node = WaypointFilter(path_file)
    rospy.spin()
