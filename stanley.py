#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rospy
import numpy as np
from math import cos, sin, sqrt, atan2, pi
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from tf.transformations import euler_from_quaternion

class Stanley:
    def __init__(self):
        rospy.init_node('stanley', anonymous=True)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)

        self.is_path = self.is_odom = self.is_status = False
        self.current_position = Point()
        self.target_vel = 30.0  # km/h
        self.vehicle_length = rospy.get_param('~vehicle_length', 4.635)
        self.k = rospy.get_param('~stanley_gain', 0.3)
        self.v_t = rospy.get_param('~velocity_gain', 1.0)

        self.control_loop()

    def cmd_vel_callback(self, msg):
        self.target_vel = msg.linear.x
        rospy.loginfo(f"[Stanley] Received target velocity: {self.target_vel:.2f} km/h")

    def control_loop(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.stanley_control()
            else:
                self.missing_topics()
            self.reset_flags()
            rate.sleep()

    def stanley_control(self):
        vehicle_position = self.current_position
        self.is_look_forward_point = False
        translation_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), vehicle_position.x],
            [sin(self.vehicle_yaw),  cos(self.vehicle_yaw), vehicle_position.y],
            [0, 0, 1]
        ])
        inv_translation_matrix = np.linalg.inv(translation_matrix)

        temp = np.zeros((2, 2))
        dis_min = float('inf')
        j = 0

        # 경로 상에서 현재 차량의 위치를 기준으로 가까운 점을 찾기
        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = inv_translation_matrix.dot(global_path_point)
            if local_path_point[0] < 0:
                continue
            dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
            if dis <= dis_min:
                dis_min = dis
                j = num
                temp[0] = local_path_point[:2]
                temp_global = global_path_point
                self.is_look_forward_point = True
            if num == j + 5:  # Lookahead 포인트를 잘 선택
                temp[1] = local_path_point[:2]

        # Lookahead 포인트가 없으면 fallback 처리
        if np.all(temp[1] == 0):
            temp[1] = temp[0] + [1.0, 0.0]

        if self.is_look_forward_point:
            self.publish_control_commands(temp, temp_global, vehicle_position)
        else:
            self.ctrl_cmd_msg = CtrlCmd()
            self.ctrl_cmd_msg.steering = self.ctrl_cmd_msg.accel = self.ctrl_cmd_msg.brake = 0.0
            rospy.logwarn("Forward point not found")
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def publish_control_commands(self, temp, temp_global, vehicle_position):
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        heading_error = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
        cte = sin(self.vehicle_yaw) * (temp_global[0] - vehicle_position.x) - cos(self.vehicle_yaw) * (temp_global[1] - vehicle_position.y)
        v = max(self.current_vel, 1.0)  # avoid division by zero
        crosstrack_error = -atan2(self.k * cte, v + self.v_t)

        steering_angle = heading_error + crosstrack_error
        self.ctrl_cmd_msg.steering = np.clip(steering_angle, -pi/4, pi/4)

        # Speed control: improve smoother control
        speed_error = self.target_vel - self.current_vel
        output = np.clip(speed_error * 0.05, -1.0, 1.0)

        # Apply aggressive braking if in braking zone
        if self.is_in_braking_zone():
            self.ctrl_cmd_msg.brake = 1.0  # Apply maximum brake in the braking zone
            self.ctrl_cmd_msg.accel = 0.0  # No acceleration

        # Acceleration and braking adjustments
        if not self.is_in_braking_zone():
            self.ctrl_cmd_msg.accel = max(0.0, output)
            self.ctrl_cmd_msg.brake = np.clip(-output, 0.0, 1.0)

        # Logging for control outputs
        rospy.loginfo(f"[Stanley] Steer: {self.ctrl_cmd_msg.steering*180/pi:.2f}°, "
                    f"Accel: {self.ctrl_cmd_msg.accel:.2f}, Brake: {self.ctrl_cmd_msg.brake:.2f}")

        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def is_in_braking_zone(self):
        # Braking zone logic: check if the vehicle is close to a sharp turn or obstacle
        distance_to_goal = sqrt((self.current_position.x - self.path.poses[-1].pose.position.x)**2 +
                                (self.current_position.y - self.path.poses[-1].pose.position.y)**2)
        curvature_ahead = self.estimate_curvature_ahead()  # Calculate curvature ahead
        if distance_to_goal < 15.0 and curvature_ahead > 0.2:  # Near the goal and sharp turn
            return True
        return False

    def estimate_curvature_ahead(self):
        # Estimate the curvature of the path ahead using 3 consecutive waypoints
        path_points = [p.pose.position for p in self.path.poses]
        idx = self.find_closest_point()
        if idx < len(path_points) - 2:
            p1 = path_points[idx]
            p2 = path_points[idx + 1]
            p3 = path_points[idx + 2]

            # Calculate curvature using angle between vectors
            vec1 = np.array([p2.x - p1.x, p2.y - p1.y])
            vec2 = np.array([p3.x - p2.x, p3.y - p2.y])
            dot_product = np.dot(vec1, vec2)
            norm1 = np.linalg.norm(vec1)
            norm2 = np.linalg.norm(vec2)
            cos_theta = dot_product / (norm1 * norm2)
            angle = np.arccos(np.clip(cos_theta, -1, 1))
            distance = np.linalg.norm(np.array([p3.x - p1.x, p3.y - p1.y]))
            if distance > 0:
                return angle / distance
        return 0.0

    def find_closest_point(self):
        path_points = [p.pose.position for p in self.path.poses]
        closest_dist = float('inf')
        closest_idx = 0
        for idx, point in enumerate(path_points):
            dist = np.sqrt((point.x - self.current_position.x)**2 + (point.y - self.current_position.y)**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = idx
        return closest_idx

    def missing_topics(self):
        if not self.is_path:
            rospy.logwarn("Missing '/local_path'")
        if not self.is_odom:
            rospy.logwarn("Missing '/odom'")
        if not self.is_status:
            rospy.logwarn("Missing '/Ego_topic'")

    def reset_flags(self):
        self.is_path = self.is_odom = self.is_status = False

    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position = msg.pose.pose.position

if __name__ == '__main__':
    try:
        stanley = Stanley()
    except rospy.ROSInterruptException:
        pass
