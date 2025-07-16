#!/home/cjh/miniconda3/envs/zhujiang/bin/python

import sys
sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang/lib/python3.10/site-packages")
sys.path.insert(0, "/home/cjh/zhujiang_ws/src")

import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from robot_msgs.srv import unique_move, unique_moveResponse
from unique_move_pkg.scripts.trajectory import CarTrajectory
import numpy as np
import cv2
import threading
import math
from scipy.stats import mode


class UniqueMoveNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('unique_move_node')
        
        # 创建发布者，发布到/cmd_vel话题，消息类型为Twist
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 创建新的发布者，发布到/aruco_marker_info话题，消息类型为String
        self.marker_info_pub = rospy.Publisher('/aruco_marker_info', String, queue_size=10)
        
        # 创建服务，服务类型为MoveRobot，回调函数为handle_move_request
        self.service = rospy.Service('/unique_move_service', unique_move, self.handle_move_request)

        self.car = CarTrajectory(x=0,y=-0.71, theta=0)
        

        # 参数设置
        self.linear_x = 0.1  # 线速度(m/s)
        self.angular_z = 0.2  # 角速度(rad/s)
        self.distance2wall = None
        self.angle2wall = None
        self.camera_dis_offset = 0.05
        self.camera_angle_offset = 0
        self.wall2edge_offset = 0.5

        # 加载相机校准参数
        calibration_file = "/home/cjh/zhujiang_ws/src/unique_move_pkg/scripts/charuco_camera_calibration.yaml"
        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("camera_matrix").mat()
        self.dist_coeffs = fs.getNode("dist_coeff").mat()
        fs.release()

        # 定义 ArUco 字典和检测参数
        self.mark_size = 0.15  # ArUco 码的大小
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters()  # 修复 DetectorParameters_create 问题
        self.cap = None
        self.dist_window = []
        self.angle_window = []
        self.window_size = 30

        rospy.loginfo("Unique Move Service is ready...")

    def data_handling(self, data):
        """剔除离群点，返回剔除后的数据"""
        if not data or len(data) == 0:
            return []
        result = mode(data)
        most_common = result.mode
        # rospy.loginfo(f"Data len: {len(data)} ,Most common value: {most_common}, Count: {result.count}")
        if result.count < (len(data) / 2):
            rospy.logwarn(f"Data len: {len(data)} , Count: {result.count}")
            return []
        return most_common.item()


    def rvec_to_euler(self,rvec):
        """将旋转向量转换为欧拉角"""
        rotation_matrix, _ = cv2.Rodrigues(rvec)  # 将旋转向量转换为旋转矩阵
        sy = math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x_angle = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y_angle = math.atan2(-rotation_matrix[2, 0], sy)
            z_angle = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x_angle = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y_angle = math.atan2(-rotation_matrix[2, 0], sy)
            z_angle = 0

        return math.degrees(x_angle), math.degrees(y_angle), math.degrees(z_angle)

    def detect_to_wall_distance(self):
        """
        ArUco 位姿检测
        """
        # rospy.loginfo("Starting ArUco pose detection...")
        cap = cv2.VideoCapture("/dev/ttyMyVideo1")
        if not cap.isOpened():
            rospy.logerr("Failed to open camera.")
            return None, None

        dist_window = []
        angle_window = []

        while True:
            ret, frame = cap.read()
            if not ret:
                rospy.logerr("Failed to read frame from camera.")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is None or len(ids) == 0:
                cap.release()
                rospy.logwarn("No markers detected.")
                return None, None

            if ids[0][0] == 123:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.mark_size, self.camera_matrix, self.dist_coeffs)
                dect_distance = abs(tvecs[0][0][2])
                x_angle, y_angle, z_angle = self.rvec_to_euler(rvecs[0]) 
                dect_angle = y_angle

                if dect_distance > 3:
                    continue

                dist_window.append(dect_distance)
                angle_window.append(dect_angle)

                if len(dist_window) >= self.window_size:
                    # 剔除离群点
                    filtered_dist = self.data_handling(dist_window)
                    filtered_angle = self.data_handling(angle_window)
                    cap.release()
                    if filtered_dist and filtered_angle:
                        return filtered_dist, filtered_angle
                    rospy.logwarn("No valid distance or angle detected after filtering.")
                    return None, None


    def publish_linear_velocity(self, distance=None):
        """
        发布线速度指令
        :param distance: 前进距离
        """
        duration_time = abs(distance) / self.linear_x
        linear_x = (distance / abs(distance)) * self.linear_x

        vel_msg = Twist()
        vel_msg.linear.x = linear_x

        rate = rospy.Rate(20)  # 设置发布频率为20Hz
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(duration_time):
            self.vel_pub.publish(vel_msg)
            rate.sleep()

        # 停止机器人
        vel_msg.linear.x = 0.0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.5)  # 减速过程

    def publish_angular_velocity(self, angle=None):
        """
        发布角速度指令
        :param angle: 旋转角度
        """
        if abs(angle) > (np.pi / 2):
            angular_z = abs(angle)/angle * self.angular_z
        else:
            angular_z = angle / (np.pi / 2) * self.angular_z  # 调整角速度比例   

        angle = angle / 180.0 * np.pi
        duration_time = abs(angle) / abs(angular_z)

        vel_msg = Twist()
        vel_msg.angular.z = angular_z

        rate = rospy.Rate(20)  # 设置发布频率为20Hz
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(duration_time):
            self.vel_pub.publish(vel_msg)
            rate.sleep()

        # 停止机器人
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.5)  # 减速过程
        
    
    def handle_move_request(self, req):
        """
        服务回调函数，处理移动请求
        """
        response = unique_moveResponse()
        if req.order == "move":
            try:
                last_angle = 0  # 基础旋转角度(度)
                new_angle = 0    # 每次旋转角度增量(度)
                move_distance = self.car.D  # 每次前进距离
                step = 0
                
                while True:
                    distance2wall, angle2wall = self.detect_to_wall_distance()
                    if distance2wall is None or angle2wall is None:
                        continue  # 如果未检测到有效的距离或角度，跳过当前循环

                    # 更新小车的离墙距离和角度
                    self.car.d = distance2wall - self.camera_dis_offset - self.wall2edge_offset
                    self.car.angle2wall = -angle2wall - self.camera_angle_offset

                    if self.car.d <= self.car.minimum_distance:
                        break

                    rospy.loginfo(f"Step {step+1}: Distance to wall: {self.car.d:.2f} m, Angle to wall: {self.car.angle2wall:.2f}°")

                    if self.car.d > self.car.maximum_distance:
                        continue

                    # 调用 solve_theta 方法并获取返回值
                    valid_solutions = self.car.solve_theta()
                    if not valid_solutions:
                        rospy.loginfo("No valid solutions, using default angle of 45 degrees")
                        new_angle = 45
                    else:
                        # 使用第一个有效解作为旋转角度
                        new_angle = valid_solutions[0] * self.car.k * 180 / np.pi

                    # 扭正角度
                    current_angle = self.car.angle2wall

                    # 扭正
                    self.car.rotate(current_angle)
                    self.publish_angular_velocity(current_angle)
                    rospy.loginfo(f"Step {step+1}: Rotated {current_angle:.1f}°, New orientation: {self.car.get_orientation():.1f}°")
                    
                    #需要旋转的角度
                    current_angle = new_angle

                    # 旋转
                    self.car.rotate(current_angle)
                    self.publish_angular_velocity(current_angle)
                    rospy.loginfo(f"Step {step+1}: Rotated {current_angle:.1f}°, New orientation: {self.car.get_orientation():.1f}°")
                    
                    # 前进
                    self.car.move_forward(move_distance)
                    self.publish_linear_velocity(move_distance)
                    rospy.loginfo(f"Step {step+1}: Moved {move_distance} m")

                    # 旋转
                    self.car.rotate(-2 * current_angle)
                    self.publish_angular_velocity(-2 * current_angle)
                    rospy.loginfo(f"Step {step+1}: Rotated {-2 * current_angle:.1f}°, New orientation: {self.car.get_orientation():.1f}°")
                    
                    # 前进
                    self.car.move_forward(-move_distance)
                    self.publish_linear_velocity(-move_distance)
                    rospy.loginfo(f"Step {step+1}: Moved {move_distance} m)")

                    # 回正 
                    self.car.rotate(current_angle)
                    self.publish_angular_velocity(current_angle)
                    rospy.loginfo(f"Step {step+1}: Rotated {current_angle:.1f}°, New orientation: {self.car.get_orientation():.1f}°")
                
                    step += 1

                response.success = True
            except Exception as e:
                rospy.logerr(f"Failed to publish velocity command: {str(e)}")
                response.success = False
                response.message = str(e)
            
            return response

if __name__ == "__main__":
    try:
        node = UniqueMoveNode()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")