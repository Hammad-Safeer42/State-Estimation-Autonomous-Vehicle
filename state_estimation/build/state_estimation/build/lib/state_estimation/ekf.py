# Code to perform EKF using GPS, IMU and Encoders or just IMU and Encoders
# It has been taken and modified from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py
# Link for thorough understanding of EKF: https://automaticaddison.com/extended-kalman-filter-ekf-with-python-code-example/ with pre-reqs

import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from eufs_msgs.msg import CarState, ImuValues
from sensor_msgs.msg import NavSatFix, Imu 
from sbg_driver.msg import SbgImuData, SbgGpsPos, SbgEkfQuat, SbgGpsVel
from state_estimation.ekf_publisher import EKFPublisher
from rclpy.qos import ReliabilityPolicy, QoSProfile
import matplotlib.pyplot as plt 
from state_estimation.geonav_conversions import ll2xy


class StateEstimation(Node):
    def __init__(self, DT):
        super().__init__('StateEstimation')
        # Covariance for EKF simulation
        self.Q = np.diag([
            0.1,  # variance of location on x-axis
            0.1,  # variance of location on y-axis
            np.deg2rad(1),  # variance of yaw angle
            0.2  # variance of velocity
        ]) ** 2  # predict state covariance
        self.R = np.diag([1.0, 1.0]) ** 2  # Observation x, y position covariance

        self.u = np.array([None, None])  # control inputs

        self.xEst = np.zeros((4, 1))  # ekf estimated state

        self.xComp = np.zeros((8, 1))  # complete state: x, y, yaw, w, vx, vy, ax, ay

        self.PEst = np.eye(4)  # covariance of state estimate

        self.pos_cartesian = np.array([None, None])  # position in cartesian (x, y) coordinates

        self.euler_orientation = np.array([None, None, None])

        self.INPUT_NOISE = np.diag([0.5, np.deg2rad(1)]) ** 2
        self.GPS_NOISE = np.diag([0.05, 0.05]) ** 2

        self.DT = DT  # time tick [s] to be adjusted according to the frequency of sensors

        publish_interval = DT

        self.publisher_timer = self.create_timer(publish_interval, self.publisher_cb)

        self.car_state = CarState()  # data from topic odometry_integration/car_state
        self.imu = Imu()  # data from topic imu/data
        self.gps = NavSatFix()  # data from topic /gps

        self.car_state_subscriber = self.create_subscription(
            CarState,
            '/odometry_integration/car_state',
            self.car_state_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.gps_subscriber = self.create_subscription(
            SbgGpsPos,
            '/sbg/gps_pos',
            self.gps_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.imu_subscriber = self.create_subscription(
            SbgImuData,
            '/sbg/imu_data', self.imu_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            
                
        self.quat_subscriber = self.create_subscription(
            SbgEkfQuat,
            '/sbg/ekf_quat',
            self.quat_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            
        self.gps_vel_subscriber = self.create_subscription(
            SbgGpsVel,
            '/sbg/gps_vel',
            self.gps_vel_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.publisher = self.create_publisher(ImuValues, "/imu_values",10)
        self.gps_vel_subscriber  # prevent unused variable warning
        self.quat_subscriber # prevent unused variable warning
        self.car_state_subscriber  # prevent unused variable warning
        self.gps_subscriber  # prevent unused variable warning
        self.imu_subscriber  # prevent unused variable warning

    # callback for car_state topic
    def car_state_cb(self, msg):
        self.car_state = msg

        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y

        w = msg.twist.twist.angular.z

        v = math.sqrt(x**2 + y**2)  # input velocity

        self.u = np.array([[v], [w]])

    # callback for gps topic
    def gps_cb(self, msg):
        self.gps = msg
        self.get_logger().info("GPS Data Received:")

    # Print header information
        self.get_logger().info("header:")
        self.get_logger().info("  stamp:")
        self.get_logger().info("    sec: {}".format(msg.header.stamp.sec))
        self.get_logger().info("    nanosec: {}".format(msg.header.stamp.nanosec))
        self.get_logger().info("  frame_id: {}".format(msg.header.frame_id))

    # Print time_stamp
        self.get_logger().info("time_stamp: {}".format(msg.time_stamp))

    # Print status
        self.get_logger().info("status:")
        for field, value in msg.status.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.status, field)))

    # Print gps_tow
        self.get_logger().info("gps_tow: {}".format(msg.gps_tow))

    # Print latitude, longitude, and altitude
        self.get_logger().info("latitude: {}".format(msg.latitude))
        self.get_logger().info("longitude: {}".format(msg.longitude))
        self.get_logger().info("altitude: {}".format(msg.altitude))

    # Print undulation
        self.get_logger().info("undulation: {}".format(msg.undulation))

    # Print position_accuracy
        self.get_logger().info("position_accuracy:")
        for field, value in msg.position_accuracy.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.position_accuracy, field)))

    # Print num_sv_used
        self.get_logger().info("num_sv_used: {}".format(msg.num_sv_used))

    # Print base_station_id and diff_age
        self.get_logger().info("base_station_id: {}".format(msg.base_station_id))
        self.get_logger().info("diff_age: {}".format(msg.diff_age))

        self.get_logger().info("---")

    # Your additional processing logic here


        # callback for imu topic
    def imu_cb(self, msg):
        self.imu = msg
        self.get_logger().info("IMU Data Received:")

    # Print header information
        self.get_logger().info("header:")
        self.get_logger().info("  stamp:")
        self.get_logger().info("    sec: {}".format(msg.header.stamp.sec))
        self.get_logger().info("    nanosec: {}".format(msg.header.stamp.nanosec))
        self.get_logger().info("  frame_id: {}".format(msg.header.frame_id))

    # Print time_stamp
        self.get_logger().info("time_stamp: {}".format(msg.time_stamp))

    # Print imu_status
        self.get_logger().info("imu_status:")
        for field, value in msg.imu_status.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.imu_status, field)))

    # Print accel
        self.get_logger().info("accel:")
        for field, value in msg.accel.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.accel, field)))

    # Print gyro
        self.get_logger().info("gyro:")
        for field, value in msg.gyro.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.gyro, field)))

    # Print temp
        self.get_logger().info("temp: {}".format(msg.temp))

    # Print delta_vel
        self.get_logger().info("delta_vel:")
        for field, value in msg.delta_vel.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.delta_vel, field)))

    # Print delta_angle
        self.get_logger().info("delta_angle:")
        for field, value in msg.delta_angle.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.delta_angle, field)))

        self.get_logger().info("---")

    # Your additional processing logic here
    
    
    def quat_cb(self, msg):
        # Quaternion values from the message
        quaternion_msg = msg.quaternion

        # Display quaternion values
        self.get_logger().info("Quaternion values:")
        self.get_logger().info("  x: {}".format(quaternion_msg.x))
        self.get_logger().info("  y: {}".format(quaternion_msg.y))
        self.get_logger().info("  z: {}".format(quaternion_msg.z))
        self.get_logger().info("  w: {}".format(quaternion_msg.w))

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.euler_from_quaternion(quaternion_msg)

        # Display Euler angles
        self.get_logger().info("Yaw: {}".format(yaw))
        self.get_logger().info("Pitch: {}".format(pitch))
        self.get_logger().info("Roll: {}".format(roll))


    def gps_vel_cb(self, msg):
        self.get_logger().info("GPS Velocity Data Received:")

        # Print header information
        self.get_logger().info("header:")
        self.get_logger().info("  stamp:")
        self.get_logger().info("    sec: {}".format(msg.header.stamp.sec))
        self.get_logger().info("    nanosec: {}".format(msg.header.stamp.nanosec))
        self.get_logger().info("  frame_id: {}".format(msg.header.frame_id))

        # Print time_stamp
        self.get_logger().info("time_stamp: {}".format(msg.time_stamp))

        # Print status
        self.get_logger().info("status:")
        for field, value in msg.status.get_fields_and_field_types().items():
            self.get_logger().info("  {}: {}".format(field, getattr(msg.status, field)))

        # Print gps_tow
        self.get_logger().info("gps_tow: {}".format(msg.gps_tow))

        # Print velocity
        self.get_logger().info("velocity:")
        self.get_logger().info("  x: {}".format(msg.velocity.x))
        self.get_logger().info("  y: {}".format(msg.velocity.y))
        self.get_logger().info("  z: {}".format(msg.velocity.z))

        # Print velocity_accuracy
        self.get_logger().info("velocity_accuracy:")
        self.get_logger().info("  x: {}".format(msg.velocity_accuracy.x))
        self.get_logger().info("  y: {}".format(msg.velocity_accuracy.y))
        self.get_logger().info("  z: {}".format(msg.velocity_accuracy.z))

        # Print course and course_acc
        self.get_logger().info("course: {}".format(msg.course))
        self.get_logger().info("course_acc: {}".format(msg.course_acc))

        self.get_logger().info("---")


    def publisher_cb(self):
        # pass this node as an argument
        ekf_publisher = EKFPublisher(self)

        # run code only when both imu and car_state data have been received
        if not None in self.u and not None in self.euler_orientation:

            yaw_rad_prev = self.xEst[2]  # yaw of the previous state

            vx_prev = self.xEst[3] * math.cos(yaw_rad_prev)  # vx of the previous state
            vy_prev = self.xEst[3] * math.sin(yaw_rad_prev)  # vy of the previous state

            # first step of ekf
            z, u = self.observation()

            # rest of the steps of ekf
            self.ekf_estimation(z, u)

            # xComp represents the complete state: x, y, yaw, w, vx, vy, ax, ay
            self.xComp[0] = self.xEst[0]
            self.xComp[1] = self.xEst[1]
            self.xComp[2] = self.xEst[2]

            yaw_rad_new = self.xEst[2]

            vx_new = float(self.xEst[3]) * math.cos(yaw_rad_new)  # v cos (yaw)
            vy_new = float(self.xEst[3]) * math.sin(yaw_rad_new)  # v sin (yaw)

            ax = (vx_new - vx_prev) / self.DT
            ay = (vy_new - vy_prev) / self.DT

            yaw_rate = (yaw_rad_new - yaw_rad_prev) / self.DT

            self.xComp[3] = vx_new
            self.xComp[4] = vy_new
            self.xComp[5] = yaw_rate
            self.xComp[6] = ax
            self.xComp[7] = ay

            # publish complete state
            ekf_publisher.publish(self.xComp)
             # Now, create and publish the ImuValues message
            imu_values_msg = ImuValues()

            imu_values_msg.accel_x = self.imu.accel.x
            imu_values_msg.accel_y = self.imu.accel.y
            imu_values_msg.accel_z = self.imu.accel.z

            imu_values_msg.gyro_x = self.imu.gyro.x
            imu_values_msg.gyro_y = self.imu.gyro.y
            imu_values_msg.gyro_z = self.imu.gyro.z

            imu_values_msg.gps_latitude = self.gps.latitude
            imu_values_msg.gps_longitude = self.gps.longitude

            imu_values_msg.velocity = math.sqrt(self.u[0] ** 2 + self.u[1] ** 2)

            imu_values_msg.delta_angle_x = self.imu.delta_angle.x
            imu_values_msg.delta_angle_y = self.imu.delta_angle.y
            imu_values_msg.delta_angle_z = self.imu.delta_angle.z

            imu_values_msg.velocity_accuracy_x = self.gps.velocity_accuracy.x
            imu_values_msg.velocity_accuracy_y = self.gps.velocity_accuracy.y
            imu_values_msg.velocity_accuracy_z = self.gps.velocity_accuracy.z

            # Publish the ImuValues message
            self.publisher.publish(imu_values_msg)
    



    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    state_estimation = StateEstimation(0.25)  # set DT

    rclpy.spin(state_estimation)

    state_estimation.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

