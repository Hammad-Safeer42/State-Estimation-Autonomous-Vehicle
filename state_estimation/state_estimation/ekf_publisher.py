import rclpy
from rclpy.node import Node
from eufs_msgs.msg import CarState
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Accel
import math
import numpy as np

class EKFPublisher():
    def __init__(self, node):
        #take the main node as an argument
        self.node = node
    
        self.xComp_pub = node.create_publisher(CarState, '/odometry_filtered/car_state', 10)

        self.car_state = CarState()

        self.xComp_list = []

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def publish(self, xComp):
        self.node.get_logger().info('Publishing state...')

        #header
        self.car_state.header.stamp = self.node.get_clock().now().to_msg()
        self.car_state.header.frame_id = "map"
        self.car_state.child_frame_id = "base_footprint"

        #pose position
        self.car_state.pose.pose.position.x = float(xComp[0])
        self.car_state.pose.pose.position.y = float(xComp[1])
        q = self.quaternion_from_euler(0, 0, xComp[2]) #convert orientation from euler back to quaternion

        #pose orientation
        self.car_state.pose.pose.orientation.x = q[0]
        self.car_state.pose.pose.orientation.y = q[1]
        self.car_state.pose.pose.orientation.z = q[2]
        self.car_state.pose.pose.orientation.w = q[3]

        #velocities
        self.car_state.twist.twist.linear.x = float(xComp[3])
        self.car_state.twist.twist.linear.y = float(xComp[4])
        self.car_state.twist.twist.angular.z = float(xComp[5])

        #accelerations
        self.car_state.linear_acceleration.x = float(xComp[6])
        self.car_state.linear_acceleration.y = float(xComp[7])
        
        self.xComp_pub.publish(self.car_state)
      