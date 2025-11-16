#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class DiffFusion(Node):
    def __init__(self):
        super().__init__('diff_drive_fusion')

        # 파라미터
        self.declare_parameter('wheel_base', 0.32)  # 바퀴 간 거리(m) - 로봇에 맞게 수정
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_left_only', False)  # 디버그용 (한쪽만 있을 때)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.use_left_only = bool(self.get_parameter('use_left_only').value)

        # 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp_ns: Optional[int] = None
        self.v_l = 0.0
        self.v_r = 0.0

        # 퍼블리셔/브로드캐스터
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.br = TransformBroadcaster(self)

        # 구독
        self.sub_left  = self.create_subscription(Odometry, '/wheel/left/odometry',  self.cb_left,  10)
        self.sub_right = self.create_subscription(Odometry, '/wheel/right/odometry', self.cb_right, 10)

        # 주기적 퍼블리시 (50Hz)
        self.timer = self.create_timer(1.0/50.0, self.on_timer)

        self.get_logger().info(f'diff_drive_fusion started: wheel_base={self.wheel_base:.3f} m')

    def cb_left(self, msg: Odometry):
        self.v_l = float(msg.twist.twist.linear.x)
        # stamp 기준을 왼쪽/오른쪽 중 최신으로 사용
        self.last_stamp_ns = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)

    def cb_right(self, msg: Odometry):
        self.v_r = float(msg.twist.twist.linear.x)
        self.last_stamp_ns = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)

    def on_timer(self):
        now = self.get_clock().now().nanoseconds
        if self.last_stamp_ns is None:
            self.last_stamp_ns = now
            return

        dt = (now - self.last_stamp_ns) / 1e9
        if dt <= 0.0 or dt > 0.2:  # 너무 큰 간격(200ms)이면 드랍
            self.last_stamp_ns = now
            return
        self.last_stamp_ns = now

        # 속도 융합 (좌/우 평균 → v, 차이로 ω)
        if self.use_left_only:
            v = self.v_l
            w = 0.0
        else:
            v = 0.5 * (self.v_l + self.v_r)
            w = (self.v_r - self.v_l) / max(self.wheel_base, 1e-6)

        # 상태 적분 (unicycle)
        self.yaw += w * dt
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        self.x += v * cy * dt
        self.y += v * sy * dt

        # 쿼터니언
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # 간단 공분산
        odom.pose.covariance[0]  = 0.05  # x
        odom.pose.covariance[7]  = 0.05  # y
        odom.pose.covariance[35] = 0.1   # yaw
        self.odom_pub.publish(odom)

        # TF(odom -> base_link)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = DiffFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
