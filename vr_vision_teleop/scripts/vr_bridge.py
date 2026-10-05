#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class VRBridgePublisher(Node):
    def __init__(self):
        # 初始化 ROS 2 节点，命名为 vr_bridge_node
        super().__init__('vr_bridge_node')
        
        # 创建一个发布者：
        # 数据类型：PoseStamped（包含时间戳、参考坐标系和 xyz位移/四元数旋转 的复合消息）
        # 话题名称：'/vr_target_pose'
        # 队列长度：10（防止网络短暂卡顿时数据堆积）
        self.publisher_ = self.create_publisher(PoseStamped, '/vr_target_pose', 10)
        
        # 设置定时器，模拟 VR 设备的刷新率
        # 1.0 / 60.0 表示 60Hz，这也是很多 VR 头显和手柄的默认追踪频率
        timer_period = 1.0 / 60.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('虚拟 VR 桥接节点已启动：正在向 /vr_target_pose 发布动态目标点')

    def timer_callback(self):
        """定时器回调函数，每秒执行60次"""
        msg = PoseStamped()
        
        # 1. 填充消息头 (Header)
        # 记录当前时间，这对后续算法计算延迟或速度非常重要
        msg.header.stamp = self.get_clock().now().to_msg()
        # 【核心重点】参考坐标系。这里必须填底盘的基坐标系，表示这个目标点是相对于机器人底盘的绝对位置。
        msg.header.frame_id = "base_footprint"  

        # 2. 生成动态的xyz坐标
        # 获取当前系统的绝对时间(秒)，用来作为三角函数的自变量
        t = self.get_clock().now().nanoseconds / 1e9
        
        # 设定机械臂正前方 0.4m 处为圆心，半径为 0.1m 画圆
        # 这样机械臂会在 x 和 y 轴平面上持续画圈，方便我们在 RViz 中直观观察 Servo 的连续跟随效果
        msg.pose.position.x = 0.4 + 0.1 * math.sin(t)
        msg.pose.position.y = 0.0 + 0.1 * math.cos(t)
        msg.pose.position.z = 0.5  # 设定一个固定的高度，防止机械臂砸到自己的底盘

        # 3. 设定目标姿态 (四元数)
        # w=1, x=y=z=0 代表无旋转。具体姿态需要根据你实际夹爪的 URDF 初始朝向来调整。
        # 这里的重点是先测试平移(xyz)伺服，旋转可以暂设默认值。
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        # 发布这条消息
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VRBridgePublisher()
    rclpy.spin(node) # 保持节点运行，直到被手动中断 (Ctrl+C)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()