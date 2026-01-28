import rclpy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def main():
    rclpy.init()
    node = rclpy.create_node('add_base_obstacle')
    
    # 创建一个发布者，发布到 MoveIt 的规划场景
    pub = node.create_publisher(CollisionObject, '/planning_scene_world', 10)
    
    # 等待连接
    rclpy.spin_once(node, timeout_sec=1)
    
    # 定义碰撞物体（你的小车底盘）
    collision_object = CollisionObject()
    collision_object.header.frame_id = "fr3_link0" # 相对于机械臂基座
    collision_object.id = "mobile_base_body"
    
    # 定义形状：一个长方体
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    # Ranger Mini 大概尺寸：长0.5m, 宽0.4m, 高0.2m (根据实际调整)
    primitive.dimensions = [0.55, 0.45, 0.25] 
    
    # 定义位置：在机械臂基座正下方
    pose = Pose()
    # 假设机械臂安装在车顶，往下的偏移量 = 车高/2 + 安装高度
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = -0.15 # 负数，表示在下面
    pose.orientation.w = 1.0
    
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD
    
    # 发布
    print("正在添加底盘障碍物...")
    for i in range(5): # 多发几次保证收到
        pub.publish(collision_object)
        rclpy.spin_once(node, timeout_sec=0.1)
        
    print("完成！请查看 RViz。")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
