#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>

class VRServoBridge : public rclcpp::Node {
public:
    VRServoBridge() : Node("vr_servo_bridge") {
        // 1. 订阅 VR 发来的目标位姿 (Pose)
        sub_vr_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vr_target_pose", 1, std::bind(&VRServoBridge::vrCallback, this, std::placeholders::_1));

        // 2. 创建发布者，向 MoveIt Servo 发布速度指令 (Twist)
        // 话题名必须与 servo_config.yaml 中的 `cartesian_command_in_topic` 匹配
        // 如果 Servo 节点名为 servo_node，话题通常为 /servo_node/delta_twist_cmds
        pub_servo_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 1);

        // 3. 初始化 TF2 (Transform 坐标变换) 监听器
        // 作用：实时查询机械臂末端在空间中的实际位置
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        

        if (!servo_start_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Servo start_servo 服务不可用！");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        servo_start_client_->async_send_request(request);  // 只发一次


        RCLCPP_INFO(this->get_logger(), "VR 伺服桥接节点已启动。正在等待 VR 目标点...");
    }

private:
    // 当收到新的 VR 目标点时，触发此回调函数
    void vrCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped current_ee_tf;
        try {
            // 第一步：获取机器人的【当前状态】
            // 询问 TF 树："在 base_footprint 坐标系下，fr3_hand_tcp (末端执行器) 现在在哪？"
            current_ee_tf = tf_buffer_->lookupTransform(
                "base_footprint", "fr3_hand_tcp", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            // 如果刚启动时 TF 树还没建立好，打印警告并直接返回，避免程序崩溃
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "无法获取坐标变换: %s", ex.what());
            return;
        }

        // 第二步：计算【位置误差】 (Error = Target - Current)
        // 计算目标点与机械臂当前末端点在 X, Y, Z 三个方向上的距离差
        double err_x = msg->pose.position.x - current_ee_tf.transform.translation.x;
        double err_y = msg->pose.position.y - current_ee_tf.transform.translation.y;
        double err_z = msg->pose.position.z - current_ee_tf.transform.translation.z;

        // 第三步：构建【速度指令】 (Twist)
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        // 【核心重点】下发给 Servo 的速度指令，其参考系也必须是基坐标系
        twist_msg.header.frame_id = "base_footprint"; 

        // 第四步：执行【比例控制 (P-Control)】
        // 速度 = 误差 * 比例系数(Kp)
        // Kp 越大，追踪越快，但太大容易导致机械臂抽搐或震荡。
        // Kp 越小，动作越柔和，但会有明显的跟随延迟。2.5 是一个适合起步调试的安全值。
        double Kp_lin = 2.5; 
        
        twist_msg.twist.linear.x = err_x * Kp_lin;
        twist_msg.twist.linear.y = err_y * Kp_lin;
        twist_msg.twist.linear.z = err_z * Kp_lin;

        // 由于上面我们暂不处理旋转跟随，因此将角速度设为0，保持机械臂当前姿态不变
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        // 第五步：将计算好的速度指令发布给 MoveIt Servo
        pub_servo_cmd_->publish(twist_msg);
    }

    // 类成员变量声明
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vr_target_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_servo_cmd_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VRServoBridge>());
    rclcpp::shutdown();
    return 0;
}