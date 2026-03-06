#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class VRServoBridge : public rclcpp::Node {
public:
    VRServoBridge() : Node("vr_servo_bridge") {
        // 订阅 VR 目标位姿
        sub_vr_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vr_target_pose", 1, std::bind(&VRServoBridge::vrCallback, this, std::placeholders::_1));

        // 发布给 Servo 的速度指令 (Twist)
        // 注意：Servo 默认订阅 /servo_server/delta_twist_cmds
        pub_servo_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_server/delta_twist_cmds", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "VR Servo Bridge Started. Waiting for VR target...");
    }

private:
    void vrCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 1. 获取当前末端执行器的位置 (查询 TF)
        geometry_msgs::msg::TransformStamped current_ee_tf;
        try {
            // "base_link" 是基座, "fr3_hand_tcp" 是末端
            current_ee_tf = tf_buffer_->lookupTransform("base_link", "fr3_hand_tcp", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // 2. 计算误差 (P控制器)
        // 目标位置 (VR) - 当前位置 (Robot)
        double err_x = msg->pose.position.x - current_ee_tf.transform.translation.x;
        double err_y = msg->pose.position.y - current_ee_tf.transform.translation.y;
        double err_z = msg->pose.position.z - current_ee_tf.transform.translation.z;

        // 3. 构建速度指令
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link"; // 指令是相对于基座的

        // 简单的比例控制 (P Gain)
        double Kp_lin = 2.0; 
        
        twist_msg.twist.linear.x = err_x * Kp_lin;
        twist_msg.twist.linear.y = err_y * Kp_lin;
        twist_msg.twist.linear.z = err_z * Kp_lin;

        // 这里简化了旋转控制，如果需要跟随旋转，需要计算四元数差
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        // 4. 发布
        pub_servo_cmd_->publish(twist_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vr_target_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_servo_cmd_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VRServoBridge>());
    rclcpp::shutdown();
    return 0;
}