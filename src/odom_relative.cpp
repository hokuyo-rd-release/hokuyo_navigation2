#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomRelativeNode : public rclcpp::Node {
private:
    bool is_set_target_odom_;
    nav_msgs::msg::Odometry base_odom_;
    nav_msgs::msg::Odometry target_odom_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    std::string base_odom_topic_;
    std::string target_odom_topic_;
    std::string publish_odom_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    void publish_odom(){
        nav_msgs::msg::Odometry rel = target_odom_;
        rel.header.frame_id = frame_id_;
        rel.child_frame_id = child_frame_id_;
        
        tf2::Quaternion base_q, target_q;
        tf2::fromMsg(base_odom_.pose.pose.orientation, base_q);
        tf2::fromMsg(target_odom_.pose.pose.orientation, target_q);

        tf2::Vector3 base_position(
            base_odom_.pose.pose.position.x,
            base_odom_.pose.pose.position.y,
            base_odom_.pose.pose.position.z
        );
        tf2::Vector3 target_position(
            target_odom_.pose.pose.position.x,
            target_odom_.pose.pose.position.y,
            target_odom_.pose.pose.position.z
        );

        tf2::Vector3 dp = target_position - base_position;
        tf2::Quaternion base_q_inv = base_q.inverse();
        tf2::Vector3 dp_rel = tf2::quatRotate(base_q_inv, dp);
        rel.pose.pose.position.x = dp_rel.x();
        rel.pose.pose.position.y = dp_rel.y();
        rel.pose.pose.position.z = dp_rel.z();

        tf2::Quaternion q_rel = base_q_inv * target_q;
        rel.pose.pose.orientation = tf2::toMsg(q_rel);

        publisher_->publish(rel);
        
    }

public:
    OdomRelativeNode() : Node("odom_relative_node") {

        is_set_target_odom_ = false;

        // Declare and get parameters
        this->declare_parameter<std::string>("base_odom_topic", "/rsf/utm_coord_odom");
        this->declare_parameter<std::string>("target_odom_topic", "/rsf/fix_on_utm");
        this->declare_parameter<std::string>("publish_odom_topic", "/rsf/fix_on_odom");
        this->declare_parameter<std::string>("frame_id", "lio_odom");
        this->declare_parameter<std::string>("child_frame_id", "fix_on_odom");

        this->get_parameter("base_odom_topic", base_odom_topic_);
        this->get_parameter("target_odom_topic", target_odom_topic_);
        this->get_parameter("publish_odom_topic", publish_odom_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);

        publisher_ = create_publisher<nav_msgs::msg::Odometry>(publish_odom_topic_, 10);
        base_odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            base_odom_topic_, 10,
            std::bind(&OdomRelativeNode::base_odom_callback, this, std::placeholders::_1)
        );
        target_odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            target_odom_topic_, 10,
            std::bind(&OdomRelativeNode::target_odom_callback, this, std::placeholders::_1)
        );
    }

    void base_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        base_odom_ = *msg;
        if(is_set_target_odom_){
            publish_odom();
            is_set_target_odom_ = false;
        }
    }
    void target_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        target_odom_ = *msg;
        is_set_target_odom_ = true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomRelativeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
