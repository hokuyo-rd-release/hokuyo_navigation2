#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomNormalizeNode : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    std::string sub_odom_topic_;
    std::string pub_odom_topic_;

public:
    OdomNormalizeNode() : Node("odom_normalize_node") {
        // Declare and get parameters
        this->declare_parameter<std::string>("sub_odom_topic", "/rsf/rsf_odom_tmp");
        this->declare_parameter<std::string>("pub_odom_topic", "/rsf/rsf_odom");

        this->get_parameter("sub_odom_topic", sub_odom_topic_);
        this->get_parameter("pub_odom_topic", pub_odom_topic_);

        publisher_ = create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic_, 10);
        target_odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            sub_odom_topic_, 10,
            std::bind(&OdomNormalizeNode::target_odom_callback, this, std::placeholders::_1)
        );
    }

    void target_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        nav_msgs::msg::Odometry pub_msg = *msg;
        double qx = pub_msg.pose.pose.orientation.x;
        double qy = pub_msg.pose.pose.orientation.y;
        double qz = pub_msg.pose.pose.orientation.z;
        double qw = pub_msg.pose.pose.orientation.w;
        double q_norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        pub_msg.pose.pose.orientation.x = qx/q_norm;
        pub_msg.pose.pose.orientation.y = qy/q_norm;
        pub_msg.pose.pose.orientation.z = qz/q_norm;
        pub_msg.pose.pose.orientation.w = qw/q_norm;
        publisher_->publish(pub_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomNormalizeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
