#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;

class OdomToTf : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string sub_topic_name_;
    std::string pub_topic_name_;
    std::string frame_id_;
    std::string child_frame_id_;

    bool initial_tf_en_;

public:
    OdomToTf() : Node("odom_to_tf") {
        // Declare and get parameters
        this->declare_parameter<std::string>("sub_topic_name", "odometry/switch_on_map");
        this->declare_parameter<std::string>("pub_topic_name", "dummy");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<bool>("initial_tf_en", true);

        this->get_parameter("sub_topic_name", sub_topic_name_);
        this->get_parameter("pub_topic_name", pub_topic_name_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);
        this->get_parameter("initial_tf_en", initial_tf_en_);


        publisher_ = create_publisher<nav_msgs::msg::Odometry>(pub_topic_name_, 10);
        subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            sub_topic_name_, 10,
            std::bind(&OdomToTf::callback, this, std::placeholders::_1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        if (initial_tf_en_) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = this->now();
            transform_stamped.header.frame_id = frame_id_;
            transform_stamped.child_frame_id = child_frame_id_;
            transform_stamped.transform.translation.x = 0.0;
            transform_stamped.transform.translation.y = 0.0;
            transform_stamped.transform.translation.z = 0.0;
            transform_stamped.transform.rotation.x = 0.0;
            transform_stamped.transform.rotation.y = 0.0;
            transform_stamped.transform.rotation.z = 0.0;
            transform_stamped.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(transform_stamped);
        }
    }

    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto pub_msg = *msg;
        pub_msg.header.frame_id = frame_id_;
        pub_msg.child_frame_id = child_frame_id_;

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = msg->header;
        transform_stamped.header.frame_id = frame_id_;
        transform_stamped.child_frame_id = child_frame_id_;
        transform_stamped.transform.translation.x = pub_msg.pose.pose.position.x;
        transform_stamped.transform.translation.y = pub_msg.pose.pose.position.y;
        transform_stamped.transform.translation.z = pub_msg.pose.pose.position.z;
        transform_stamped.transform.rotation = pub_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
