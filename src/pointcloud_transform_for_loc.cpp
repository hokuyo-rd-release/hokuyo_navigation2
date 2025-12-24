#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <string>
#include <memory>
#include <cmath>

class PointCloudTransform : public rclcpp::Node {
public:
    PointCloudTransform() : Node("pointcloud_transform_for_loc"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        is_initialized_transform_ = false;

        // Declare and get parameters
        this->declare_parameter("sub_topic", "hokuyo3d3/hokuyo_cloud2");
        this->declare_parameter("sub_odom_topic", "hokuyo_lio/lidar_odom");
        this->declare_parameter("pub_topic", "world/pcd");

        sub_pcd_topic_ = this->get_parameter("sub_topic").as_string();
        pub_pcd_topic_ = this->get_parameter("pub_topic").as_string();
        sub_odom_topic_ = this->get_parameter("sub_odom_topic").as_string();

        // Subscribers and Publishers
        pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_pcd_topic_, 10,
            std::bind(&PointCloudTransform::pcd_sub_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            sub_odom_topic_, 10,
            std::bind(&PointCloudTransform::odom_sub_callback, this, std::placeholders::_1));

        pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_pcd_topic_, 10);
    }

    ~PointCloudTransform() {
    }

private:
    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        last_transform_.header = msg->header;
        last_transform_.child_frame_id = msg->child_frame_id;
        last_transform_.transform.translation.x = msg->pose.pose.position.x;
        last_transform_.transform.translation.y = msg->pose.pose.position.y;
        last_transform_.transform.translation.z = msg->pose.pose.position.z;
        last_transform_.transform.rotation = msg->pose.pose.orientation;
        is_initialized_transform_ = true;
    }

    void pcd_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        sensor_msgs::msg::PointCloud2 sub_msg = *msg;
        sub_msg.header.frame_id = orig_frame_;

        sensor_msgs::msg::PointCloud2 pub_msg;

        if(is_initialized_transform_){
            try { 
                tf2::doTransform(sub_msg, pub_msg, last_transform_);
                pcd_pub_->publish(pub_msg);
            }
            catch(tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            }
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Could not transform: no odometry topic yet" );
        }

    }

    // Parameters
    std::string sub_pcd_topic_, sub_odom_topic_, pub_pcd_topic_;
    std::string orig_frame_, target_frame_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::TransformStamped last_transform_;
    bool is_initialized_transform_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}