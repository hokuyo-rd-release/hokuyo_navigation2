#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <memory>
#include <cmath>

class PcdTfTransform : public rclcpp::Node {
public:
    PcdTfTransform() : Node("pcd_tf_transform"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Declare and get parameters
        this->declare_parameter("save_map", false);
        this->declare_parameter("sub_topic", "hokuyo3d3/hokuyo_cloud2");
        this->declare_parameter("sub_odom_topic", "hokuyo_lio/lidar_odom");
        this->declare_parameter("pub_topic", "world/pcd");
        this->declare_parameter("orig_frame", "Odometry");
        this->declare_parameter("target_frame", "odom");
        this->declare_parameter("map_dir", "/home/ubuntu/ros2_ws/src/fusion_tools/test_tools/PCD");
        this->declare_parameter("map_name", "map.pcd");
        this->declare_parameter("pc_save_distance", 1);

        save_map_flg_ = this->get_parameter("save_map").as_bool();
        sub_pcd_topic_ = this->get_parameter("sub_topic").as_string();
        sub_odom_topic_ = this->get_parameter("sub_odom_topic").as_string();
        pub_pcd_topic_ = this->get_parameter("pub_topic").as_string();
        orig_frame_ = this->get_parameter("orig_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        map_dir_ = this->get_parameter("map_dir").as_string();
        map_name_ = this->get_parameter("map_name").as_string();
        pc_save_distance_ = this->get_parameter("pc_save_distance").as_int();

        last_position_.x = 1e6;
        last_position_.y = 1e6;
        last_position_.z = 1e6;

        // Subscribers and Publishers
        pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_pcd_topic_, 10,
            std::bind(&PcdTfTransform::pcd_sub_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            sub_odom_topic_, 10,
            std::bind(&PcdTfTransform::odom_sub_callback, this, std::placeholders::_1));

        pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_pcd_topic_, 10);
    }

    ~PcdTfTransform() {
        save_Map();
    }

private:
    void save_Map() {
        if (save_map_flg_) {
            std::string file_path = map_dir_ + "/" + map_name_;
            pcl::io::savePCDFileASCII(file_path, pcd_map_);
            RCLCPP_INFO(this->get_logger(), "Saved map to %s", file_path.c_str());
        }
    }

    void pcd_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        double dx = current_position_.x - last_position_.x;
        double dy = current_position_.y - last_position_.y;
        double dz = current_position_.z - last_position_.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (distance >= pc_save_distance_) {
            last_position_ = current_position_;

            auto pc2_pub_msg = *msg;
            pc2_pub_msg.header.frame_id = target_frame_;
            pcd_pub_->publish(pc2_pub_msg);

            if (save_map_flg_) {
                pcl::PointCloud<pcl::PointXYZ> pcd_data;
                pcl::fromROSMsg(pc2_pub_msg, pcd_data);
                pcd_map_ += pcd_data;
            }
        }
    }

    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_ = msg->pose.pose.position;
    }

    // Parameters
    bool save_map_flg_;
    int pc_save_distance_;
    std::string sub_pcd_topic_, sub_odom_topic_, pub_pcd_topic_;
    std::string orig_frame_, target_frame_;
    std::string map_dir_, map_name_;

    geometry_msgs::msg::Point last_position_, current_position_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Point cloud map
    pcl::PointCloud<pcl::PointXYZ> pcd_map_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdTfTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
