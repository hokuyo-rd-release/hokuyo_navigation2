#include <string>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/msg/point.hpp>

// ROS1のgeometry_msgs::PointはROS2ではgeometry_msgs::msg::Pointに変わります
using geometry_msgs::msg::Point;

class PcdTfTransform : public rclcpp::Node {
private:
    // ROSの設定.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub;
    
    // tf2の設定
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // パラメータ
    bool save_map_flg;
    double pc_save_distance;
    std::string sub_pcd_topic;
    std::string sub_odom_topic;
    std::string pub_pcd_topic;
    std::string orig_frame;
    std::string target_frame;
    std::string map_dir;
    std::string map_name;
    
    Point last_position;
    Point current_position;

    // 点群地図.
    pcl::PointCloud<pcl::PointXYZ> pcd_map;

public:
    // コンストラクタでノードを初期化します
    PcdTfTransform() : Node("pcd_tf_transform") {
        // パラメータの宣言と取得
        this->declare_parameter("save_map", false);
        this->declare_parameter("sub_topic", "hokuyo3d3/hokuyo_cloud2");
        this->declare_parameter("sub_odom_topic", "hokuyo_lio/lidar_odom");
        this->declare_parameter("pub_topic", "world/pcd");
        this->declare_parameter("orig_frame", "Odometry");
        this->declare_parameter("target_frame", "odom");
        this->declare_parameter("map_dir", "/home/ubuntu/catkin_ws/src/fusion_tools/test_tools/PCD");
        this->declare_parameter("map_name", "map.pcd");
        this->declare_parameter("pc_save_distance", 1.0);

        this->get_parameter("save_map", save_map_flg);
        this->get_parameter("sub_topic", sub_pcd_topic);
        this->get_parameter("sub_odom_topic", sub_odom_topic);
        this->get_parameter("pub_topic", pub_pcd_topic);
        this->get_parameter("orig_frame", orig_frame);
        this->get_parameter("target_frame", target_frame);
        this->get_parameter("map_dir", map_dir);
        this->get_parameter("map_name", map_name);
        this->get_parameter("pc_save_distance", pc_save_distance);

        // tf2の初期化
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // publisher,subscriberの設定.
        pcd_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_pcd_topic, 10, std::bind(&PcdTfTransform::pcd_sub_Callback, this, std::placeholders::_1));
        
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            sub_odom_topic, 10, std::bind(&PcdTfTransform::odom_sub_Callback, this, std::placeholders::_1));

        pcd_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_pcd_topic, 10);
        
        // 最初のpoint_cloudを保存するために、大きな値を入れる
        last_position.x = 1000000;
        last_position.y = 1000000;
        last_position.z = 1000000;
    }

    // デストラクタで終了時の処理を実行します
    ~PcdTfTransform() {
        save_Map();
    }

    // 終了時処理(mapを保存)
    void save_Map(){
        if(save_map_flg){
            std::string file_path = map_dir + "/" + map_name;
            pcl::io::savePCDFileASCII (file_path, pcd_map);
            RCLCPP_INFO(this->get_logger(), "Map saved to %s", file_path.c_str());
        }
    }

    //pcdコールバック関数.
    void pcd_sub_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc2_sub_msg){
        double odom_diff_x = current_position.x - last_position.x;
        double odom_diff_y = current_position.y - last_position.y;
        double odom_diff_z = current_position.z - last_position.z;
        double odom_distance = std::sqrt(odom_diff_x*odom_diff_x + odom_diff_y*odom_diff_y + odom_diff_z*odom_diff_z);

        if(odom_distance >= pc_save_distance){
            RCLCPP_INFO(this->get_logger(), "Odometry distance threshold met. Transforming and saving point cloud.");
            last_position = current_position;
            
            sensor_msgs::msg::PointCloud2 transformed_pc;
            
            try {
                // tf2の変換
                // tf2::Duration(1.0) の代わりに tf2::durationFromSec(1.0) を使用する
                tf_buffer->transform(*pc2_sub_msg, transformed_pc, target_frame, tf2::durationFromSec(1.0));
                
                pcd_pub->publish(transformed_pc);
                
                if(save_map_flg){
                    pcl::PointCloud<pcl::PointXYZ> pcd_data;
                    pcl::fromROSMsg(transformed_pc, pcd_data);
                    pcd_map += pcd_data;
                    RCLCPP_INFO(this->get_logger(), "Point cloud added to map. Map size: %zu points.", pcd_map.size());
                }
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            }
        }
    }

    //odomコールバック関数.
    void odom_sub_Callback(const nav_msgs::msg::Odometry::SharedPtr odom_sub_msg){
        current_position = odom_sub_msg->pose.pose.position;
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdTfTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}