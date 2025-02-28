#include <string>
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;


class PcdTfTransform{
private:    
    
    // ROSの設定.
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber pcd_sub;
    ros::Subscriber odom_sub;
    ros::Publisher pcd_pub;
    
    bool save_map_flg;
    int pc_save_distance;
    // 扱うトピック名.
    std::string sub_pcd_topic;
    std::string sub_odom_topic;
    std::string pub_pcd_topic;
    // フレームID.
    std::string orig_frame;
    std::string target_frame;

    std::string map_dir;
    std::string map_name;
    geometry_msgs::Point last_position;
    geometry_msgs::Point current_position;

    tf::TransformListener _tflistener;


    // 点群地図.
    pcl::PointCloud<pcl::PointXYZ> pcd_map;

    
public:

    // 終了時処理(mapを保存)
    void save_Map(){
        if(save_map_flg){
            string file_path = map_dir + "/" + map_name;
            pcl::PCDWriter pcd_writer;
            // pcd_writer.writeBinary(file_path, pcd_map);
            pcl::io::savePCDFileASCII (file_path, pcd_map);
        }
    }

    //pcdコールバック関数.
    void pcd_sub_Callback(const sensor_msgs::PointCloud2& pc2_sub_msg){
        double odom_diff_x = current_position.x - last_position.x;
        double odom_diff_y = current_position.y - last_position.z;
        double odom_diff_z = current_position.z - last_position.z;
        double odom_distance = sqrt(odom_diff_x*odom_diff_x + odom_diff_y*odom_diff_y + odom_diff_z*odom_diff_z);
        if(odom_distance >= pc_save_distance){
            last_position = current_position;

            sensor_msgs::PointCloud pc1_sub_msg;
            sensor_msgs::PointCloud pc1_pub_msg;
            sensor_msgs::PointCloud2 pc2_pub_msg;

            //sensor_msgs::convertPointCloud2ToPointCloud(pc2_sub_msg, pc1_sub_msg);
            try{
                //_tflistener.waitForTransform(target_frame, pc2_sub_msg.header.frame_id, pc2_sub_msg.header.stamp, ros::Duration(1.0));
                //_tflistener.transformPointCloud(target_frame, pc2_sub_msg.header.stamp, pc1_sub_msg, pc2_sub_msg.header.frame_id, pc1_pub_msg);
                //sensor_msgs::convertPointCloudToPointCloud2(pc1_pub_msg, pc2_pub_msg);
                pc2_pub_msg = pc2_sub_msg;
                pc2_pub_msg.header.frame_id = target_frame;
                pcd_pub.publish(pc2_pub_msg);
                if(save_map_flg){
                    pcl::PointCloud<pcl::PointXYZ> pcd_data;    
                    pcl::fromROSMsg(pc2_pub_msg, pcd_data);
                    pcd_map += pcd_data;
                }
            }
            catch(tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }

    }

    //odomコールバック関数.
    void odom_sub_Callback(const nav_msgs::Odometry& odom_sub_msg){
        current_position = odom_sub_msg.pose.pose.position;
    }
    
    //初期化処理.
    PcdTfTransform() : nh(),pnh("~"){
        //各rosparamのデフォルト値.
        save_map_flg = false;
        sub_pcd_topic = "hokuyo3d3/hokuyo_cloud2";
        sub_odom_topic = "hokuyo_lio/lidar_odom";
        pub_pcd_topic = "world/pcd";
        orig_frame = "Odometry";
        target_frame = "odom";
        map_dir = "/home/ubuntu/catkin_ws/src/fusion_tools/test_tools/PCD";
        map_name = "map.pcd";
        pc_save_distance = 1;
        
        //rosparamの取得.
        pnh.getParam("save_map", save_map_flg);
        pnh.getParam("sub_topic", sub_pcd_topic);
        pnh.getParam("sub_odom_topic", sub_odom_topic);
        pnh.getParam("pub_topic", pub_pcd_topic);
        pnh.getParam("orig_frame", orig_frame);
        pnh.getParam("target_frame", target_frame);
        pnh.getParam("map_dir", map_dir);
        pnh.getParam("map_name", map_name);
        pnh.getParam("pc_save_distance", pc_save_distance);
        
        //publisher,subscriberの設定.
        pcd_sub = nh.subscribe( sub_pcd_topic,10 ,&PcdTfTransform::pcd_sub_Callback , this);
        odom_sub = nh.subscribe( sub_odom_topic,10 ,&PcdTfTransform::odom_sub_Callback , this);
        pcd_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_pcd_topic,10);

        //↓　一回目のpoint_cloudは保存したい.
        //↓　ので、移動距離判定が通るように大きな値を入れる.
        last_position.x = 1000000;
        last_position.y = 1000000;
        last_position.z = 1000000;
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "pcd_tf_transform");
    PcdTfTransform pcd_tf_transform;
    
    ros::spin();
    pcd_tf_transform.save_Map();

    return 0;
}
