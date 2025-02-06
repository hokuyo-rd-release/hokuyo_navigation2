#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_rviz_plugins/OverlayMenu.h>
#include <string>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>

jsk_rviz_plugins::OverlayText text;
std_msgs::ColorRGBA color0, color1, color2, color3;


class OverlayText{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Publisher text_publisher;
    ros::Publisher float_publisher;
    ros::Subscriber subscriber;
    std::string pub_fix_text;
    std::string sub_topic;
    std::string pub_fix_data;
    std_msgs::Float32 float_data;

public:
    void NavSatStatusCallBack(const sensor_msgs::NavSatFix msg){
        text.action = jsk_rviz_plugins::OverlayText::ADD;
        text.font = "Ubuntu";
        text.text = "";
        color0.r = 25.0 / 255;
        color0.g = 255.0 / 255;
        color0.b = 240.0 / 255;
        color0.a = 0.8;
        text.fg_color = color0;

        float gnss_status = sqrt(msg.position_covariance[0]);
        float_data.data = gnss_status;

        if(gnss_status <= 0.1){
            color1.r = 0.0 / 255;
            color1.g = 255.0 / 255;
            color1.b = 0.0 / 255;
            color1.a = 0.8;
            text.fg_color = color1;
            text.text = "GNSSの精度は良好です。";
        }
        if(gnss_status > 0.1 && gnss_status <= 4){
            color2.r = 237.0 / 255;
            color2.g = 212.0 / 255;
            color2.b = 0.0 / 255;
            color2.a = 0.8;
            text.fg_color = color2;
            text.text="GNSSの精度は中程度です。";
        }
        if(gnss_status > 4){
            color3.r = 255.0 / 255;
            color3.g = 0.0 / 255;
            color3.b = 0.0 / 255;
            color3.a = 0.8;
            text.fg_color = color3;
            text.text = "GNSSの精度が低い状態です。";
        }
        text_publisher.publish(text);
        float_publisher.publish(float_data);
    }

    OverlayText() : nh(),pnh("~"){
        sub_topic = "fix";
        pub_fix_text = "gnss_fix_text";
        pub_fix_data = "gnss_fix_float";
        subscriber = nh.subscribe(sub_topic,10,&OverlayText::NavSatStatusCallBack, this);
        text_publisher = nh.advertise<jsk_rviz_plugins::OverlayText>(pub_fix_text,10);
        float_publisher = nh.advertise<std_msgs::Float32>(pub_fix_data,10);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "gnss_fix_node");
    OverlayText gnss_fix;
    ros::spin();
    return 0;
}