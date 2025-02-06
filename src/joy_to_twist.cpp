#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class SubPub{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Publisher cmd_pub;
    ros::Subscriber joy_sub;

    float max_x =0.5;
    float max_LR =0.5;
    float linear_velocity;
    float angular_velocity;
    float high_mode_scale;
    float low_mode_scale;
    float vel_scale;
    int forward_axis = 1;
    int left_button = 2;
    int right_button = 1;
    int high_button = 5;
    int low_button = 4;
    bool pre_zero = false;
    geometry_msgs::Twist pre_cmd_vel;


public:
    void velPublisher (const sensor_msgs::Joy& joy_msg){
       
        //速度と角速度の大きさ（高速入力(high_button)なら2倍、低速入力(low_button)なら0.5倍）.
        if (joy_msg.buttons[low_button] == 1 && joy_msg.buttons[high_button] == 1) vel_scale = low_mode_scale;
        else if(joy_msg.buttons[low_button] == 1) vel_scale = 1.0;
        else if(joy_msg.buttons[high_button] == 1) vel_scale = high_mode_scale;
        else vel_scale = 0.0;
        
        linear_velocity = max_x*vel_scale;
        angular_velocity = max_LR*vel_scale;

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity * joy_msg.axes[forward_axis];
        cmd_vel.angular.z = angular_velocity * (joy_msg.buttons[left_button]-joy_msg.buttons[right_button]);

        //navigationとの衝突を避けるため、速度・角速度がゼロの司令が続くときは最初の1度だけをパブリッシュ.
        //ゼロ付近のとき,
        if((cmd_vel.linear.x <= 0.01*linear_velocity && cmd_vel.linear.x >= -0.01*linear_velocity ) && (cmd_vel.angular.z <= 0.01*angular_velocity && cmd_vel.angular.z >= -0.01*angular_velocity )){
            //かつ、前回はゼロ付近でなければ、パブリッシュ.
            if(!pre_zero) cmd_pub.publish(cmd_vel);
            pre_zero = true;
        }
        else{
            cmd_pub.publish(cmd_vel);
            pre_zero = false;
        }
    }


    SubPub() : nh(),pnh("~"){
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",5);
        joy_sub = nh.subscribe("joy",5,&SubPub::velPublisher, this);
        
        pnh.getParam("linear_velocity", max_x);
        pnh.getParam("angular_velocity", max_LR);
        pnh.getParam("low_mode_scale", low_mode_scale);
        pnh.getParam("high_mode_scale", high_mode_scale);
        pnh.getParam("forward_axis", forward_axis);
        pnh.getParam("left_button", left_button);
        pnh.getParam("right_button", right_button);
        pnh.getParam("high_button", high_button);
        pnh.getParam("low_button", low_button);
    }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "joy_to_twist_new");
    SubPub subpub;
    ros::spin();
    return 0;
}

