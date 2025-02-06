#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


int assign_x = -1;
int assign_L = -1;
int assign_R = -1;
int assign_high = -1;
int assign_low = -1;
int update_axe = -1;
int update_button = -1;
bool axes[6] = {false};
bool pre_axes[6] = {false};
bool buttons[12] = {false};
bool pre_buttons[12] = {false};


class SubPub{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber joy_sub;

    float max_x =0.9;
    float max_LR =0.5;
    float vel_line,vel_ang,vel_scale;

    bool pre_zero = false;
    geometry_msgs::Twist pre_cmd_vel;


public:
    void update (const sensor_msgs::Joy& joy_msg){
       
        //速度と角速度の大きさ（高速入力(assign_high)なら2倍、低速入力(assign_low)なら0.5倍）.
        if(joy_msg.buttons[assign_low] == 1) vel_scale = 0.5;
        else if(joy_msg.buttons[assign_high] == 1) vel_scale = 2.0;
        else vel_scale = 1.0;
        
        vel_line = max_x*vel_scale;
        vel_ang = max_LR*vel_scale;

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = vel_line * joy_msg.axes[assign_x];
        cmd_vel.angular.z = vel_ang * (joy_msg.buttons[assign_L]-joy_msg.buttons[assign_R]);

        //navigationとの衝突を避けるため、速度・角速度がゼロの司令が続くときは最初の1度だけをパブリッシュ.
        //ゼロ付近のとき,
        if((cmd_vel.linear.x <= 0.01*vel_line && cmd_vel.linear.x >= -0.01*vel_line ) && (cmd_vel.angular.z <= 0.01*vel_ang && cmd_vel.angular.z >= -0.01*vel_ang )){
            //かつ、前回はゼロ付近でなければ、パブリッシュ.
            if(!pre_zero) cmd_pub.publish(cmd_vel);
            pre_zero = true;
        }
        else{
            cmd_pub.publish(cmd_vel);
            pre_zero = false;
        }


        //入力されたボタンを更新する.
        int i;
        for(i=0 ; i<11 ; i++){
            pre_buttons[i] = buttons[i];
            if(joy_msg.buttons[i] == 0) buttons[i] = false;
            else buttons[i] = true;
        }
        for(i=0 ; i<5 ; i++){
            pre_axes[i] = axes[i];
            if(joy_msg.axes[i] <= 0.1 && joy_msg.axes[i] >= -0.1) axes[i] = false;
            else axes[i] = true;
        }
    }



    SubPub() : nh(){
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",5);
        joy_sub = nh.subscribe("joy",5,&SubPub::update, this);
        
    }

};


    bool update_axes (){
        int i;
        int j=0;
        for(i=0 ; i<5 ; i++){
            if(pre_axes[i] != axes[i]){
                j++;
                update_axe = i;
            }
        }
        if(j==1){return true;}
        else {return false;}
    }
    bool update_buttons (){
        int i;
        int j=0;
        for(i=0 ; i<11 ; i++){
            if(pre_buttons[i] != buttons[i]){
                j++;
                update_button = i;
            }
        }
        if(j==1){return true;}
        else {return false;}
    }

int main(int argc, char** argv){

    ros::init(argc, argv, "controller_setup");
    SubPub subpub;
    
    ros::Rate loop_rate(10.0);
        ros::spinOnce();

        printf("\n\n\n次の制御に用いるスティックを入力してください。\n");
        printf("前進\n");
        while(!update_axes()){
            ros::spinOnce();
        }
        assign_x = update_axe;
        printf("%d\n",assign_x);


        printf("\n\n\n次の制御に用いるボタンを押してください。\n");
        printf("左回転\n");
        while(!update_buttons() || (update_button == assign_L) ||(update_button == assign_R) ||(update_button == assign_high) ||(update_button == assign_low)){
            ros::spinOnce();
        }
        assign_L = update_button;
        printf("%d\n",assign_L);

        printf("右回転\n");
        while(!update_buttons() || (update_button == assign_L) ||(update_button == assign_R) ||(update_button == assign_high) ||(update_button == assign_low)){
            ros::spinOnce();
        }
        assign_R = update_button;
        printf("%d\n",assign_R);
        
        printf("高速モード\n");
        while(!update_buttons() || (update_button == assign_L) ||(update_button == assign_R) ||(update_button == assign_high) ||(update_button == assign_low)){
            ros::spinOnce();
        }
        assign_high = update_button;
        printf("%d\n",assign_high);
        
        printf("低速モード\n");
        while(!update_buttons() || (update_button == assign_L) ||(update_button == assign_R) ||(update_button == assign_high) ||(update_button == assign_low)){
            ros::spinOnce();
        }
        assign_low = update_button;
        printf("%d\n",assign_low);

        printf("\n\n~/catkin_ws/src/wizurg/params/teleop_joy.yaml\n");
        printf("を開き、次のように編集してください。\n\n");
        printf("forward_axis: %d",assign_x);
        printf("left_button: %d",assign_L);
        printf("right_button: %d",assign_R);
        printf("high_button: %d",assign_high);
        printf("low_button: %d",assign_low);

    return 0;
}



