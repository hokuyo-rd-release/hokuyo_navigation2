#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>

class CmdVelStopper : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slow_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    bool stop_cmdvel_;
    bool slow_down_;
    geometry_msgs::msg::Twist twist_zero_;
    geometry_msgs::msg::Twist twist_slow_;
    float slow_down_cmd_;

public:
    CmdVelStopper() : Node("cmdvel_stopper"), stop_cmdvel_(false) {
        twist_zero_.linear.x = 0.0;
        twist_zero_.linear.y = 0.0;
        twist_zero_.linear.z = 0.0;
        twist_zero_.angular.x = 0.0;
        twist_zero_.angular.y = 0.0;
        twist_zero_.angular.z = 0.0;

        std::string cmd_in_topic = "/wizurg_tmp/cmd_vel";
        std::string cmd_out_topic = "/cmd_vel";
        std::string cmd_stop_topic = "/wizurg/stop_cmd_vel";
        std::string cmd_start_topic = "/wizurg/start_cmd_vel";
        std::string cmd_slow_topic = "/wizurg/slow_cmd_vel";

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_in_topic, 10, std::bind(&CmdVelStopper::callbackCMD, this, std::placeholders::_1));
        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            cmd_stop_topic, 10, std::bind(&CmdVelStopper::callbackStop, this, std::placeholders::_1));
        sub_start_ = this->create_subscription<std_msgs::msg::Empty>(
            cmd_start_topic, 10, std::bind(&CmdVelStopper::callbackStart, this, std::placeholders::_1));
        sub_slow_ = this->create_subscription<std_msgs::msg::Float32>(
            cmd_slow_topic, 10, std::bind(&CmdVelStopper::callbackSlow, this, std::placeholders::_1));
        
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic, 10);
    }

    void callbackStop(const std_msgs::msg::Empty::SharedPtr msg) {
        stop_cmdvel_ = true;
    }

    void callbackStart(const std_msgs::msg::Empty::SharedPtr msg) {
        stop_cmdvel_ = false;
        slow_down_ = false;
    }

    void callbackSlow(const std_msgs::msg::Float32::SharedPtr msg){
        slow_down_ = true;
        slow_down_cmd_ = msg->data;
        twist_slow_.linear.x = slow_down_cmd_;
    }

    void callbackCMD(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (stop_cmdvel_) {
            pub_cmd_->publish(twist_zero_);
        } 
        else if (slow_down_){
            twist_slow_ = *msg;
            if (msg->linear.x > slow_down_cmd_){
                twist_slow_.linear.x = slow_down_cmd_;

                if (msg->linear.x != 0.0){
                    twist_slow_.angular.z = twist_slow_.linear.x*(msg->angular.z)/(msg->linear.x);
                } else {
                    twist_slow_.angular.z = 0.0;
                }
            }
            pub_cmd_->publish(twist_slow_);
        }
        else {
            pub_cmd_->publish(*msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelStopper>());
    rclcpp::shutdown();
    return 0;
}