#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>

class CmdVelStopper : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    bool stop_cmdvel_;
    geometry_msgs::msg::Twist twist_zero_;

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

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_in_topic, 10, std::bind(&CmdVelStopper::callbackCMD, this, std::placeholders::_1));
        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            cmd_stop_topic, 10, std::bind(&CmdVelStopper::callbackStop, this, std::placeholders::_1));
        sub_start_ = this->create_subscription<std_msgs::msg::Empty>(
            cmd_start_topic, 10, std::bind(&CmdVelStopper::callbackStart, this, std::placeholders::_1));
        
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic, 10);
    }

    void callbackStop(const std_msgs::msg::Empty::SharedPtr msg) {
        stop_cmdvel_ = true;
    }

    void callbackStart(const std_msgs::msg::Empty::SharedPtr msg) {
        stop_cmdvel_ = false;
    }

    void callbackCMD(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (stop_cmdvel_) {
            pub_cmd_->publish(twist_zero_);
        } else {
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