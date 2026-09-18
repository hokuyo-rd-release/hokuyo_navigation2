#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>

class CmdVelStopper : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slow_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_obstacle_stop_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_obstacle_slow_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    bool stop_cmdvel_;
    bool slow_down_;
    geometry_msgs::msg::Twist twist_zero_;
    geometry_msgs::msg::Twist twist_slow_;
    float slow_down_cmd_;

    // obstacle_monitor からの停止指令 (レベル方式).
    // waypoint_manager が使う stop/start_cmd_vel とは独立に保持し、OR で合成するため
    // 互いの停止指令を打ち消すことがない.
    bool obstacle_stop_;
    rclcpp::Time obstacle_stop_stamp_;
    double obstacle_stop_timeout_;

    // obstacle_monitor からの減速指令 (0 以下は制限なし)
    float obstacle_slow_cmd_;
    rclcpp::Time obstacle_slow_stamp_;

public:
    CmdVelStopper() : Node("cmdvel_stopper"), stop_cmdvel_(false), slow_down_(false),
                      slow_down_cmd_(0.0f), obstacle_stop_(false),
                      obstacle_stop_stamp_(0, 0, RCL_ROS_TIME),
                      obstacle_slow_cmd_(0.0f),
                      obstacle_slow_stamp_(0, 0, RCL_ROS_TIME) {
        twist_zero_.linear.x = 0.0;
        twist_zero_.linear.y = 0.0;
        twist_zero_.linear.z = 0.0;
        twist_zero_.angular.x = 0.0;
        twist_zero_.angular.y = 0.0;
        twist_zero_.angular.z = 0.0;

        std::string cmd_in_topic = "/wizurg/cmd_vel";
        std::string cmd_out_topic = "/cmd_vel";
        std::string cmd_stop_topic = "/wizurg/stop_cmd_vel";
        std::string cmd_start_topic = "/wizurg/start_cmd_vel";
        std::string cmd_slow_topic = "/wizurg/slow_cmd_vel";
        std::string cmd_obstacle_stop_topic = "/wizurg/obstacle_stop_cmd_vel";
        std::string cmd_obstacle_slow_topic = "/wizurg/obstacle_slow_cmd_vel";

        // 停止指令が途絶えた場合は停止を解除する (obstacle_monitor が落ちても走行不能にしない).
        // 衝突回避自体は local_costmap と DWB が担保する.
        obstacle_stop_timeout_ = this->declare_parameter<double>("obstacle_stop_timeout", 1.0);

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_in_topic, 10, std::bind(&CmdVelStopper::callbackCMD, this, std::placeholders::_1));
        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            cmd_stop_topic, 10, std::bind(&CmdVelStopper::callbackStop, this, std::placeholders::_1));
        sub_start_ = this->create_subscription<std_msgs::msg::Empty>(
            cmd_start_topic, 10, std::bind(&CmdVelStopper::callbackStart, this, std::placeholders::_1));
        sub_slow_ = this->create_subscription<std_msgs::msg::Float32>(
            cmd_slow_topic, 10, std::bind(&CmdVelStopper::callbackSlow, this, std::placeholders::_1));
        sub_obstacle_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            cmd_obstacle_stop_topic, 10,
            std::bind(&CmdVelStopper::callbackObstacleStop, this, std::placeholders::_1));
        sub_obstacle_slow_ = this->create_subscription<std_msgs::msg::Float32>(
            cmd_obstacle_slow_topic, 10,
            std::bind(&CmdVelStopper::callbackObstacleSlow, this, std::placeholders::_1));

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic, 10);
    }

    void callbackObstacleStop(const std_msgs::msg::Bool::SharedPtr msg) {
        obstacle_stop_ = msg->data;
        obstacle_stop_stamp_ = this->now();
    }

    bool obstacleStopActive() {
        if (!obstacle_stop_) {
            return false;
        }
        if ((this->now() - obstacle_stop_stamp_).seconds() > obstacle_stop_timeout_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "障害物停止指令が途絶えたため停止を解除します");
            obstacle_stop_ = false;
            return false;
        }
        return true;
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

    void callbackObstacleSlow(const std_msgs::msg::Float32::SharedPtr msg) {
        // 0 以下は「制限なし」を意味する
        obstacle_slow_cmd_ = msg->data;
        obstacle_slow_stamp_ = this->now();
    }

    bool obstacleSlowActive() {
        if (obstacle_slow_cmd_ <= 0.0f) {
            return false;
        }
        if ((this->now() - obstacle_slow_stamp_).seconds() > obstacle_stop_timeout_) {
            obstacle_slow_cmd_ = 0.0f;
            return false;
        }
        return true;
    }

    void callbackCMD(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (stop_cmdvel_ || obstacleStopActive()) {
            pub_cmd_->publish(twist_zero_);
            return;
        }

        // 複数の減速要求のうち最も遅いものを採用する
        float limit = -1.0f;
        if (slow_down_) {
            limit = slow_down_cmd_;
        }
        if (obstacleSlowActive() && (limit < 0.0f || obstacle_slow_cmd_ < limit)) {
            limit = obstacle_slow_cmd_;
        }

        if (limit >= 0.0f && msg->linear.x > limit) {
            twist_slow_ = *msg;
            twist_slow_.linear.x = limit;
            if (msg->linear.x != 0.0) {
                twist_slow_.angular.z = twist_slow_.linear.x*(msg->angular.z)/(msg->linear.x);
            } else {
                twist_slow_.angular.z = 0.0;
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