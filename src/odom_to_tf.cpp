#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "hokuyo_navigation2/rsf_chain_monitor.hpp"

using namespace std;

class OdomToTf : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string sub_topic_name_;
    std::string pub_topic_name_;
    std::string frame_id_;
    std::string child_frame_id_;

    bool initial_tf_en_;

    // 受信状況のログ出力
    std::unique_ptr<RsfChainMonitor> in_monitor_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    uint64_t tf_sent_ = 0;
    uint64_t n_invalid_ = 0;
    nav_msgs::msg::Odometry last_msg_;

public:
    OdomToTf() : Node("odom_to_tf") {
        // Declare and get parameters
        this->declare_parameter<std::string>("sub_topic_name", "/rsf/rsf_odom_on_map");
        this->declare_parameter<std::string>("pub_topic_name", "/dummy");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<bool>("initial_tf_en", true);
        this->declare_parameter<double>("diag_period", 1.0);
        // 最も遅い GNSS 系のトピックが 1Hz なので、1Hz で 2 回続けて欠けたら途絶とみなす
        this->declare_parameter<double>("stall_timeout", 3.0);

        this->get_parameter("sub_topic_name", sub_topic_name_);
        this->get_parameter("pub_topic_name", pub_topic_name_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);
        this->get_parameter("initial_tf_en", initial_tf_en_);
        const double diag_period = this->get_parameter("diag_period").as_double();
        const double stall_timeout = this->get_parameter("stall_timeout").as_double();


        publisher_ = create_publisher<nav_msgs::msg::Odometry>(pub_topic_name_, 10);
        subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            sub_topic_name_, 10,
            std::bind(&OdomToTf::callback, this, std::placeholders::_1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        in_monitor_ = std::make_unique<RsfChainMonitor>(this, sub_topic_name_, stall_timeout);
        diag_timer_ = create_wall_timer(
            std::chrono::duration<double>(diag_period), std::bind(&OdomToTf::diagCallback, this));
        RCLCPP_INFO(get_logger(), "起動: %s -> TF %s->%s (diag_period=%.2fs, stall_timeout=%.2fs)",
            sub_topic_name_.c_str(), frame_id_.c_str(), child_frame_id_.c_str(), diag_period, stall_timeout);


        if (initial_tf_en_) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = this->now();
            transform_stamped.header.frame_id = frame_id_;
            transform_stamped.child_frame_id = child_frame_id_;
            transform_stamped.transform.translation.x = 0.0;
            transform_stamped.transform.translation.y = 0.0;
            transform_stamped.transform.translation.z = 0.0;
            transform_stamped.transform.rotation.x = 0.0;
            transform_stamped.transform.rotation.y = 0.0;
            transform_stamped.transform.rotation.z = 0.0;
            transform_stamped.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(transform_stamped);
        }
    }

    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        in_monitor_->onReceive(msg->header.stamp);
        checkPose(*msg);
        last_msg_ = *msg;

        auto pub_msg = *msg;
        pub_msg.header.frame_id = frame_id_;
        pub_msg.child_frame_id = child_frame_id_;

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = msg->header;
        transform_stamped.header.frame_id = frame_id_;
        transform_stamped.child_frame_id = child_frame_id_;
        transform_stamped.transform.translation.x = pub_msg.pose.pose.position.x;
        transform_stamped.transform.translation.y = pub_msg.pose.pose.position.y;
        transform_stamped.transform.translation.z = pub_msg.pose.pose.position.z;
        transform_stamped.transform.rotation = pub_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform_stamped);
        ++tf_sent_;
    }

    // NaN や正規化されていないクォータニオンは tf2 側で捨てられ、TF が止まって見えるので記録する.
    void checkPose(const nav_msgs::msg::Odometry & msg) {
        const auto & p = msg.pose.pose.position;
        const auto & q = msg.pose.pose.orientation;
        const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(q_norm) || std::abs(q_norm - 1.0) > 0.01) {
            ++n_invalid_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "不正な姿勢を受信 (TF が捨てられる可能性): pos=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f) |q|=%.4f",
                p.x, p.y, p.z, q.x, q.y, q.z, q.w, q_norm);
        }
    }

    void diagCallback() {
        in_monitor_->checkStall();
        const std::string summary = in_monitor_->summarize();
        if (!in_monitor_->received()) {
            RCLCPP_INFO(get_logger(), "%s", summary.c_str());
            return;
        }
        const auto & p = last_msg_.pose.pose.position;
        const auto & q = last_msg_.pose.pose.orientation;
        RCLCPP_INFO(get_logger(), "%s | tf_sent=%lu invalid=%lu x=%.3f y=%.3f yaw=%.1fdeg",
            summary.c_str(), static_cast<unsigned long>(tf_sent_), static_cast<unsigned long>(n_invalid_),
            p.x, p.y, rsfChainYaw(q.x, q.y, q.z, q.w) * 180.0 / M_PI);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
