#include <rclcpp/rclcpp.hpp>
#include <jsk_rviz_plugin_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <string>

using std::placeholders::_1;

class OverlayTextNode : public rclcpp::Node
{
public:
  OverlayTextNode()
  : Node("sensor_status_display_node")
  {
    // パラメータを宣言し、初期値を設定
    this->declare_parameter("sub_gnss_topic", "fix");
    this->declare_parameter("pub_gnss_text", "gnss_fix_text");
    this->declare_parameter("pub_gnss_data", "gnss_fix_float");
    this->declare_parameter("sub_odometry_topic", "/odometry/switch/type");
    this->declare_parameter("pub_odometry_text_topic", "odometry_type_text");

    // パラメータを取得
    this->get_parameter("sub_gnss_topic", sub_gnss_topic_);
    this->get_parameter("pub_gnss_text", pub_gnss_text_);
    this->get_parameter("pub_gnss_data", pub_gnss_data_);
    this->get_parameter("sub_odometry_topic", sub_odometry_topic_);
    this->get_parameter("pub_odometry_text_topic", pub_odometry_text_topic_);

    // パブリッシャーとサブスクライバーの初期化
    gnss_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>(pub_gnss_text_, 10);
    float_publisher_ = this->create_publisher<std_msgs::msg::Float32>(pub_gnss_data_, 10);
    odometry_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>(pub_odometry_text_topic_, 10);
    
    gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      sub_gnss_topic_, 10, std::bind(&OverlayTextNode::navSatStatusCallBack, this, _1));
    odometry_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      sub_odometry_topic_, 10, std::bind(&OverlayTextNode::odometrySwitchTypeCallBack, this, _1));

    // GNSSテキストの初期設定
    gnss_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    gnss_text_.font = "Ubuntu";
    gnss_text_.left = 10;
    gnss_text_.top = 40;
    gnss_text_.width = 300;
    gnss_text_.height = 30;
    std_msgs::msg::ColorRGBA gnss_bg_color;
    gnss_bg_color.r = 0.0;
    gnss_bg_color.g = 0.0;
    gnss_bg_color.b = 0.0;
    gnss_bg_color.a = 0.5;
    gnss_text_.bg_color = gnss_bg_color;

    // オドメトリテキストの初期設定
    odometry_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    odometry_text_.font = "Ubuntu";
    odometry_text_.left = 10;
    odometry_text_.top = 100;
    odometry_text_.width = 300;
    odometry_text_.height = 30;

    std_msgs::msg::ColorRGBA odometry_bg_color;
    odometry_bg_color.r = 0.0;
    odometry_bg_color.g = 0.0;
    odometry_bg_color.b = 0.0;
    odometry_bg_color.a = 0.5;
    odometry_text_.bg_color = odometry_bg_color;
    odometry_text_.fg_color = odometry_default_color_;
    odometry_text_.text = "Odometry Type: N/A";
  }

private:
  // メンバー変数として再定義
  jsk_rviz_plugin_msgs::msg::OverlayText gnss_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText odometry_text_;
  std_msgs::msg::ColorRGBA odometry_lio_switch_color_ = createColor(0.0, 1.0, 1.0, 0.8);
  std_msgs::msg::ColorRGBA odometry_gnss_switch_color_ = createColor(0.0, 1.0, 0.0, 0.8);
  std_msgs::msg::ColorRGBA odometry_lio_raw_color_ = createColor(1.0, 1.0, 0.0, 0.8);
  std_msgs::msg::ColorRGBA odometry_default_color_ = createColor(1.0, 1.0, 1.0, 0.8);

  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr gnss_text_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_publisher_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr odometry_text_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr odometry_subscriber_;

  std::string pub_gnss_text_;
  std::string sub_gnss_topic_;
  std::string pub_gnss_data_;
  std::string sub_odometry_topic_;
  std::string pub_odometry_text_topic_;
  
  // 色を簡単に作成するためのヘルパー関数
  std_msgs::msg::ColorRGBA createColor(double r, double g, double b, double a) {
      std_msgs::msg::ColorRGBA color;
      color.r = r;
      color.g = g;
      color.b = b;
      color.a = a;
      return color;
  }

  void navSatStatusCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    gnss_text_.text = "";
    std_msgs::msg::ColorRGBA color;
    double gnss_status = std::sqrt(msg->position_covariance[0]);
    std_msgs::msg::Float32 float_data;
    float_data.data = static_cast<float>(gnss_status);

    if (gnss_status <= 0.1) {
      color = createColor(0.0, 1.0, 0.0, 0.8);
      gnss_text_.text = "GNSSの精度は良好です。";
    } else if (gnss_status > 0.1 && gnss_status <= 4) {
      color = createColor(237.0 / 255.0, 212.0 / 255.0, 0.0, 0.8);
      gnss_text_.text = "GNSSの精度は中程度です。";
    } else if (gnss_status > 4) {
      color = createColor(1.0, 0.0, 0.0, 0.8);
      gnss_text_.text = "GNSSの精度が低い状態です。";
    } else {
        color = createColor(25.0 / 255.0, 255.0 / 255.0, 240.0 / 255.0, 0.8);
        gnss_text_.text = "GNSSの精度: N/A";
    }
    
    gnss_text_.fg_color = color;
    gnss_text_publisher_->publish(gnss_text_);
    float_publisher_->publish(float_data);
  }

  void odometrySwitchTypeCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    odometry_text_.text = "Odometry Type: " + msg->data;

    // 受信した文字列に応じて文字色を変更
    if (msg->data == "LIO (switch)") {
      odometry_text_.fg_color = odometry_lio_switch_color_;
    } else if (msg->data == "GNSS (switch)") {
      odometry_text_.fg_color = odometry_gnss_switch_color_;
    } else if (msg->data == "LIO (raw)") {
      odometry_text_.fg_color = odometry_lio_raw_color_;
    } else {
      odometry_text_.fg_color = odometry_default_color_;
    }

    odometry_text_publisher_->publish(odometry_text_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OverlayTextNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}