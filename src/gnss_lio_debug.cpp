#include <rclcpp/rclcpp.hpp>
#include <jsk_rviz_plugin_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector> // std::vector のために追加
#include <numeric> // std::accumulate のために追加

using std::placeholders::_1;

// 移動平均を計算する際の周期のサンプル数
// 平均周波数を計算するために、直近の周期を保持するウィンドウサイズを1000に設定
constexpr size_t WINDOW_SIZE = 1000; 

class OverlayTextNode : public rclcpp::Node
{
public:
  OverlayTextNode()
  : Node("sensor_status_display_node"), last_lidar_odom_time_(this->now()) 
  {
    // パラメータを宣言し、初期値を設定 (変更なし)
    this->declare_parameter("sub_gnss_topic", "fix");
    this->declare_parameter("pub_gnss_text", "gnss_fix_text");
    this->declare_parameter("pub_gnss_data", "gnss_fix_float");
    this->declare_parameter("sub_odometry_topic", "/odometry/switch/type");
    this->declare_parameter("pub_odometry_text_topic", "odometry_type_text");
    this->declare_parameter("sub_lidar_odom_topic", "/hokuyo_lio/lidar_odom"); 
    this->declare_parameter("pub_lidar_odom_rate_text", "lidar_odom_rate_text");

    // パラメータを取得 (変更なし)
    this->get_parameter("sub_gnss_topic", sub_gnss_topic_);
    this->get_parameter("pub_gnss_text", pub_gnss_text_);
    this->get_parameter("pub_gnss_data", pub_gnss_data_);
    this->get_parameter("sub_odometry_topic", sub_odometry_topic_);
    this->get_parameter("pub_odometry_text_topic", pub_odometry_text_topic_);
    this->get_parameter("sub_lidar_odom_topic", sub_lidar_odom_topic_);
    this->get_parameter("pub_lidar_odom_rate_text", pub_lidar_odom_rate_text_);

    // パブリッシャーとサブスクライバーの初期化 (変更なし)
    gnss_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>(pub_gnss_text_, 10);
    float_publisher_ = this->create_publisher<std_msgs::msg::Float32>(pub_gnss_data_, 10);
    odometry_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>(pub_odometry_text_topic_, 10);
    lidar_odom_rate_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>(pub_lidar_odom_rate_text_, 10);
    
    gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      sub_gnss_topic_, 10, std::bind(&OverlayTextNode::navSatStatusCallBack, this, _1));
    odometry_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      sub_odometry_topic_, 10, std::bind(&OverlayTextNode::odometrySwitchTypeCallBack, this, _1));
    lidar_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_lidar_odom_topic_, 10, std::bind(&OverlayTextNode::lidarOdomCallBack, this, _1));

    // 0.5秒タイマーの初期化
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), 
      std::bind(&OverlayTextNode::timer_callback, this));
    
    // OverlayTextの初期設定 (変更なし)
    std_msgs::msg::ColorRGBA gnss_bg_color = createColor(0.0, 0.0, 0.0, 0.5);
    std_msgs::msg::ColorRGBA odometry_bg_color = createColor(0.0, 0.0, 0.0, 0.5);

    gnss_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    gnss_text_.font = "Ubuntu";
    gnss_text_.left = 10;
    gnss_text_.top = 40;
    gnss_text_.width = 300;
    gnss_text_.height = 30;
    gnss_text_.bg_color = gnss_bg_color;

    lidar_odom_rate_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    lidar_odom_rate_text_.font = "Ubuntu";
    lidar_odom_rate_text_.left = 10;
    lidar_odom_rate_text_.top = 70;
    lidar_odom_rate_text_.width = 300;
    lidar_odom_rate_text_.height = 30;
    lidar_odom_rate_text_.bg_color = gnss_bg_color;
    lidar_odom_rate_text_.fg_color = odometry_default_color_;
    // 表示テキストを周波数ベースに変更
    lidar_odom_rate_text_.text = "Lidar Odom Rate: N/A";

    odometry_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    odometry_text_.font = "Ubuntu";
    odometry_text_.left = 10;
    odometry_text_.top = 130; 
    odometry_text_.width = 300;
    odometry_text_.height = 30;
    odometry_text_.bg_color = odometry_bg_color;
    odometry_text_.fg_color = odometry_default_color_;
    odometry_text_.text = "Odometry Type: N/A";
  }

private:
  // メンバー変数 (省略)
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
  
  // Lidar Odometry 周期計測用のメンバー変数
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr lidar_odom_rate_text_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_subscriber_; 
  std::string sub_lidar_odom_topic_;
  std::string pub_lidar_odom_rate_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText lidar_odom_rate_text_;
  rclcpp::Time last_lidar_odom_time_;
  
  // 周期の移動平均を計算するためのコンテナを再追加
  std::vector<double> lidar_odom_periods_; 

  // 0.5秒周期でパブリッシュするためのタイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // 色を簡単に作成するためのヘルパー関数 (変更なし)
  std_msgs::msg::ColorRGBA createColor(double r, double g, double b, double a) {
      std_msgs::msg::ColorRGBA color;
      color.r = r;
      color.g = g;
      color.b = b;
      color.a = a;
      return color;
  }

  // Odometry 用のコールバック関数 (周期計測と移動平均へのデータ追加)
  void lidarOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr /* msg */)
  {
    rclcpp::Time current_time = this->now();
    rclcpp::Duration period_duration = current_time - last_lidar_odom_time_;
    last_lidar_odom_time_ = current_time;
    
    double current_period = period_duration.seconds();

    // 1. 新しい周期をリストに追加
    if (lidar_odom_periods_.empty() && current_period < 0.001) {
        // 初回起動時の異常な短い周期は無視
    } else {
        lidar_odom_periods_.push_back(current_period);
    }

    // 2. リストのサイズを制限 (移動窓の維持)
    if (lidar_odom_periods_.size() > WINDOW_SIZE) {
        lidar_odom_periods_.erase(lidar_odom_periods_.begin()); // 最も古い要素を削除
    }
  }

  // 0.5秒ごとに呼び出されるタイマーコールバック関数 (平均周波数計算とパブリッシュを実行)
  void timer_callback()
  {
    double avg_period = 0.0;
    double frequency_hz = 0.0;
    
    // 1. 移動平均を計算
    if (lidar_odom_periods_.size() > 0) {
        // リスト内の全ての周期を合計
        double sum_of_periods = std::accumulate(lidar_odom_periods_.begin(), lidar_odom_periods_.end(), 0.0);
        
        // 平均周期を計算
        avg_period = sum_of_periods / lidar_odom_periods_.size();

        // 平均周波数 (Hz) を計算 (周波数を表示したいので、周期の逆数をとる)
        if (avg_period > 0.0) {
            frequency_hz = 1.0 / avg_period;
        }
    }

    // 2. 表示テキストの整形とパブリッシュ
    std::stringstream ss;
    if (frequency_hz > 0.0) {
        // 周波数 (Hz) を表示
        ss << "Lidar Odom Rate: " << std::fixed << std::setprecision(2) << frequency_hz << " Hz";

        // 周波数に応じて文字色を変更 (10Hzを基準)
        std_msgs::msg::ColorRGBA rate_color;
        if (frequency_hz >= 9.5) { 
            rate_color = createColor(0.0, 1.0, 0.0, 0.8); // 緑 (良好)
        } else if (frequency_hz >= 5.0) { 
            rate_color = createColor(1.0, 1.0, 0.0, 0.8); // 黄 (許容範囲)
        } else { 
            rate_color = createColor(1.0, 0.0, 0.0, 0.8); // 赤 (要確認)
        }
        lidar_odom_rate_text_.fg_color = rate_color;
    } else {
        // データがまだ十分でない、またはトピックが流れていない場合
        ss << "Lidar Odom Rate: N/A";
        lidar_odom_rate_text_.fg_color = odometry_default_color_;
    }
    
    lidar_odom_rate_text_.text = ss.str();
    lidar_odom_rate_text_publisher_->publish(lidar_odom_rate_text_);
  }

  // GNSS と Odometry のコールバック関数は省略 (変更なし)
  void navSatStatusCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // ... (GNSSステータス表示ロジック)
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