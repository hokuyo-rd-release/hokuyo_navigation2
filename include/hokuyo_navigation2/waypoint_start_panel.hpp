#ifndef HOKUYO_NAVIGATION2__WAYPOINT_START_PANEL_HPP_
#define HOKUYO_NAVIGATION2__WAYPOINT_START_PANEL_HPP_

#include <atomic>

#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>

namespace hokuyo_navigation2
{

// wait 属性のウェイポイントで待機中に /start_waypoint (std_msgs/Empty) を送るパネル。
// waypoint_manager が出す /waiting_waypoint (待機中のウェイポイント番号, -1 = 待機していない) を見て、
// 待機中のときだけボタンを押せるようにする。
class WaypointStartPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointStartPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onStartClicked();
  void refresh();

private:
  static constexpr int kUnknown = -2;  // /waiting_waypoint をまだ受信していない

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_start_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_waiting_;
  // ROS のコールバックで書き、Qt のタイマで読む
  std::atomic<int> waiting_index_{kUnknown};
  int shown_index_{kUnknown - 1};  // 画面に出している状態 (kUnknown - 1 = 未表示)

  QLabel * status_label_;
  QPushButton * start_button_;
  QTimer * timer_;
};

}  // namespace hokuyo_navigation2

#endif  // HOKUYO_NAVIGATION2__WAYPOINT_START_PANEL_HPP_
