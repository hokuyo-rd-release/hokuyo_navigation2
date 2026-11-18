#include "hokuyo_navigation2/waypoint_start_panel.hpp"

#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace hokuyo_navigation2
{

WaypointStartPanel::WaypointStartPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  status_label_ = new QLabel;
  status_label_->setWordWrap(true);
  start_button_ = new QPushButton("開始 (wait 解除)");
  start_button_->setMinimumHeight(48);
  start_button_->setEnabled(false);

  QVBoxLayout * layout = new QVBoxLayout;
  layout->addWidget(status_label_);
  layout->addWidget(start_button_);
  layout->addStretch();
  setLayout(layout);

  timer_ = new QTimer(this);
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartClicked()));
  connect(timer_, SIGNAL(timeout()), this, SLOT(refresh()));
}

void WaypointStartPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pub_start_ = node->create_publisher<std_msgs::msg::Empty>("/start_waypoint", rclcpp::QoS(10));
  // waypoint_manager は TRANSIENT_LOCAL で出すので、後から起動しても現在の状態を受け取れる
  sub_waiting_ = node->create_subscription<std_msgs::msg::Int32>(
    "/waiting_waypoint", rclcpp::QoS(1).transient_local().reliable(),
    [this](std_msgs::msg::Int32::ConstSharedPtr msg) {waiting_index_ = msg->data;});
  timer_->start(100);
  refresh();
}

void WaypointStartPanel::onStartClicked()
{
  if (!pub_start_) {
    return;
  }
  pub_start_->publish(std_msgs::msg::Empty());
  // 二重押し防止。waypoint_manager が待機を解除すると /waiting_waypoint が -1 になる。
  // 2 秒たっても待機したままなら (指令が届かなかったなど) 表示を戻してもう一度押せるようにする
  start_button_->setEnabled(false);
  status_label_->setText("開始指令を送信しました");
  QTimer::singleShot(2000, this, [this]() {
      shown_index_ = kUnknown - 1;
      refresh();
    });
}

void WaypointStartPanel::refresh()
{
  const int index = waiting_index_;
  if (index == shown_index_) {
    return;
  }
  shown_index_ = index;

  if (index >= 0) {
    status_label_->setText(QString("<b>WP #%1 で待機中</b>").arg(index));
    start_button_->setEnabled(true);
  } else if (index == -1) {
    status_label_->setText("待機していません");
    start_button_->setEnabled(false);
  } else {
    status_label_->setText("waypoint_manager の状態を受信していません");
    start_button_->setEnabled(false);
  }
}

}  // namespace hokuyo_navigation2

PLUGINLIB_EXPORT_CLASS(hokuyo_navigation2::WaypointStartPanel, rviz_common::Panel)
