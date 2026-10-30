// rsf_fix → TF(map→base_link) の位置系統で、各段の受信状況をログに残すための補助クラス.
// fix2xyz_packages_ros2/fix2xyz/include/rsf_chain_monitor.h と同じ内容.
//
// 出力するログ:
//   - 初回受信 (INFO)
//   - スタンプの巻き戻り・重複・飛び (WARN, 即時)
//   - 受信途絶の開始と復帰 (WARN, 即時. 途絶判定は stall_timeout 秒. 1Hz のトピックがあるので 3 秒程度にする)
//   - 定期要約 (INFO, summarize() の呼び出しごと)
#ifndef RSF_CHAIN_MONITOR_HPP_
#define RSF_CHAIN_MONITOR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

class RsfChainMonitor {
public:
  RsfChainMonitor(rclcpp::Node * node, const std::string & label, double stall_timeout)
  : node_(node), label_(label), stall_timeout_(stall_timeout),
    start_(Clock::now()), window_start_(start_) {}

  // メッセージを受信したときに呼ぶ.
  void onReceive(const builtin_interfaces::msg::Time & stamp_msg) {
    const auto now = Clock::now();
    const double stamp = stamp_msg.sec + stamp_msg.nanosec * 1e-9;
    const double latency = node_->now().seconds() - stamp;

    if (total_ == 0) {
      RCLCPP_INFO(node_->get_logger(), "[%s] 初回受信: stamp=%.3f now-stamp=%+.3fs (起動から %.2fs)",
        label_.c_str(), stamp, latency, elapsed(start_, now));
    } else {
      const double rx_gap = elapsed(last_rx_, now);
      max_rx_gap_ = std::max(max_rx_gap_, rx_gap);

      const double d_stamp = stamp - last_stamp_;
      if (d_stamp < 0.0) {
        ++n_backward_;
        RCLCPP_WARN(node_->get_logger(), "[%s] スタンプが巻き戻った: %.3f -> %.3f (%+.3fs)",
          label_.c_str(), last_stamp_, stamp, d_stamp);
      } else if (d_stamp == 0.0) {
        ++n_repeat_;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "[%s] 同じスタンプを再受信: %.3f", label_.c_str(), stamp);
      } else if (d_stamp > stall_timeout_) {
        RCLCPP_WARN(node_->get_logger(), "[%s] スタンプが飛んだ: %.3f -> %.3f (%+.3fs, 受信間隔 %.3fs)",
          label_.c_str(), last_stamp_, stamp, d_stamp, rx_gap);
      }

      if (stalled_) {
        RCLCPP_WARN(node_->get_logger(), "[%s] 受信再開: %.2fs 途絶していた (stamp %.3f -> %.3f, now-stamp=%+.3fs)",
          label_.c_str(), rx_gap, last_stamp_, stamp, latency);
        stalled_ = false;
      }
    }
    if (stamp_msg.sec == 0 && stamp_msg.nanosec == 0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[%s] スタンプが 0", label_.c_str());
    }

    last_rx_ = now;
    last_stamp_ = stamp;
    last_latency_ = latency;
    ++total_;
    ++window_;
  }

  // 受信が途絶していないかを調べる. 途絶し始めたときだけ WARN を出す.
  void checkStall() {
    if (total_ == 0 || stalled_) {
      return;
    }
    const double age = elapsed(last_rx_, Clock::now());
    if (age > stall_timeout_) {
      stalled_ = true;
      RCLCPP_WARN(node_->get_logger(), "[%s] 受信が途絶: 最終受信から %.2fs, 最終stamp=%.3f (受信累計 %lu)",
        label_.c_str(), age, last_stamp_, static_cast<unsigned long>(total_));
    }
  }

  // 前回の呼び出しからの受信状況を 1 行にまとめて返し、周期内の集計をリセットする.
  std::string summarize() {
    const auto now = Clock::now();
    const double period = elapsed(window_start_, now);
    char buf[384];
    if (total_ == 0) {
      std::snprintf(buf, sizeof(buf), "[%s] 未受信 (起動から %.1fs)", label_.c_str(), elapsed(start_, now));
    } else {
      std::string anomalies;
      if (n_backward_ > 0) {
        anomalies += " backward=" + std::to_string(n_backward_);
      }
      if (n_repeat_ > 0) {
        anomalies += " repeat=" + std::to_string(n_repeat_);
      }
      std::snprintf(buf, sizeof(buf),
        "[%s] rx=%lu (%.1fHz) age=%.2fs max_gap=%.2fs stamp=%.3f now-stamp=%+.3fs%s",
        label_.c_str(), static_cast<unsigned long>(window_), period > 0.0 ? window_ / period : 0.0,
        elapsed(last_rx_, now), max_rx_gap_, last_stamp_, last_latency_, anomalies.c_str());
    }
    window_ = 0;
    max_rx_gap_ = 0.0;
    n_backward_ = 0;
    n_repeat_ = 0;
    window_start_ = now;
    return buf;
  }

  bool received() const { return total_ > 0; }

  // 最終受信からの経過時間 [s]. 未受信なら負値.
  double age() const { return total_ == 0 ? -1.0 : elapsed(last_rx_, Clock::now()); }

private:
  using Clock = std::chrono::steady_clock;  // PC の時刻補正の影響を受けないように steady を使う

  static double elapsed(Clock::time_point from, Clock::time_point to) {
    return std::chrono::duration<double>(to - from).count();
  }

  rclcpp::Node * node_;
  std::string label_;
  double stall_timeout_;

  Clock::time_point start_;
  Clock::time_point last_rx_;
  Clock::time_point window_start_;

  double last_stamp_ = 0.0;
  double last_latency_ = 0.0;
  double max_rx_gap_ = 0.0;
  uint64_t total_ = 0;
  uint64_t window_ = 0;
  uint64_t n_backward_ = 0;
  uint64_t n_repeat_ = 0;
  bool stalled_ = false;
};

// クォータニオンからヨー角 [rad] を求める.
inline double rsfChainYaw(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

#endif  // RSF_CHAIN_MONITOR_HPP_
