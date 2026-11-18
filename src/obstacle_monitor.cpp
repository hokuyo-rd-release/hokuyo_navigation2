// obstacle_monitor
//
// 3D 点群 (/hokuyo3d/hokuyo_cloud2) をそのまま使い、グローバルプラン上を塞ぐ障害物があれば
// 停止して待つ (渋滞待ち) ノード。2D スキャンへの切り出しは行わない。
//
// 処理の流れ:
//  1. 点群から地面 (base_link 基準の高さ) と自車体を除き、障害物点を得る。
//  2. 障害物点を地図座標のセル (メモリ) に記録する。
//  3. プラン前方 stop_distance [m] 以内・中心線から path_clearance [m] 以内のセルが
//     min_blocking_cells 個以上あれば「塞がれている」とし、停止指令を出す。
//  4. 塞いでいるものが無くなってから resume_delay [s] 経過したら再開する。
//
// 死角の扱い:
//  YVT-35LX の垂直視野は -5°〜+35° のため、背の低い物 (他のロボットなど) は近づくと
//  視野の下に外れて点が出なくなる。見えなくなっただけで消すと、目の前の障害物を忘れて
//  走り出してしまうため、メモリのセルは
//    - 「その位置 (の最も高い点) が現在の視野内にあるのに observed_clear_time 以上点が出ない」
//      場合のみ消す (= 実際に居なくなったことを確認できた場合)
//    - 視野外 (死角) のセルは memory_timeout [s] で消す
//  こととする。
//
// 停止指令は cmdvel_stopper の /wizurg/obstacle_stop_cmd_vel (std_msgs/Bool) に一定周期で
// 出し続けるレベル方式。
//
// 通過モード (pass_through):
//  通路に張り出した枝葉など、障害物として扱いたくない物がある区間のための強制通過。
//  waypoint_manager は pass_through 属性のウェイポイントから次のウェイポイントまでの間、
//  pass_through_topic (std_msgs/Bool) に true を出し続ける (速度は waypoint_manager が slow_cmd_vel で制限)。
//  その間は
//    - グローバルプランの不感帯にあるセルをメモリから消す
//      (停止判定にも local_costmap にも入らなくなり、DWB も避けない)。
//      不感帯は、ロボットの進行方向から見て経路の中心線から
//        左へ path_clearance × pass_through_deadband_left_ratio
//        右へ path_clearance × pass_through_deadband_right_ratio
//      の範囲 (前方 pass_through_lookahead まで)。
//    - 不感帯の外側 pass_through_costmap_margin までのセルも local_costmap には入れない
//      (経路のすぐ脇のセルで DWB の軌道がすべて弾かれて動けなくなるのを防ぐ。停止判定には使う)
//    - 不感帯の外に残ったセルで通常どおり停止判定する (範囲は stop_distance × 左右 path_clearance)。
//      ただし「塞がれている」とするセル数は pass_through_min_blocking_cells を使う。
//      割合が 1.0 の側は、停止判定の範囲がなくなる。
//  pass_through_timeout 以上指令が途絶えたら通過モードを終了する。
//  不感帯の中では経路上の人も見えなくなるため、使う区間・不感帯は最小限にすること。
//
// 可視化 (RViz):
//  obstacle_monitor/markers (MarkerArray)
//    - cells     : 障害物セル (床から最も高い点までの柱)。
//                  橙 = 最近見えている, 紫 = 記憶のみ (死角など), 赤 = 停止判定の範囲 (検出帯) で検出,
//                  薄い黄 = 通過モードの不感帯で無視 (今回のフレームで消したセル)
//    - stop_zone : 停止判定の範囲 (プラン前方 stop_distance × 左右 path_clearance)。緑 = 空き, 赤 = 停止中
//                  通過モード中は不感帯を除いた左右の部分だけを描く
//    - pass_through_deadband : 通過モード中の不感帯 (黄色)
//    - status    : ロボット上の状態表示
//  obstacle_monitor/markers_2d (MarkerArray): vizanti 用。stop_zone・pass_through_deadband を LINE_STRIP で表したものと status
//  obstacle_monitor/cells_seen, cells_memory, cells_blocking (GridCells): vizanti 用の障害物セル。
//    markers の cells と同じ分類 (互いに重ならない)。vizanti の PointCloud は compression: none だと
//    data (base64 文字列) を読めないため GridCells で出す
//  vizanti 用のトピックは購読者がいるときだけ出す
//  obstacle_monitor/obstacle_cloud (PointCloud2): 今回のフレームで障害物とした点
//
// costmap について:
//  local_costmap : memory_cloud (記憶している障害物セル) を marking に、全ビーム +inf のスキャン
//                  (costmap_scan_topic) を clearing に使う。毎周期センサ周囲を消去してから
//                  記憶中のセルを書き込むため、DWB は obstacle_monitor と同じ障害物を見て避ける。
//  global_costmap: 障害物は入れない (グローバルプランは静的地図のみで決まり、迂回はしない)。
//                  ただし観測ソースが途絶えると Nav2 が costmap を not current とみなして
//                  プランニングが止まるため、空の点群を出し続ける。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

// 障害物メモリのセル (地図座標)
struct MemCell
{
  double x{0.0};      // セル内で最後に見えた点 (global frame)
  double y{0.0};
  double z_top{0.0};  // セル内で見えた最も高い点の z (global frame)。視野内判定に使う
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

struct Point3
{
  double x;
  double y;
  double z;
};

// 点 p と線分 ab の距離。t には線分上の射影位置 (0〜1) を返す
double distancePointToSegment(
  double px, double py, double ax, double ay, double bx, double by, double & t)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-9) {
    t = 0.0;
    return std::hypot(px - ax, py - ay);
  }
  t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

tf2::Transform toTf2(const geometry_msgs::msg::TransformStamped & ts)
{
  const auto & q = ts.transform.rotation;
  const auto & t = ts.transform.translation;
  tf2::Transform tf;
  tf.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
  tf.setOrigin(tf2::Vector3(t.x, t.y, t.z));
  return tf;
}

sensor_msgs::msg::PointCloud2 makeCloud(
  const std_msgs::msg::Header & header, const std::vector<Point3> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  for (const auto & p : points) {
    *it_x = static_cast<float>(p.x);
    *it_y = static_cast<float>(p.y);
    *it_z = static_cast<float>(p.z);
    ++it_x;
    ++it_y;
    ++it_z;
  }
  return cloud;
}

}  // namespace

class ObstacleMonitor : public rclcpp::Node
{
public:
  ObstacleMonitor()
  : Node("obstacle_monitor"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    steady_clock_(RCL_STEADY_TIME)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");

    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/hokuyo3d/hokuyo_cloud2");
    const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
    const auto stop_topic =
      declare_parameter<std::string>("stop_topic", "/wizurg/obstacle_stop_cmd_vel");

    // --- 障害物点の抽出 ---
    min_range_ = declare_parameter<double>("min_range", 0.3);
    max_range_ = declare_parameter<double>("max_range", 8.0);
    // 床の高さ (robot_base_frame 基準)。wizurg は base_link が床から 0.068 m 上
    ground_z_ = declare_parameter<double>("ground_z", -0.068);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", 0.10);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.5);
    // 自車体の除外範囲 (robot_base_frame 基準, footprint と同じ)
    footprint_min_x_ = declare_parameter<double>("footprint_min_x", -0.535);
    footprint_max_x_ = declare_parameter<double>("footprint_max_x", 0.195);
    footprint_half_width_ = declare_parameter<double>("footprint_half_width", 0.29);
    self_filter_margin_ = declare_parameter<double>("self_filter_margin", 0.05);

    // --- 障害物メモリ ---
    cell_size_ = declare_parameter<double>("cell_size", 0.1);
    observed_clear_time_ = declare_parameter<double>("observed_clear_time", 1.0);
    memory_timeout_ = declare_parameter<double>("memory_timeout", 8.0);
    sensor_min_elevation_deg_ = declare_parameter<double>("sensor_min_elevation_deg", -5.0);
    sensor_horizontal_fov_deg_ = declare_parameter<double>("sensor_horizontal_fov_deg", 210.0);
    // 視野の端ぎりぎりの点は見えたり見えなかったりするため、この角度だけ内側を視野とみなす
    visibility_margin_deg_ = declare_parameter<double>("visibility_margin_deg", 1.0);

    // --- 停止判定 ---
    stop_distance_ = declare_parameter<double>("stop_distance", 2.0);
    path_clearance_ = declare_parameter<double>("path_clearance", 0.45);
    min_blocking_cells_ = declare_parameter<int>("min_blocking_cells", 2);
    block_confirm_count_ = declare_parameter<int>("block_confirm_count", 2);
    resume_delay_ = declare_parameter<double>("resume_delay", 2.0);
    sensor_timeout_ = declare_parameter<double>("sensor_timeout", 0.5);

    // --- 通過モード ---
    const auto pass_through_topic =
      declare_parameter<std::string>("pass_through_topic", "/wizurg/pass_through");
    pass_through_lookahead_ = declare_parameter<double>("pass_through_lookahead", 6.0);
    pass_through_timeout_ = declare_parameter<double>("pass_through_timeout", 1.0);
    deadband_left_ratio_ = std::clamp(
      declare_parameter<double>("pass_through_deadband_left_ratio", 1.0), 0.0, 1.0);
    deadband_right_ratio_ = std::clamp(
      declare_parameter<double>("pass_through_deadband_right_ratio", 1.0), 0.0, 1.0);
    updateDeadband();
    pass_through_costmap_margin_ =
      std::max(0.0, declare_parameter<double>("pass_through_costmap_margin", 0.15));
    pass_through_min_blocking_cells_ =
      declare_parameter<int>("pass_through_min_blocking_cells", min_blocking_cells_);

    // --- costmap 用の出力 (現段階は中身なし) ---
    const auto costmap_marking_topic =
      declare_parameter<std::string>("costmap_marking_topic", "obstacle_monitor/static_cloud");
    const auto costmap_clearing_topic =
      declare_parameter<std::string>("costmap_clearing_topic", "obstacle_monitor/clearing_cloud");
    const auto costmap_scan_topic =
      declare_parameter<std::string>("costmap_scan_topic", "obstacle_monitor/filtered_scan");

    // 停止判定・通過モードのパラメータは実行中にも変更できる (GUI の通過モード設定から ros2 param で送る)
    param_cb_ = add_on_set_parameters_callback(
      std::bind(&ObstacleMonitor::onSetParameters, this, std::placeholders::_1));

    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleMonitor::cloudCallback, this, std::placeholders::_1));
    sub_pass_through_ = create_subscription<std_msgs::msg::Bool>(
      pass_through_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data != pass_through_) {
          RCLCPP_WARN(
            get_logger(), msg->data ?
            "通過モード開始: 経路付近の不感帯の障害物を無視します" : "通過モード終了");
        }
        pass_through_ = msg->data;
        pass_through_stamp_ = steady_clock_.now();
      });
    sub_plan_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic, rclcpp::QoS(1),
      std::bind(&ObstacleMonitor::planCallback, this, std::placeholders::_1));

    pub_stop_ = create_publisher<std_msgs::msg::Bool>(stop_topic, rclcpp::QoS(10));
    pub_status_ =
      create_publisher<std_msgs::msg::String>("obstacle_monitor/status", rclcpp::QoS(10));
    pub_obstacle_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_monitor/obstacle_cloud", rclcpp::SensorDataQoS());
    pub_memory_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_monitor/memory_cloud", rclcpp::SensorDataQoS());
    pub_blocking_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_monitor/blocking_cloud", rclcpp::SensorDataQoS());
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "obstacle_monitor/markers", rclcpp::QoS(1));
    pub_markers_2d_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "obstacle_monitor/markers_2d", rclcpp::QoS(1));
    pub_cells_seen_ =
      create_publisher<nav_msgs::msg::GridCells>("obstacle_monitor/cells_seen", rclcpp::QoS(1));
    pub_cells_memory_ =
      create_publisher<nav_msgs::msg::GridCells>("obstacle_monitor/cells_memory", rclcpp::QoS(1));
    pub_cells_blocking_ =
      create_publisher<nav_msgs::msg::GridCells>("obstacle_monitor/cells_blocking", rclcpp::QoS(1));
    pub_costmap_marking_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      costmap_marking_topic, rclcpp::SensorDataQoS());
    pub_costmap_clearing_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      costmap_clearing_topic, rclcpp::SensorDataQoS());
    pub_costmap_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(
      costmap_scan_topic, rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate)),
      std::bind(&ObstacleMonitor::publishState, this));

    RCLCPP_INFO(
      get_logger(), "obstacle_monitor started: cloud=%s plan=%s stop=%s",
      cloud_topic.c_str(), plan_topic.c_str(), stop_topic.c_str());
  }

private:
  // ---------------------------------------------------------------- callbacks

  void updateDeadband()
  {
    pass_through_deadband_left_ = path_clearance_ * deadband_left_ratio_;
    pass_through_deadband_right_ = path_clearance_ * deadband_right_ratio_;
  }

  // 実行中のパラメータ変更。対象外のパラメータは値の保存だけ行い、動作には反映しない (起動時のみ有効)
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto as_double = [](const rclcpp::Parameter & p) {
        return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
               static_cast<double>(p.as_int()) : p.as_double();
      };
    // 先に全部検査してから反映する (一部だけ反映されるのを防ぐ)
    for (const auto & p : params) {
      const auto & n = p.get_name();
      const auto type = p.get_type();
      const bool numeric = type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
        type == rclcpp::ParameterType::PARAMETER_INTEGER;
      if (n == "stop_distance" || n == "path_clearance" || n == "pass_through_lookahead" ||
        n == "pass_through_timeout" || n == "pass_through_costmap_margin" ||
        n == "pass_through_deadband_left_ratio" || n == "pass_through_deadband_right_ratio")
      {
        if (!numeric || as_double(p) < 0.0) {
          result.successful = false;
          result.reason = n + " は 0 以上の数値にしてください";
          return result;
        }
      } else if (n == "min_blocking_cells" || n == "pass_through_min_blocking_cells") {
        if (type != rclcpp::ParameterType::PARAMETER_INTEGER || p.as_int() < 1) {
          result.successful = false;
          result.reason = n + " は 1 以上の整数にしてください";
          return result;
        }
      }
    }
    for (const auto & p : params) {
      const auto & n = p.get_name();
      bool applied = true;
      if (n == "stop_distance") {
        stop_distance_ = as_double(p);
      } else if (n == "path_clearance") {
        path_clearance_ = as_double(p);
      } else if (n == "min_blocking_cells") {
        min_blocking_cells_ = static_cast<int>(p.as_int());
      } else if (n == "pass_through_lookahead") {
        pass_through_lookahead_ = as_double(p);
      } else if (n == "pass_through_timeout") {
        pass_through_timeout_ = as_double(p);
      } else if (n == "pass_through_costmap_margin") {
        pass_through_costmap_margin_ = as_double(p);
      } else if (n == "pass_through_deadband_left_ratio") {
        deadband_left_ratio_ = std::clamp(as_double(p), 0.0, 1.0);
      } else if (n == "pass_through_deadband_right_ratio") {
        deadband_right_ratio_ = std::clamp(as_double(p), 0.0, 1.0);
      } else if (n == "pass_through_min_blocking_cells") {
        pass_through_min_blocking_cells_ = static_cast<int>(p.as_int());
      } else {
        applied = false;
      }
      if (applied) {
        RCLCPP_INFO(get_logger(), "パラメータを変更しました: %s = %s",
          n.c_str(), p.value_to_string().c_str());
      }
    }
    updateDeadband();
    return result;
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<std::pair<double, double>> path;
    path.reserve(msg->poses.size());
    if (msg->header.frame_id == global_frame_ || msg->header.frame_id.empty()) {
      for (const auto & p : msg->poses) {
        path.emplace_back(p.pose.position.x, p.pose.position.y);
      }
    } else {
      geometry_msgs::msg::TransformStamped ts;
      try {
        ts = tf_buffer_->lookupTransform(
          global_frame_, msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "プランの座標変換に失敗: %s", ex.what());
        return;
      }
      const tf2::Transform tf = toTf2(ts);
      for (const auto & p : msg->poses) {
        const tf2::Vector3 v = tf * tf2::Vector3(p.pose.position.x, p.pose.position.y, 0.0);
        path.emplace_back(v.x(), v.y());
      }
    }
    path_ = std::move(path);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    tf2::Transform sensor_to_base;
    tf2::Transform sensor_to_global;
    try {
      sensor_to_base = toTf2(tf_buffer_->lookupTransform(
          robot_base_frame_, cloud->header.frame_id, cloud->header.stamp,
          tf2::durationFromSec(0.1)));
      sensor_to_global = toTf2(tf_buffer_->lookupTransform(
          global_frame_, cloud->header.frame_id, cloud->header.stamp,
          tf2::durationFromSec(0.1)));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "点群の座標変換に失敗: %s", ex.what());
      return;
    }
    const rclcpp::Time stamp(cloud->header.stamp, RCL_ROS_TIME);
    last_cloud_steady_ = steady_clock_.now();
    has_cloud_ = true;

    const std::vector<Point3> obstacles = extractObstacles(*cloud, sensor_to_base, sensor_to_global);
    updateMemory(obstacles, sensor_to_global, stamp);

    const tf2::Transform base_to_global = sensor_to_global * sensor_to_base.inverse();
    const tf2::Vector3 robot = base_to_global.getOrigin();
    std::vector<int64_t> blocking_keys;
    std::vector<std::pair<double, double>> deadband_zone;
    std::vector<Point3> ignored_cells;  // 不感帯で消したセル (可視化用)
    pass_through_now_ = passThroughActive();
    if (pass_through_now_) {
      // 通過モード: 不感帯のセルを消す。残ったセルで停止判定する
      deadband_zone = pathAround(robot.x(), robot.y(), 1.0, pass_through_lookahead_);
      ignored_cells = eraseCellsNearPath(
        deadband_zone, pass_through_deadband_left_, pass_through_deadband_right_);
    }
    std::vector<std::pair<double, double>> zone;
    const bool blocked = findBlockingCells(
      robot.x(), robot.y(), pass_through_now_ ? pass_through_min_blocking_cells_ : min_blocking_cells_,
      blocking_keys, zone);
    updateStopState(blocked, stamp);

    std_msgs::msg::Header global_header;
    global_header.stamp = cloud->header.stamp;
    global_header.frame_id = global_frame_;
    pub_obstacle_cloud_->publish(makeCloud(global_header, obstacles));
    std::vector<Point3> blocking_cells;
    for (const int64_t key : blocking_keys) {
      const MemCell & c = memory_.at(key);
      blocking_cells.push_back(Point3{c.x, c.y, c.z_top});
    }
    pub_blocking_cloud_->publish(makeCloud(global_header, blocking_cells));
    std::vector<Point3> memory_points;
    memory_points.reserve(memory_.size());
    for (const auto & kv : memory_) {
      // 通過モード中は、不感帯の外側 pass_through_costmap_margin までのセルも local_costmap に入れない
      // (経路のすぐ脇に残ったセルで DWB の軌道がすべて弾かれて動けなくなるのを防ぐ)
      if (pass_through_now_ && inPathBand(
          kv.second.x, kv.second.y, deadband_zone,
          pass_through_deadband_left_ + pass_through_costmap_margin_,
          pass_through_deadband_right_ + pass_through_costmap_margin_))
      {
        continue;
      }
      memory_points.push_back(Point3{kv.second.x, kv.second.y, kv.second.z_top});
    }
    pub_memory_cloud_->publish(makeCloud(global_header, memory_points));

    if (publish_markers_) {
      publishMarkers(global_header, stamp, robot, blocking_keys, zone, deadband_zone, ignored_cells);
      publishGridCells(global_header, stamp, blocking_keys);
    }
    publishCostmapPlaceholders(cloud->header);
  }

  // ------------------------------------------------------------------ helpers

  // 地面・自車体・範囲外を除いた障害物点 (global frame) を返す
  std::vector<Point3> extractObstacles(
    const sensor_msgs::msg::PointCloud2 & cloud, const tf2::Transform & sensor_to_base,
    const tf2::Transform & sensor_to_global) const
  {
    std::vector<Point3> out;
    const double self_min_x = footprint_min_x_ - self_filter_margin_;
    const double self_max_x = footprint_max_x_ + self_filter_margin_;
    const double self_half_w = footprint_half_width_ + self_filter_margin_;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
        continue;
      }
      const tf2::Vector3 ps(*it_x, *it_y, *it_z);
      const double r = std::hypot(ps.x(), ps.y());
      if (r < min_range_ || r > max_range_) {
        continue;
      }
      const tf2::Vector3 pb = sensor_to_base * ps;
      const double height = pb.z() - ground_z_;
      if (height < min_obstacle_height_ || height > max_obstacle_height_) {
        continue;
      }
      if (pb.x() >= self_min_x && pb.x() <= self_max_x && std::abs(pb.y()) <= self_half_w) {
        continue;
      }
      const tf2::Vector3 pg = sensor_to_global * ps;
      out.push_back(Point3{pg.x(), pg.y(), pg.z()});
    }
    return out;
  }

  static int64_t cellKey(int ix, int iy)
  {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(iy));
  }

  int cellIndex(double v) const
  {
    return static_cast<int>(std::floor(v / cell_size_));
  }

  // セルの最も高い点が、いまセンサの視野内にあるか (= 居なくなったことを確認できる位置か)
  bool isObservable(const MemCell & c, const tf2::Transform & global_to_sensor) const
  {
    const tf2::Vector3 q = global_to_sensor * tf2::Vector3(c.x, c.y, c.z_top);
    const double d = std::hypot(q.x(), q.y());
    if (d < min_range_ || d > max_range_) {
      return false;
    }
    const double margin = visibility_margin_deg_ * M_PI / 180.0;
    const double elevation = std::atan2(q.z(), d);
    if (elevation < sensor_min_elevation_deg_ * M_PI / 180.0 + margin) {
      return false;
    }
    const double azimuth = std::atan2(q.y(), q.x());
    return std::abs(azimuth) <= 0.5 * sensor_horizontal_fov_deg_ * M_PI / 180.0 - margin;
  }

  void updateMemory(
    const std::vector<Point3> & obstacles, const tf2::Transform & sensor_to_global,
    const rclcpp::Time & stamp)
  {
    for (const auto & p : obstacles) {
      MemCell & c = memory_[cellKey(cellIndex(p.x), cellIndex(p.y))];
      // 久しぶりに見えたセルは高さを測り直す (別の物が来た可能性があるため)
      const bool fresh = c.last_seen.nanoseconds() > 0 &&
        (stamp - c.last_seen).seconds() < observed_clear_time_;
      c.z_top = fresh ? std::max(c.z_top, p.z) : p.z;
      c.x = p.x;
      c.y = p.y;
      c.last_seen = stamp;
    }

    const tf2::Transform global_to_sensor = sensor_to_global.inverse();
    for (auto it = memory_.begin(); it != memory_.end(); ) {
      const double age = (stamp - it->second.last_seen).seconds();
      const bool expired = age > memory_timeout_;
      const bool confirmed_gone =
        age > observed_clear_time_ && isObservable(it->second, global_to_sensor);
      if (expired || confirmed_gone) {
        it = memory_.erase(it);
      } else {
        ++it;
      }
    }
  }

  bool passThroughActive()
  {
    if (!pass_through_) {
      return false;
    }
    if ((steady_clock_.now() - pass_through_stamp_).seconds() > pass_through_timeout_) {
      RCLCPP_WARN(get_logger(), "通過モード指令が途絶えたため通過モードを終了します");
      pass_through_ = false;
      return false;
    }
    return true;
  }

  // ロボットに最も近いプラン上の点から、後方 back [m]・前方 ahead [m] の区間を切り出す
  std::vector<std::pair<double, double>> pathAround(
    double rx, double ry, double back, double ahead) const
  {
    std::vector<std::pair<double, double>> segs;
    if (path_.size() < 2) {
      return segs;
    }
    size_t start = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_.size(); ++i) {
      const double d = std::hypot(path_[i].first - rx, path_[i].second - ry);
      if (d < best) {
        best = d;
        start = i;
      }
    }
    size_t first = start;
    for (double arc = 0.0; first > 0 && arc < back; --first) {
      arc += std::hypot(
        path_[first].first - path_[first - 1].first, path_[first].second - path_[first - 1].second);
    }
    segs.push_back(path_[first]);
    double arc = 0.0;
    for (size_t i = first + 1; i < path_.size() && arc <= back + ahead; ++i) {
      arc += std::hypot(path_[i].first - path_[i - 1].first, path_[i].second - path_[i - 1].second);
      segs.push_back(path_[i]);
    }
    return segs;
  }

  // 点 (px, py) が、プラン区間 segs の中心線から進行方向に向かって左 left_width・右 right_width
  // 以内にあるか。左右は最も近い区間の向きで決める
  static bool inPathBand(
    double px, double py, const std::vector<std::pair<double, double>> & segs,
    double left_width, double right_width)
  {
    if (segs.size() < 2) {
      return false;
    }
    double best = std::numeric_limits<double>::max();
    double cross = 0.0;  // > 0 なら進行方向の左
    for (size_t i = 0; i + 1 < segs.size(); ++i) {
      const auto & a = segs[i];
      const auto & b = segs[i + 1];
      double t = 0.0;
      const double d = distancePointToSegment(px, py, a.first, a.second, b.first, b.second, t);
      if (d < best) {
        best = d;
        cross = (b.first - a.first) * (py - a.second) - (b.second - a.second) * (px - a.first);
      }
    }
    return best <= (cross >= 0.0 ? left_width : right_width);
  }

  // プラン区間 segs の中心線から、進行方向に向かって左 left_width・右 right_width 以内にある
  // メモリのセルを消し、消したセルを返す (可視化用)
  std::vector<Point3> eraseCellsNearPath(
    const std::vector<std::pair<double, double>> & segs, double left_width, double right_width)
  {
    std::vector<Point3> erased;
    for (auto it = memory_.begin(); it != memory_.end(); ) {
      if (inPathBand(it->second.x, it->second.y, segs, left_width, right_width)) {
        erased.push_back(Point3{it->second.x, it->second.y, it->second.z_top});
        it = memory_.erase(it);
      } else {
        ++it;
      }
    }
    return erased;
  }

  // プラン前方 stop_distance 以内・中心線から path_clearance 以内にあるメモリのセルを集め、
  // min_cells 個以上あれば true を返す。zone には判定に使ったプランの区間を返す (可視化用)
  bool findBlockingCells(
    double rx, double ry, int min_cells, std::vector<int64_t> & blocking,
    std::vector<std::pair<double, double>> & zone) const
  {
    blocking.clear();
    zone.clear();
    if (path_.size() < 2) {
      return false;
    }

    // ロボットに最も近いプラン上の点から前方のみを対象にする
    size_t start = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_.size(); ++i) {
      const double d = std::hypot(path_[i].first - rx, path_[i].second - ry);
      if (d < best) {
        best = d;
        start = i;
      }
    }
    std::vector<std::pair<double, double>> segs{path_[start]};
    std::vector<double> seg_arc{0.0};  // 各点までの経路長
    for (size_t i = start + 1; i < path_.size() && seg_arc.back() <= stop_distance_; ++i) {
      seg_arc.push_back(
        seg_arc.back() +
        std::hypot(path_[i].first - path_[i - 1].first, path_[i].second - path_[i - 1].second));
      segs.push_back(path_[i]);
    }
    if (segs.size() < 2) {
      return false;
    }
    zone = segs;

    const double reach = seg_arc.back() + path_clearance_;
    for (const auto & kv : memory_) {
      const MemCell & c = kv.second;
      if (std::hypot(c.x - segs.front().first, c.y - segs.front().second) > reach) {
        continue;
      }
      for (size_t i = 0; i + 1 < segs.size(); ++i) {
        double t = 0.0;
        const double d = distancePointToSegment(
          c.x, c.y, segs[i].first, segs[i].second, segs[i + 1].first, segs[i + 1].second, t);
        const double arc = seg_arc[i] + t * (seg_arc[i + 1] - seg_arc[i]);
        if (d <= path_clearance_ && arc <= stop_distance_) {
          blocking.push_back(kv.first);
          break;
        }
      }
    }
    return static_cast<int>(blocking.size()) >= min_cells;
  }

  void updateStopState(bool blocked, const rclcpp::Time & stamp)
  {
    if (blocked) {
      block_streak_++;
      if (block_streak_ >= block_confirm_count_) {
        last_block_time_ = stamp;
        if (!stop_) {
          RCLCPP_INFO(get_logger(), "経路上に障害物: 停止して待ちます");
        }
        stop_ = true;
      }
      return;
    }
    block_streak_ = 0;
    if (stop_ && (stamp - last_block_time_).seconds() >= resume_delay_) {
      RCLCPP_INFO(get_logger(), "経路が空きました: 走行を再開します");
      stop_ = false;
    }
  }

  // 床から高さ z_top までの柱を 0.05 m 刻みで積む (高さが分かるように)
  void addColumn(
    visualization_msgs::msg::Marker & cells, double x, double y, double z_top, double floor_z,
    const std_msgs::msg::ColorRGBA & col) const
  {
    const int layers = std::max(1, static_cast<int>(std::ceil((z_top - floor_z) / 0.05)));
    for (int k = 0; k < std::min(layers, 40); ++k) {
      geometry_msgs::msg::Point p;
      p.x = cellIndex(x) * cell_size_ + 0.5 * cell_size_;
      p.y = cellIndex(y) * cell_size_ + 0.5 * cell_size_;
      p.z = floor_z + 0.025 + 0.05 * k;
      cells.points.push_back(p);
      cells.colors.push_back(col);
    }
  }

  void publishMarkers(
    const std_msgs::msg::Header & header, const rclcpp::Time & stamp, const tf2::Vector3 & robot,
    const std::vector<int64_t> & blocking_keys, const std::vector<std::pair<double, double>> & zone,
    const std::vector<std::pair<double, double>> & deadband_zone,
    const std::vector<Point3> & ignored_cells)
  {
    const auto lifetime = rclcpp::Duration::from_seconds(0.5);
    const std::set<int64_t> blocking(blocking_keys.begin(), blocking_keys.end());
    // 床面 (global frame)。ロボットの床面の高さで描く
    const double floor_z = robot.z() + ground_z_;
    visualization_msgs::msg::MarkerArray arr;

    auto color = [](float r, float g, float b, float a) {
        std_msgs::msg::ColorRGBA c;
        c.r = r; c.g = g; c.b = b; c.a = a;
        return c;
      };

    // 障害物セル: 床から最も高い点までの柱
    visualization_msgs::msg::Marker cells;
    cells.header = header;
    cells.ns = "cells";
    cells.id = 0;
    cells.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cells.action = visualization_msgs::msg::Marker::ADD;
    cells.pose.orientation.w = 1.0;
    cells.scale.x = cell_size_;
    cells.scale.y = cell_size_;
    cells.scale.z = 0.05;
    cells.lifetime = lifetime;
    for (const auto & kv : memory_) {
      const MemCell & c = kv.second;
      std_msgs::msg::ColorRGBA col;
      if (blocking.count(kv.first)) {
        col = color(1.0f, 0.1f, 0.1f, 0.9f);  // 経路を塞いでいる
      } else if ((stamp - c.last_seen).seconds() <= observed_clear_time_) {
        // 今見えている (点群はまばらなので observed_clear_time 以内に見えたものを含める)
        col = color(1.0f, 0.6f, 0.0f, 0.8f);
      } else {
        col = color(0.7f, 0.3f, 1.0f, 0.8f);  // 記憶のみ (死角など)
      }
      addColumn(cells, c.x, c.y, c.z_top, floor_z, col);
    }
    for (const auto & c : ignored_cells) {
      addColumn(cells, c.x, c.y, c.z, floor_z, color(1.0f, 0.95f, 0.3f, 0.5f));  // 不感帯で無視
    }
    if (cells.points.empty()) {
      cells.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back(cells);

    // プラン区間 segs の中心線から、進行方向の左を正とした横方向の距離 lo〜hi の帯
    auto make_band = [&](
      const std::vector<std::pair<double, double>> & segs, double lo, double hi,
      const std::string & ns, int id, const std_msgs::msg::ColorRGBA & col) {
        visualization_msgs::msg::Marker band;
        band.header = header;
        band.ns = ns;
        band.id = id;
        band.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        band.action = visualization_msgs::msg::Marker::ADD;
        band.pose.orientation.w = 1.0;
        band.scale.x = band.scale.y = band.scale.z = 1.0;
        band.color = col;
        band.lifetime = lifetime;
        for (size_t i = 0; hi > lo && i + 1 < segs.size(); ++i) {
          const double dx = segs[i + 1].first - segs[i].first;
          const double dy = segs[i + 1].second - segs[i].second;
          const double len = std::hypot(dx, dy);
          if (len < 1e-6) {
            continue;
          }
          const double nx = -dy / len;  // 進行方向の左向きの単位ベクトル
          const double ny = dx / len;
          auto pt = [&](size_t k, double w) {
              geometry_msgs::msg::Point p;
              p.x = segs[k].first + nx * w;
              p.y = segs[k].second + ny * w;
              p.z = floor_z + 0.02;
              return p;
            };
          const auto al = pt(i, hi);
          const auto ar = pt(i, lo);
          const auto bl = pt(i + 1, hi);
          const auto br = pt(i + 1, lo);
          band.points.insert(band.points.end(), {al, ar, bl, ar, br, bl});
        }
        if (band.points.empty()) {
          band.action = visualization_msgs::msg::Marker::DELETE;
        }
        return band;
      };

    // 停止判定の範囲 (検出帯): プラン区間を左右に path_clearance だけ広げた帯。
    // 通過モード中は不感帯を除いた左右の部分だけを描く
    const auto zone_color =
      stop_ ? color(1.0f, 0.1f, 0.1f, 0.35f) : color(0.1f, 0.9f, 0.3f, 0.25f);
    const double pc = path_clearance_;
    std::vector<std::array<double, 2>> detect_ranges;  // 検出帯の横方向の範囲 {lo, hi}
    if (pass_through_now_) {
      detect_ranges.push_back({std::min(pass_through_deadband_left_, pc), pc});
      detect_ranges.push_back({-pc, -std::min(pass_through_deadband_right_, pc)});
    } else {
      detect_ranges.push_back({-pc, pc});
      detect_ranges.push_back({0.0, 0.0});  // 通過モードで使う id 1 を消す
    }
    std::vector<visualization_msgs::msg::Marker> zone_bands;
    for (size_t k = 0; k < detect_ranges.size(); ++k) {
      zone_bands.push_back(make_band(
          zone, detect_ranges[k][0], detect_ranges[k][1], "stop_zone", static_cast<int>(k),
          zone_color));
      arr.markers.push_back(zone_bands.back());
    }
    // 通過モードの不感帯
    const auto deadband = make_band(
      pass_through_now_ ? deadband_zone : std::vector<std::pair<double, double>>{},
      -pass_through_deadband_right_, pass_through_deadband_left_, "pass_through_deadband", 0,
      color(1.0f, 0.85f, 0.0f, 0.3f));
    arr.markers.push_back(deadband);

    // 状態表示 (RViz は日本語を描けないため英語)
    visualization_msgs::msg::Marker text;
    text.header = header;
    text.ns = "status";
    text.id = 0;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = robot.x();
    text.pose.position.y = robot.y();
    text.pose.position.z = floor_z + 1.6;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.3;
    text.color = stop_ ? color(1.0f, 0.2f, 0.2f, 1.0f) :
      (pass_through_now_ ? color(1.0f, 0.85f, 0.0f, 1.0f) : color(0.3f, 1.0f, 0.4f, 1.0f));
    text.text = std::string(pass_through_now_ ? "PASS_THROUGH: " : "") + (stop_ ?
      "WAIT_BLOCKED (" + std::to_string(blocking_keys.size()) + " cells)" : "CLEAR");
    text.lifetime = lifetime;
    arr.markers.push_back(text);

    pub_markers_->publish(arr);

    // vizanti 用 (2D)。vizanti は TRIANGLE_LIST を描けず CUBE_LIST も位置がずれるため、
    // 停止判定の範囲は線幅 (scale.x) つきの LINE_STRIP で表し、状態表示と合わせて送る。
    // 障害物セルは obstacle_monitor/memory_cloud, blocking_cloud を vizanti の PointCloud で表示する。
    // 購読者がいないときは送らない。
    if (pub_markers_2d_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray arr2d;
      // 帯 (横方向 lo〜hi) を、中心線を左へ (lo + hi) / 2 ずらした線幅 hi - lo の線で表す
      auto make_line = [&](
        const visualization_msgs::msg::Marker & band_marker,
        const std::vector<std::pair<double, double>> & segs, double lo, double hi) {
          visualization_msgs::msg::Marker line = band_marker;  // header, ns, id, 色, lifetime を引き継ぐ
          line.type = visualization_msgs::msg::Marker::LINE_STRIP;
          line.scale.x = hi - lo;
          line.points.clear();
          const double offset = 0.5 * (lo + hi);
          for (size_t k = 0; k < segs.size() && segs.size() >= 2; ++k) {
            const size_t a = k + 1 < segs.size() ? k : k - 1;
            const double dx = segs[a + 1].first - segs[a].first;
            const double dy = segs[a + 1].second - segs[a].second;
            const double len = std::max(1e-6, std::hypot(dx, dy));
            geometry_msgs::msg::Point p;
            p.x = segs[k].first - dy / len * offset;
            p.y = segs[k].second + dx / len * offset;
            p.z = floor_z + 0.02;
            line.points.push_back(p);
          }
          if (line.points.size() < 2 || band_marker.action == visualization_msgs::msg::Marker::DELETE) {
            line.action = visualization_msgs::msg::Marker::DELETE;
          }
          return line;
        };
      for (size_t k = 0; k < zone_bands.size(); ++k) {
        arr2d.markers.push_back(
          make_line(zone_bands[k], zone, detect_ranges[k][0], detect_ranges[k][1]));
      }
      arr2d.markers.push_back(make_line(
          deadband, pass_through_now_ ? deadband_zone : std::vector<std::pair<double, double>>{},
          -pass_through_deadband_right_, pass_through_deadband_left_));
      arr2d.markers.push_back(text);
      pub_markers_2d_->publish(arr2d);
    }
  }

  // vizanti 用の障害物セル (markers の cells と同じ色分けを 3 トピックに分ける)
  void publishGridCells(
    const std_msgs::msg::Header & header, const rclcpp::Time & stamp,
    const std::vector<int64_t> & blocking_keys)
  {
    const bool want_seen = pub_cells_seen_->get_subscription_count() > 0;
    const bool want_memory = pub_cells_memory_->get_subscription_count() > 0;
    const bool want_blocking = pub_cells_blocking_->get_subscription_count() > 0;
    if (!want_seen && !want_memory && !want_blocking) {
      return;
    }
    const std::set<int64_t> blocking(blocking_keys.begin(), blocking_keys.end());
    nav_msgs::msg::GridCells seen, memory, blocked;
    for (auto * g : {&seen, &memory, &blocked}) {
      g->header = header;
      g->cell_width = cell_size_;
      g->cell_height = cell_size_;
    }
    for (const auto & kv : memory_) {
      const MemCell & c = kv.second;
      geometry_msgs::msg::Point p;
      p.x = cellIndex(c.x) * cell_size_ + 0.5 * cell_size_;
      p.y = cellIndex(c.y) * cell_size_ + 0.5 * cell_size_;
      if (blocking.count(kv.first)) {
        blocked.cells.push_back(p);
      } else if ((stamp - c.last_seen).seconds() <= observed_clear_time_) {
        seen.cells.push_back(p);
      } else {
        memory.cells.push_back(p);
      }
    }
    if (want_seen) {pub_cells_seen_->publish(seen);}
    if (want_memory) {pub_cells_memory_->publish(memory);}
    if (want_blocking) {pub_cells_blocking_->publish(blocked);}
  }

  // global_costmap 用の空の点群と、local_costmap 消去用の全ビーム +inf スキャン
  void publishCostmapPlaceholders(const std_msgs::msg::Header & header)
  {
    const std::vector<Point3> empty;
    pub_costmap_marking_->publish(makeCloud(header, empty));
    pub_costmap_clearing_->publish(makeCloud(header, empty));

    sensor_msgs::msg::LaserScan scan;
    scan.header = header;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    // 遠方でも消去レイの間に隙間ができないよう 0.5 度刻み (8 m 先で約 7 cm 間隔)
    scan.angle_increment = M_PI / 360.0;
    scan.range_min = static_cast<float>(min_range_);
    scan.range_max = static_cast<float>(max_range_);
    scan.ranges.assign(720, std::numeric_limits<float>::infinity());
    pub_costmap_scan_->publish(scan);
  }

  // 停止指令と状態を一定周期で出力する
  void publishState()
  {
    const bool sensor_lost = !path_.empty() &&
      (!has_cloud_ || (steady_clock_.now() - last_cloud_steady_).seconds() > sensor_timeout_);
    if (sensor_lost) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "点群が届いていないため停止します");
    }

    std_msgs::msg::Bool stop_msg;
    stop_msg.data = stop_ || sensor_lost;
    pub_stop_->publish(stop_msg);

    std_msgs::msg::String status;
    status.data = sensor_lost ? "NO_SENSOR" :
      (stop_ ? "WAIT_BLOCKED" : (passThroughActive() ? "PASS_THROUGH" : "CLEAR"));
    pub_status_->publish(status);
  }

  // ---------------------------------------------------------------- members

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Clock steady_clock_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pass_through_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_memory_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_blocking_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_2d_;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr pub_cells_seen_;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr pub_cells_memory_;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr pub_cells_blocking_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_costmap_marking_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_costmap_clearing_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_costmap_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::string global_frame_;
  std::string robot_base_frame_;

  double min_range_;
  double max_range_;
  double ground_z_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double footprint_min_x_;
  double footprint_max_x_;
  double footprint_half_width_;
  double self_filter_margin_;
  double cell_size_;
  double observed_clear_time_;
  double memory_timeout_;
  double sensor_min_elevation_deg_;
  double sensor_horizontal_fov_deg_;
  double visibility_margin_deg_;
  double stop_distance_;
  double path_clearance_;
  int min_blocking_cells_;
  int block_confirm_count_;
  double resume_delay_;
  double sensor_timeout_;
  double pass_through_lookahead_;
  double pass_through_timeout_;
  double deadband_left_ratio_;   // 不感帯の割合 (path_clearance に対する)
  double deadband_right_ratio_;
  double pass_through_deadband_left_;   // 不感帯の幅 (進行方向の左) [m]
  double pass_through_deadband_right_;  // 不感帯の幅 (進行方向の右) [m]
  int pass_through_min_blocking_cells_;
  double pass_through_costmap_margin_;  // 不感帯の外側で local_costmap に入れない幅 [m]
  bool pass_through_{false};
  bool pass_through_now_{false};  // 直近の点群処理時に通過モードだったか (可視化用)
  rclcpp::Time pass_through_stamp_{0, 0, RCL_STEADY_TIME};
  bool publish_markers_;

  std::vector<std::pair<double, double>> path_;
  std::unordered_map<int64_t, MemCell> memory_;

  bool stop_{false};
  int block_streak_{0};
  rclcpp::Time last_block_time_{0, 0, RCL_ROS_TIME};
  bool has_cloud_{false};
  rclcpp::Time last_cloud_steady_{0, 0, RCL_STEADY_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleMonitor>());
  rclcpp::shutdown();
  return 0;
}
