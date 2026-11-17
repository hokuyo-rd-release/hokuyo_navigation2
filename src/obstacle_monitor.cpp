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
//    - 停止指令を出さない
//    - グローバルプランから pass_through_corridor 以内のセルをメモリから消す
//      (local_costmap にも入らなくなり、DWB も避けない)
//  pass_through_timeout 以上指令が途絶えたら通過モードを終了する。
//  区間内では経路上の人も見えなくなるため、使う区間は最小限にすること。
//
// 可視化 (RViz):
//  obstacle_monitor/markers (MarkerArray)
//    - cells     : 障害物セル (床から最も高い点までの柱)。
//                  橙 = 最近見えている, 紫 = 記憶のみ (死角など), 赤 = 経路を塞いでいる
//    - stop_zone : 停止判定の範囲 (プラン前方 stop_distance × 左右 path_clearance)。緑 = 空き, 赤 = 停止中
//                  通過モード中は無視する範囲 (左右 pass_through_corridor) を黄色で表示
//    - status    : ロボット上の状態表示
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
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

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
    pass_through_corridor_ = declare_parameter<double>("pass_through_corridor", 0.6);
    pass_through_lookahead_ = declare_parameter<double>("pass_through_lookahead", 6.0);
    pass_through_timeout_ = declare_parameter<double>("pass_through_timeout", 1.0);

    // --- costmap 用の出力 (現段階は中身なし) ---
    const auto costmap_marking_topic =
      declare_parameter<std::string>("costmap_marking_topic", "obstacle_monitor/static_cloud");
    const auto costmap_clearing_topic =
      declare_parameter<std::string>("costmap_clearing_topic", "obstacle_monitor/clearing_cloud");
    const auto costmap_scan_topic =
      declare_parameter<std::string>("costmap_scan_topic", "obstacle_monitor/filtered_scan");

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
            "通過モード開始: 経路付近の障害物を無視します" : "通過モード終了");
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
    std::vector<std::pair<double, double>> zone;
    pass_through_now_ = passThroughActive();
    if (pass_through_now_) {
      // 通過モード: 経路付近のセルを消し、停止判定もしない
      zone = pathAround(robot.x(), robot.y(), 1.0, pass_through_lookahead_);
      eraseCellsNearPath(zone, pass_through_corridor_);
      if (stop_) {
        RCLCPP_INFO(get_logger(), "通過モード: 停止を解除します");
      }
      stop_ = false;
      block_streak_ = 0;
    } else {
      const bool blocked = findBlockingCells(robot.x(), robot.y(), blocking_keys, zone);
      updateStopState(blocked, stamp);
    }

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
      memory_points.push_back(Point3{kv.second.x, kv.second.y, kv.second.z_top});
    }
    pub_memory_cloud_->publish(makeCloud(global_header, memory_points));

    if (publish_markers_) {
      publishMarkers(global_header, stamp, robot, blocking_keys, zone);
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

  // プラン区間 segs から half_width 以内にあるメモリのセルを消す
  void eraseCellsNearPath(const std::vector<std::pair<double, double>> & segs, double half_width)
  {
    if (segs.size() < 2) {
      return;
    }
    for (auto it = memory_.begin(); it != memory_.end(); ) {
      bool near = false;
      for (size_t i = 0; i + 1 < segs.size() && !near; ++i) {
        double t = 0.0;
        near = distancePointToSegment(
          it->second.x, it->second.y, segs[i].first, segs[i].second,
          segs[i + 1].first, segs[i + 1].second, t) <= half_width;
      }
      it = near ? memory_.erase(it) : std::next(it);
    }
  }

  // プラン前方 stop_distance 以内・中心線から path_clearance 以内にあるメモリのセルを集める。
  // zone には判定に使ったプランの区間を返す (可視化用)
  bool findBlockingCells(
    double rx, double ry, std::vector<int64_t> & blocking,
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
    return static_cast<int>(blocking.size()) >= min_blocking_cells_;
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

  void publishMarkers(
    const std_msgs::msg::Header & header, const rclcpp::Time & stamp, const tf2::Vector3 & robot,
    const std::vector<int64_t> & blocking_keys, const std::vector<std::pair<double, double>> & zone)
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
      // 高さが分かるよう、床から最も高い点まで 0.05 m 刻みで積む
      const int layers = std::max(1, static_cast<int>(std::ceil((c.z_top - floor_z) / 0.05)));
      for (int k = 0; k < std::min(layers, 40); ++k) {
        geometry_msgs::msg::Point p;
        p.x = cellIndex(c.x) * cell_size_ + 0.5 * cell_size_;
        p.y = cellIndex(c.y) * cell_size_ + 0.5 * cell_size_;
        p.z = floor_z + 0.025 + 0.05 * k;
        cells.points.push_back(p);
        cells.colors.push_back(col);
      }
    }
    if (cells.points.empty()) {
      cells.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back(cells);

    // 停止判定の範囲: プラン区間を左右に path_clearance だけ広げた帯
    visualization_msgs::msg::Marker band;
    band.header = header;
    band.ns = "stop_zone";
    band.id = 0;
    band.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    band.action = visualization_msgs::msg::Marker::ADD;
    band.pose.orientation.w = 1.0;
    band.scale.x = band.scale.y = band.scale.z = 1.0;
    band.color = pass_through_now_ ? color(1.0f, 0.85f, 0.0f, 0.3f) :
      (stop_ ? color(1.0f, 0.1f, 0.1f, 0.35f) : color(0.1f, 0.9f, 0.3f, 0.25f));
    const double band_half_width = pass_through_now_ ? pass_through_corridor_ : path_clearance_;
    band.lifetime = lifetime;
    for (size_t i = 0; i + 1 < zone.size(); ++i) {
      const double dx = zone[i + 1].first - zone[i].first;
      const double dy = zone[i + 1].second - zone[i].second;
      const double len = std::hypot(dx, dy);
      if (len < 1e-6) {
        continue;
      }
      const double nx = -dy / len * band_half_width;
      const double ny = dx / len * band_half_width;
      auto pt = [&](double x, double y) {
          geometry_msgs::msg::Point p;
          p.x = x; p.y = y; p.z = floor_z + 0.02;
          return p;
        };
      const auto al = pt(zone[i].first + nx, zone[i].second + ny);
      const auto ar = pt(zone[i].first - nx, zone[i].second - ny);
      const auto bl = pt(zone[i + 1].first + nx, zone[i + 1].second + ny);
      const auto br = pt(zone[i + 1].first - nx, zone[i + 1].second - ny);
      band.points.insert(band.points.end(), {al, ar, bl, ar, br, bl});
    }
    if (band.points.empty()) {
      band.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back(band);

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
    text.color = pass_through_now_ ? color(1.0f, 0.85f, 0.0f, 1.0f) :
      (stop_ ? color(1.0f, 0.2f, 0.2f, 1.0f) : color(0.3f, 1.0f, 0.4f, 1.0f));
    text.text = pass_through_now_ ? "PASS_THROUGH" : stop_ ?
      "WAIT_BLOCKED (" + std::to_string(blocking_keys.size()) + " cells)" : "CLEAR";
    text.lifetime = lifetime;
    arr.markers.push_back(text);

    pub_markers_->publish(arr);
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
      (passThroughActive() ? "PASS_THROUGH" : (stop_ ? "WAIT_BLOCKED" : "CLEAR"));
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_costmap_marking_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_costmap_clearing_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_costmap_scan_;
  rclcpp::TimerBase::SharedPtr timer_;

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
  double pass_through_corridor_;
  double pass_through_lookahead_;
  double pass_through_timeout_;
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
