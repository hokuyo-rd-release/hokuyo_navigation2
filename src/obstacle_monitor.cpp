// obstacle_monitor
//
// LaserScan をクラスタリング・追跡し、障害物を「静止(static)」と「移動(dynamic)」に分類する。
//
//  - 静止障害物 : 点群を static_cloud として出力する。global_costmap の obstacle_layer が
//                 これを marking ソースとして取り込むため、グローバルプランナが
//                 ロボットサイズ(footprint + inflation)を考慮して自動的に回避経路を再生成する。
//  - 移動障害物 : global_costmap には一切書き込まない(= グローバルプランは変化しない)。
//                 代わりにグローバルプラン上を塞いでいる間だけ停止指令を出し、
//                 通過後は元のグローバルプランのまま走行を再開する。
//
// 停止指令は cmdvel_stopper の /wizurg/obstacle_stop_cmd_vel (std_msgs/Bool) に
// 一定周期で出し続けるレベル方式。waypoint_manager が使う stop/start_cmd_vel とは
// 独立したチャンネルなので、両者が互いの停止指令を打ち消すことはない。

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

struct Point2D
{
  double x;  // global frame
  double y;
  double lx;  // sensor frame
  double ly;
};

struct Cluster
{
  std::vector<Point2D> points;
  double cx{0.0};
  double cy{0.0};
  double radius{0.0};
  int track_id{-1};
};

struct Sample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double x{0.0};
  double y{0.0};
};

struct Track
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.0};
  rclcpp::Time first_seen{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_moving_time{0, 0, RCL_ROS_TIME};
  // 一定時間ぶんの重心位置。瞬間速度ではなく「窓内の正味移動量」で移動判定するために使う。
  std::deque<Sample> history;
  int moving_streak{0};
  bool is_dynamic{false};
  bool matched{false};
};

// 静止と確定した障害物の位置の記憶。クラスタが分裂・結合してトラックが作り直された際に
// 静止判定をやり直さないようにするために使う。
struct StaticMemo
{
  double x{0.0};
  double y{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

// 点 p と線分 ab の距離
double distancePointToSegment(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-9) {
    return std::hypot(px - ax, py - ay);
  }
  double t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

}  // namespace

class ObstacleMonitor : public rclcpp::Node
{
public:
  ObstacleMonitor()
  : Node("obstacle_monitor"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");

    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/hokuyo3d/scan");
    const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
    const auto static_cloud_topic =
      declare_parameter<std::string>("static_cloud_topic", "obstacle_monitor/static_cloud");
    const auto clearing_cloud_topic =
      declare_parameter<std::string>("clearing_cloud_topic", "obstacle_monitor/clearing_cloud");
    const auto stop_topic =
      declare_parameter<std::string>("stop_topic", "/wizurg/obstacle_stop_cmd_vel");
    const auto slow_topic =
      declare_parameter<std::string>("slow_topic", "/wizurg/obstacle_slow_cmd_vel");

    // 検出範囲
    min_detection_range_ = declare_parameter<double>("min_detection_range", 0.3);
    max_detection_range_ = declare_parameter<double>("max_detection_range", 8.0);
    clearing_range_ = declare_parameter<double>("clearing_range", 8.0);
    // 消去レイを計測距離より手前で止める量。
    // 0 にすると自分自身の反射でマーキング済みセルを毎スキャン消してしまい、
    // グローバルコストマップ上の障害物が点滅してプランが安定しない。
    clearing_shrink_ = declare_parameter<double>("clearing_shrink", 0.25);

    // クラスタリング
    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.35);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 2);
    max_beam_gap_ = declare_parameter<int>("max_beam_gap", 2);

    // 追跡・移動判定
    association_distance_ = declare_parameter<double>("association_distance", 0.7);
    velocity_filter_alpha_ = declare_parameter<double>("velocity_filter_alpha", 0.4);
    dynamic_speed_threshold_ = declare_parameter<double>("dynamic_speed_threshold", 0.25);
    dynamic_confirm_count_ = declare_parameter<int>("dynamic_confirm_count", 3);
    dynamic_hold_time_ = declare_parameter<double>("dynamic_hold_time", 2.0);
    // 瞬間速度だけで判定すると、接近中に見える面が増えて重心がずれるだけの静止物を
    // 移動物と誤判定してしまう。窓内の正味移動量も条件に加えて誤判定を防ぐ。
    dynamic_window_ = declare_parameter<double>("dynamic_window", 1.0);
    dynamic_min_displacement_ = declare_parameter<double>("dynamic_min_displacement", 0.4);
    static_confirm_time_ = declare_parameter<double>("static_confirm_time", 0.6);
    static_memory_time_ = declare_parameter<double>("static_memory_time", 5.0);
    max_dynamic_cluster_radius_ = declare_parameter<double>("max_dynamic_cluster_radius", 1.2);
    track_timeout_ = declare_parameter<double>("track_timeout", 0.6);

    // グローバルプランとの干渉判定
    path_lookahead_ = declare_parameter<double>("path_lookahead", 6.0);
    path_clearance_ = declare_parameter<double>("path_clearance", 0.45);
    dynamic_stop_lookahead_ = declare_parameter<double>("dynamic_stop_lookahead", 4.0);
    resume_delay_ = declare_parameter<double>("resume_delay", 1.5);
    // 静止障害物が経路上にある間は減速する。迂回プランへ乗り移るための余裕を作る。
    static_slow_lookahead_ = declare_parameter<double>("static_slow_lookahead", 5.0);
    static_slow_speed_ = declare_parameter<double>("static_slow_speed", 0.4);

    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);

    rclcpp::SensorDataQoS scan_qos;
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, scan_qos,
      std::bind(&ObstacleMonitor::scanCallback, this, std::placeholders::_1));
    sub_plan_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic, rclcpp::QoS(1),
      std::bind(&ObstacleMonitor::planCallback, this, std::placeholders::_1));

    pub_static_cloud_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(static_cloud_topic, rclcpp::SensorDataQoS());
    pub_clearing_cloud_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(clearing_cloud_topic, rclcpp::SensorDataQoS());
    pub_stop_ = create_publisher<std_msgs::msg::Bool>(stop_topic, rclcpp::QoS(10));
    pub_slow_ = create_publisher<std_msgs::msg::Float32>(slow_topic, rclcpp::QoS(10));
    pub_status_ = create_publisher<std_msgs::msg::String>("obstacle_monitor/status", rclcpp::QoS(10));
    pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_monitor/markers", rclcpp::QoS(1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate)),
      std::bind(&ObstacleMonitor::publishState, this));

    last_clear_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "obstacle_monitor started: scan=%s plan=%s static_cloud=%s stop=%s",
      scan_topic.c_str(), plan_topic.c_str(), static_cloud_topic.c_str(), stop_topic.c_str());
  }

private:
  // ---------------------------------------------------------------- callbacks

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      return;
    }
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
          global_frame_, msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(0.1));
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

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::TransformStamped ts;
    try {
      ts = tf_buffer_->lookupTransform(
        global_frame_, scan->header.frame_id, scan->header.stamp, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "スキャンの座標変換に失敗: %s", ex.what());
      return;
    }

    const rclcpp::Time stamp(scan->header.stamp);
    const tf2::Transform tf = toTf2(ts);

    std::vector<Cluster> clusters = buildClusters(*scan, tf);
    updateTracks(clusters, stamp);
    classifyTracks(stamp);
    updateStaticMemory(stamp);
    publishStaticCloud(*scan, clusters);
    publishClearingCloud(*scan);
    evaluatePath(clusters, stamp);
    if (publish_markers_) {
      publishMarkers(stamp);
    }
  }

  // ------------------------------------------------------------------ helpers

  static tf2::Transform toTf2(const geometry_msgs::msg::TransformStamped & ts)
  {
    const auto & q = ts.transform.rotation;
    const auto & t = ts.transform.translation;
    tf2::Transform tf;
    tf.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
    tf.setOrigin(tf2::Vector3(t.x, t.y, t.z));
    return tf;
  }

  // 隣接ビームの距離でスキャンを分割する簡易クラスタリング
  std::vector<Cluster> buildClusters(
    const sensor_msgs::msg::LaserScan & scan, const tf2::Transform & tf)
  {
    std::vector<Cluster> clusters;
    Cluster current;
    int last_index = -100;

    const size_t n = scan.ranges.size();
    for (size_t i = 0; i < n; ++i) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
        continue;
      }
      if (r < min_detection_range_ || r > max_detection_range_) {
        continue;
      }
      const double angle = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      Point2D p;
      p.lx = r * std::cos(angle);
      p.ly = r * std::sin(angle);
      const tf2::Vector3 v = tf * tf2::Vector3(p.lx, p.ly, 0.0);
      p.x = v.x();
      p.y = v.y();

      const bool break_cluster =
        current.points.empty() ||
        (static_cast<int>(i) - last_index) > max_beam_gap_ ||
        std::hypot(p.x - current.points.back().x, p.y - current.points.back().y) >
        cluster_tolerance_;

      if (break_cluster && !current.points.empty()) {
        finalizeCluster(current, clusters);
        current = Cluster();
      }
      current.points.push_back(p);
      last_index = static_cast<int>(i);
    }
    finalizeCluster(current, clusters);
    return clusters;
  }

  void finalizeCluster(Cluster & c, std::vector<Cluster> & out)
  {
    if (static_cast<int>(c.points.size()) < min_cluster_points_) {
      return;
    }
    double sx = 0.0;
    double sy = 0.0;
    for (const auto & p : c.points) {
      sx += p.x;
      sy += p.y;
    }
    c.cx = sx / static_cast<double>(c.points.size());
    c.cy = sy / static_cast<double>(c.points.size());
    c.radius = 0.0;
    for (const auto & p : c.points) {
      c.radius = std::max(c.radius, std::hypot(p.x - c.cx, p.y - c.cy));
    }
    out.push_back(c);
  }

  // 最近傍によるクラスタ <-> トラックの対応付け
  void updateTracks(std::vector<Cluster> & clusters, const rclcpp::Time & stamp)
  {
    for (auto & t : tracks_) {
      t.matched = false;
    }

    for (auto & c : clusters) {
      Track * best = nullptr;
      double best_dist = association_distance_;
      for (auto & t : tracks_) {
        if (t.matched) {
          continue;
        }
        const double d = std::hypot(c.cx - t.x, c.cy - t.y);
        if (d < best_dist) {
          best_dist = d;
          best = &t;
        }
      }

      if (best == nullptr) {
        Track t;
        t.id = next_track_id_++;
        t.x = c.cx;
        t.y = c.cy;
        t.radius = c.radius;
        t.first_seen = stamp;
        t.last_seen = stamp;
        t.last_moving_time = stamp;
        t.matched = true;
        // 直前まで同じ場所に静止物があったなら、静止判定をやり直さない。
        // クラスタの分裂・結合でトラックが作り直されるたびに static_cloud から
        // 点が消えると、グローバルプランが行き来してしまうため。
        if (hasStaticMemoNear(c.cx, c.cy, stamp)) {
          t.first_seen = stamp - rclcpp::Duration::from_seconds(static_confirm_time_);
        }
        t.history.push_back(Sample{stamp, c.cx, c.cy});
        tracks_.push_back(t);
        c.track_id = t.id;
        continue;
      }

      const double dt = (stamp - best->last_seen).seconds();
      if (dt > 1e-3) {
        const double vx = (c.cx - best->x) / dt;
        const double vy = (c.cy - best->y) / dt;
        best->vx = velocity_filter_alpha_ * vx + (1.0 - velocity_filter_alpha_) * best->vx;
        best->vy = velocity_filter_alpha_ * vy + (1.0 - velocity_filter_alpha_) * best->vy;
      }
      best->x = c.cx;
      best->y = c.cy;
      best->radius = c.radius;
      best->last_seen = stamp;
      best->matched = true;
      best->history.push_back(Sample{stamp, c.cx, c.cy});
      while (best->history.size() > 1 &&
        (stamp - best->history.front().stamp).seconds() > dynamic_window_)
      {
        best->history.pop_front();
      }
      c.track_id = best->id;
    }

    // 一定時間見えないトラックを破棄
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const Track & t) {
          return (stamp - t.last_seen).seconds() > track_timeout_;
        }),
      tracks_.end());
  }

  void classifyTracks(const rclcpp::Time & stamp)
  {
    for (auto & t : tracks_) {
      if (!t.matched) {
        continue;
      }
      // 壁や長い柵など大きなクラスタは形状が見え隠れして速度推定が暴れるため、
      // 常に静止物として扱う。
      if (t.radius > max_dynamic_cluster_radius_) {
        t.is_dynamic = false;
        t.moving_streak = 0;
        t.vx = 0.0;
        t.vy = 0.0;
        continue;
      }

      const double speed = std::hypot(t.vx, t.vy);
      // 窓内の正味移動量。接近によって重心が少しずつずれるだけの静止物では伸びない。
      double displacement = 0.0;
      if (t.history.size() >= 2) {
        const Sample & oldest = t.history.front();
        const Sample & newest = t.history.back();
        if ((newest.stamp - oldest.stamp).seconds() >= 0.6 * dynamic_window_) {
          displacement = std::hypot(newest.x - oldest.x, newest.y - oldest.y);
        }
      }

      if (speed >= dynamic_speed_threshold_ && displacement >= dynamic_min_displacement_) {
        t.moving_streak++;
        t.last_moving_time = stamp;
        if (t.moving_streak >= dynamic_confirm_count_) {
          t.is_dynamic = true;
        }
      } else {
        t.moving_streak = 0;
        // 一度移動物と判定したら、止まって見えても dynamic_hold_time の間は移動物のまま
        // (歩行者が立ち止まった瞬間にグローバルコストマップへ焼き付くのを防ぐ)
        if (t.is_dynamic && (stamp - t.last_moving_time).seconds() > dynamic_hold_time_) {
          t.is_dynamic = false;
        }
      }
    }
  }

  // 静止と確定したトラックの位置を記憶しておく
  void updateStaticMemory(const rclcpp::Time & stamp)
  {
    for (const auto & t : tracks_) {
      if (!t.matched || !isConfirmedStatic(t)) {
        continue;
      }
      bool merged = false;
      for (auto & m : static_memory_) {
        if (std::hypot(m.x - t.x, m.y - t.y) < association_distance_) {
          m.x = t.x;
          m.y = t.y;
          m.stamp = stamp;
          merged = true;
          break;
        }
      }
      if (!merged) {
        static_memory_.push_back(StaticMemo{t.x, t.y, stamp});
      }
    }
    static_memory_.erase(
      std::remove_if(
        static_memory_.begin(), static_memory_.end(),
        [&](const StaticMemo & m) {
          return (stamp - m.stamp).seconds() > static_memory_time_;
        }),
      static_memory_.end());
  }

  bool hasStaticMemoNear(double x, double y, const rclcpp::Time & stamp) const
  {
    for (const auto & m : static_memory_) {
      if ((stamp - m.stamp).seconds() > static_memory_time_) {
        continue;
      }
      if (std::hypot(m.x - x, m.y - y) < association_distance_) {
        return true;
      }
    }
    return false;
  }

  // global_costmap に書き込んでよい (= 静止と確定した) トラックか
  bool isConfirmedStatic(const Track & t) const
  {
    if (t.radius > max_dynamic_cluster_radius_) {
      return true;  // 壁・柵などの大きな構造物は即座に静止物扱い
    }
    return !t.is_dynamic && (t.last_seen - t.first_seen).seconds() >= static_confirm_time_;
  }

  const Track * findTrack(int id) const
  {
    for (const auto & t : tracks_) {
      if (t.id == id) {
        return &t;
      }
    }
    return nullptr;
  }

  // 静止と確定した点のみをセンサフレームのまま出力する。
  // global_costmap の obstacle_layer はこれを marking ソースに、生スキャンを
  // clearing 専用ソースに使うため、移動物のセルは毎スキャン消える。
  void publishStaticCloud(const sensor_msgs::msg::LaserScan & scan, const std::vector<Cluster> & clusters)
  {
    std::vector<const Point2D *> static_points;
    for (const auto & c : clusters) {
      const Track * t = findTrack(c.track_id);
      if (t == nullptr) {
        continue;
      }
      if (!isConfirmedStatic(*t)) {
        continue;
      }
      for (const auto & p : c.points) {
        static_points.push_back(&p);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_points.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    for (const auto * p : static_points) {
      *it_x = static_cast<float>(p->lx);
      *it_y = static_cast<float>(p->ly);
      *it_z = 0.0f;
      ++it_x;
      ++it_y;
      ++it_z;
    }
    pub_static_cloud_->publish(cloud);
  }

  // 消去専用の点群。
  //  - 反射があったビーム : 計測距離より clearing_shrink_ だけ手前までを消去する。
  //    手前で止めるのは、マーキング済みの静止障害物を「自分自身の反射」で毎スキャン
  //    消してしまわないため。障害物が実際に無くなればレイはその先まで通るので、
  //    古いセルはきちんと消去される。
  //  - 反射が無いビーム : clearing_range_ まで消去する。
  //    生スキャンをそのまま渡すと range_max 超のビームが捨てられてレイが消去されず、
  //    一度書き込まれた障害物が残り続けるため、ここで点を作って補う。
  void publishClearingCloud(const sensor_msgs::msg::LaserScan & scan)
  {
    std::vector<std::pair<double, double>> points;
    points.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      const bool hit = std::isfinite(r) && r >= scan.range_min && r <= scan.range_max;
      double range;
      if (hit) {
        range = std::min(static_cast<double>(r), clearing_range_) - clearing_shrink_;
        if (range <= min_detection_range_) {
          continue;  // 手前すぎるレイは消去に使わない
        }
      } else {
        range = clearing_range_;
      }
      const double angle = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      points.emplace_back(range * std::cos(angle), range * std::sin(angle));
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    for (const auto & p : points) {
      *it_x = static_cast<float>(p.first);
      *it_y = static_cast<float>(p.second);
      *it_z = 0.0f;
      ++it_x;
      ++it_y;
      ++it_z;
    }
    pub_clearing_cloud_->publish(cloud);
  }

  // グローバルプラン上を塞いでいる障害物を探す
  void evaluatePath(const std::vector<Cluster> & clusters, const rclcpp::Time & stamp)
  {
    dynamic_blocking_ = false;
    static_blocking_ = false;
    static_block_arc_ = std::numeric_limits<double>::max();

    if (path_.size() < 2) {
      return;
    }

    geometry_msgs::msg::TransformStamped ts;
    try {
      ts = tf_buffer_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ロボット位置の取得に失敗: %s", ex.what());
      return;
    }
    const double rx = ts.transform.translation.x;
    const double ry = ts.transform.translation.y;

    // ロボットに最も近いプラン上の点を探し、そこから前方のみを対象にする
    size_t start = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_.size(); ++i) {
      const double d = std::hypot(path_[i].first - rx, path_[i].second - ry);
      if (d < best) {
        best = d;
        start = i;
      }
    }

    // 前方 path_lookahead_ [m] 分のセグメントを切り出す(累積距離付き)
    std::vector<std::pair<double, double>> segs;
    std::vector<double> seg_arc;  // セグメント始点までの経路長
    double arc = 0.0;
    segs.push_back(path_[start]);
    seg_arc.push_back(0.0);
    for (size_t i = start + 1; i < path_.size(); ++i) {
      arc += std::hypot(path_[i].first - path_[i - 1].first, path_[i].second - path_[i - 1].second);
      segs.push_back(path_[i]);
      seg_arc.push_back(arc);
      if (arc > path_lookahead_) {
        break;
      }
    }
    if (segs.size() < 2) {
      return;
    }

    for (const auto & c : clusters) {
      const Track * t = findTrack(c.track_id);
      if (t == nullptr) {
        continue;
      }
      double min_dist = std::numeric_limits<double>::max();
      double arc_at_min = std::numeric_limits<double>::max();
      for (const auto & p : c.points) {
        for (size_t i = 0; i + 1 < segs.size(); ++i) {
          const double d = distancePointToSegment(
            p.x, p.y, segs[i].first, segs[i].second, segs[i + 1].first, segs[i + 1].second);
          if (d < min_dist) {
            min_dist = d;
            arc_at_min = seg_arc[i];
          }
        }
      }
      if (min_dist > path_clearance_) {
        continue;
      }
      if (t->is_dynamic) {
        if (arc_at_min <= dynamic_stop_lookahead_) {
          dynamic_blocking_ = true;
        }
      } else if (isConfirmedStatic(*t)) {
        // 静止と確定済み = すでに global_costmap に入っているので、
        // グローバルプランナ側で回避経路が生成される
        static_blocking_ = true;
        static_block_arc_ = std::min(static_block_arc_, arc_at_min);
      }
    }

    if (dynamic_blocking_) {
      last_dynamic_block_time_ = stamp;
    }
  }

  // 停止指令と状態を一定周期で出力する
  void publishState()
  {
    const rclcpp::Time t_now = now();

    bool stop = false;
    if (dynamic_blocking_) {
      stop = true;
    } else if (stop_active_ && last_dynamic_block_time_.nanoseconds() > 0) {
      // 障害物が通過しても resume_delay の間は停止を保持し、チャタリングを防ぐ
      stop = (t_now - last_dynamic_block_time_).seconds() < resume_delay_;
    }

    if (stop != stop_active_) {
      RCLCPP_INFO(
        get_logger(), stop ? "移動障害物を検知: 停止します" : "移動障害物が通過: 走行を再開します");
    }
    stop_active_ = stop;

    std_msgs::msg::Bool stop_msg;
    stop_msg.data = stop;
    pub_stop_->publish(stop_msg);

    // 静止障害物が経路上にある間は減速する。迂回プランが出てから乗り移るまでの
    // 余裕が無いと、プラン変更に追従できないまま障害物へ近づいてしまうため。
    // 0 以下は「制限なし」を意味する。
    const bool slow = static_blocking_ && static_block_arc_ <= static_slow_lookahead_;
    std_msgs::msg::Float32 slow_msg;
    slow_msg.data = slow ? static_cast<float>(static_slow_speed_) : 0.0f;
    pub_slow_->publish(slow_msg);

    std_msgs::msg::String status;
    if (stop) {
      status.data = "WAIT_DYNAMIC";
    } else if (static_blocking_) {
      status.data = slow ? "REPLAN_STATIC_SLOW" : "REPLAN_STATIC";
    } else {
      status.data = "CLEAR";
      last_clear_time_ = t_now;
    }
    pub_status_->publish(status);
  }

  void publishMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker del;
    del.header.frame_id = global_frame_;
    del.header.stamp = stamp;
    del.ns = "obstacle_monitor";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int id = 0;
    for (const auto & t : tracks_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = global_frame_;
      m.header.stamp = stamp;
      m.ns = "obstacle_monitor";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = t.x;
      m.pose.position.y = t.y;
      m.pose.position.z = 0.2;
      m.pose.orientation.w = 1.0;
      const double d = 2.0 * std::max(0.15, t.radius);
      m.scale.x = d;
      m.scale.y = d;
      m.scale.z = 0.4;
      m.color.a = 0.5f;
      m.color.r = t.is_dynamic ? 1.0f : 0.1f;
      m.color.g = t.is_dynamic ? 0.1f : 0.6f;
      m.color.b = t.is_dynamic ? 0.1f : 1.0f;
      m.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(m);
    }
    pub_markers_->publish(arr);
  }

  // ---------------------------------------------------------------- members

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_static_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clearing_cloud_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slow_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string global_frame_;
  std::string robot_base_frame_;

  double min_detection_range_;
  double max_detection_range_;
  double clearing_range_;
  double clearing_shrink_;
  double cluster_tolerance_;
  int min_cluster_points_;
  int max_beam_gap_;
  double association_distance_;
  double velocity_filter_alpha_;
  double dynamic_speed_threshold_;
  int dynamic_confirm_count_;
  double dynamic_hold_time_;
  double dynamic_window_;
  double dynamic_min_displacement_;
  double static_confirm_time_;
  double static_memory_time_;
  double max_dynamic_cluster_radius_;
  double track_timeout_;
  double path_lookahead_;
  double path_clearance_;
  double dynamic_stop_lookahead_;
  double resume_delay_;
  double static_slow_lookahead_;
  double static_slow_speed_;
  bool publish_markers_;

  std::vector<Track> tracks_;
  std::vector<StaticMemo> static_memory_;
  int next_track_id_{0};
  std::vector<std::pair<double, double>> path_;

  bool dynamic_blocking_{false};
  bool static_blocking_{false};
  double static_block_arc_{std::numeric_limits<double>::max()};
  bool stop_active_{false};
  rclcpp::Time last_dynamic_block_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_clear_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleMonitor>());
  rclcpp::shutdown();
  return 0;
}
