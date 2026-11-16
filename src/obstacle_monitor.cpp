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
//
// 草むら(揺れる障害物)の除外:
//  固体の障害物は「一度点が見えた場所をその後のレーザーが素通りする」ことが無い(動いた場合を除く)。
//  草や枝葉は揺れたり隙間があったりするため、同じ場所が「当たる」「素通りする」を短時間に
//  何度も繰り返す。地図座標の細かいセルごとにこの入れ替わり回数(flip)を数え、
//  入れ替わりの多いセルが大半を占めるクラスタを草むらとみなして、
//    - static_cloud (global_costmap) に書き込まない
//    - filtered_scan (local_costmap 用) から該当ビームを除去 (+inf = 何も無い) する
//    - 経路上の静止障害物(減速)として扱わない
//  移動障害物と判定されたトラックは草むら扱いしない(停止判定は従来どおり)。
//
// 通過モード (pass_through):
//  waypoint_manager の pass_through 属性の区間では /wizurg/pass_through (Bool) が true になる。
//  茂みが経路にかかっていて毎回止まる場所などのための、区間を限定した強制通過。
//    - 停止・減速指令を出さない (速度制限は waypoint_manager が slow_cmd_vel で行う)
//    - グローバルプランから pass_through_corridor 以内の点を static_cloud / filtered_scan から除く
//  区間内では経路上の障害物 (人を含む) が見えなくなるため、使う区間は最小限にすること。

#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
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
  int beam{-1};  // スキャン上のビーム番号
  double tx{0.0};  // 表面の接線方向 (global frame, 単位ベクトル)
  double ty{0.0};
  bool vegetation{false};  // 草むらとして除外する点
};

struct Cluster
{
  std::vector<Point2D> points;
  double cx{0.0};
  double cy{0.0};
  double radius{0.0};
  int track_id{-1};
  bool is_vegetation{false};
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
  bool is_vegetation{false};
  bool matched{false};
};

// 揺れ(草むら)判定用のセル。地図座標で管理する。
// hits/passes/flips は時定数 vegetation_window で指数減衰させる。
struct VegCell
{
  double px{0.0};  // 最後に当たった点 (global frame)
  double py{0.0};
  double tx{0.0};  // その点の表面接線
  double ty{0.0};
  double hits{0.0};    // 当たった回数
  double passes{0.0};  // レーザーが素通りした回数
  double flips{0.0};   // 当たる <-> 素通り が入れ替わった回数
  double stamp{0.0};   // 最後に更新した時刻 [s]
  double veg_until{0.0};  // この時刻までは草むら領域として扱う [s]
  int state{0};        // 0: 未観測, 1: 当たり, 2: 素通り
  int scan_state{0};   // 現在のスキャンでの観測結果
  uint64_t scan_seq{0};
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

    // 草むら(揺れる障害物)の除外
    vegetation_filter_enabled_ = declare_parameter<bool>("vegetation_filter_enabled", true);
    vegetation_cell_size_ = declare_parameter<double>("vegetation_cell_size", 0.1);
    vegetation_range_ = declare_parameter<double>("vegetation_range", 6.0);
    vegetation_window_ = declare_parameter<double>("vegetation_window", 2.0);
    vegetation_ray_margin_ = declare_parameter<double>("vegetation_ray_margin", 0.2);
    vegetation_perp_tolerance_ = declare_parameter<double>("vegetation_perp_tolerance", 0.03);
    vegetation_presence_radius_ = declare_parameter<double>("vegetation_presence_radius", 0.03);
    vegetation_min_cluster_points_ = declare_parameter<int>("vegetation_min_cluster_points", 3);
    vegetation_use_no_return_ = declare_parameter<bool>("vegetation_use_no_return", false);
    vegetation_hold_time_ = declare_parameter<double>("vegetation_hold_time", 3.0);
    vegetation_min_cluster_size_ = declare_parameter<double>("vegetation_min_cluster_size", 0.2);
    vegetation_min_incidence_deg_ =
      declare_parameter<double>("vegetation_min_incidence_deg", 25.0);
    vegetation_min_flips_ = declare_parameter<double>("vegetation_min_flips", 3.0);
    vegetation_max_hit_ratio_ = declare_parameter<double>("vegetation_max_hit_ratio", 0.8);
    vegetation_min_observations_ = declare_parameter<double>("vegetation_min_observations", 4.0);
    vegetation_cluster_ratio_ = declare_parameter<double>("vegetation_cluster_ratio", 0.4);
    vegetation_solid_hit_ratio_ = declare_parameter<double>("vegetation_solid_hit_ratio", 0.95);
    vegetation_solid_min_observations_ =
      declare_parameter<double>("vegetation_solid_min_observations", 10.0);
    vegetation_clear_extend_ = declare_parameter<double>("vegetation_clear_extend", 0.3);
    const auto filtered_scan_topic =
      declare_parameter<std::string>("filtered_scan_topic", "obstacle_monitor/filtered_scan");
    const auto pass_through_topic =
      declare_parameter<std::string>("pass_through_topic", "/wizurg/pass_through");
    pass_through_corridor_ = declare_parameter<double>("pass_through_corridor", 0.6);
    pass_through_lookahead_ = declare_parameter<double>("pass_through_lookahead", 6.0);
    pass_through_timeout_ = declare_parameter<double>("pass_through_timeout", 1.0);
    const auto vegetation_cloud_topic =
      declare_parameter<std::string>("vegetation_cloud_topic", "obstacle_monitor/vegetation_cloud");

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
    sub_pass_through_ = create_subscription<std_msgs::msg::Bool>(
      pass_through_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data != pass_through_) {
          RCLCPP_WARN(
            get_logger(), msg->data ?
            "通過モード開始: 経路付近の障害物を無視します" : "通過モード終了");
        }
        pass_through_ = msg->data;
        pass_through_stamp_ = now();
      });
    pub_filtered_scan_ =
      create_publisher<sensor_msgs::msg::LaserScan>(filtered_scan_topic, rclcpp::SensorDataQoS());
    pub_vegetation_cloud_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(vegetation_cloud_topic, rclcpp::SensorDataQoS());
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
      "obstacle_monitor started: scan=%s plan=%s static_cloud=%s stop=%s filtered_scan=%s "
      "vegetation_filter=%s",
      scan_topic.c_str(), plan_topic.c_str(), static_cloud_topic.c_str(), stop_topic.c_str(),
      filtered_scan_topic.c_str(), vegetation_filter_enabled_ ? "on" : "off");
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
    if (vegetation_filter_enabled_) {
      updateVegetation(*scan, tf, clusters, stamp);
    }
    updateStaticMemory(stamp);
    const std::vector<char> suppressed = suppressedBeams(*scan, tf, clusters);
    publishStaticCloud(*scan, clusters, suppressed);
    publishClearingCloud(*scan, suppressed);
    publishFilteredScan(*scan, suppressed);
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
      p.beam = static_cast<int>(i);
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
    // 隣接点から表面の接線方向を求める (揺れ判定で、壁をかすめるレーザーを除外するため)
    const size_t m = c.points.size();
    for (size_t k = 0; k < m; ++k) {
      const Point2D & a = c.points[k == 0 ? 0 : k - 1];
      const Point2D & b = c.points[k + 1 >= m ? m - 1 : k + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len = std::hypot(dx, dy);
      if (len > 1e-6) {
        c.points[k].tx = dx / len;
        c.points[k].ty = dy / len;
      }
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
      if (!t.matched || t.is_vegetation || !isConfirmedStatic(t)) {
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
  void publishStaticCloud(
    const sensor_msgs::msg::LaserScan & scan, const std::vector<Cluster> & clusters,
    const std::vector<char> & suppressed)
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
        if (suppressed[p.beam]) {
          continue;  // 草むら・通過モードの除外点は global_costmap に書き込まない
        }
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
  //  - 除外したビーム (草むら・通過モード) : 計測距離より vegetation_clear_extend_ だけ先まで消去する。
  //    過去にマーキングされたセルを確実に消すため。
  void publishClearingCloud(
    const sensor_msgs::msg::LaserScan & scan, const std::vector<char> & veg_beams)
  {
    std::vector<std::pair<double, double>> points;
    points.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      const bool hit = std::isfinite(r) && r >= scan.range_min && r <= scan.range_max;
      double range;
      if (hit && veg_beams[i]) {
        range = std::min(static_cast<double>(r) + vegetation_clear_extend_, clearing_range_);
      } else if (hit) {
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

  // ---------------------------------------------------------- vegetation

  static int64_t cellKey(int ix, int iy)
  {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(iy));
  }

  int cellIndex(double v) const
  {
    return static_cast<int>(std::floor(v / vegetation_cell_size_));
  }

  // 現在時刻まで減衰させた統計値
  void decayedStats(const VegCell & c, double t, double & hits, double & passes, double & flips) const
  {
    const double f = std::exp(-std::max(0.0, t - c.stamp) / std::max(1e-3, vegetation_window_));
    hits = c.hits * f;
    passes = c.passes * f;
    flips = c.flips * f;
  }

  // 「当たる」「素通り」を繰り返している(揺れている)セルか
  bool isFlickeringCell(const VegCell & c, double t) const
  {
    double hits, passes, flips;
    decayedStats(c, t, hits, passes, flips);
    const double obs = hits + passes;
    if (obs < vegetation_min_observations_) {
      return false;
    }
    return flips >= vegetation_min_flips_ && hits / obs <= vegetation_max_hit_ratio_;
  }

  // 安定して当たり続けている(固体とみなせる)セルか
  bool isSolidCell(const VegCell & c, double t) const
  {
    double hits, passes, flips;
    decayedStats(c, t, hits, passes, flips);
    const double obs = hits + passes;
    return obs >= vegetation_solid_min_observations_ && hits / obs >= vegetation_solid_hit_ratio_ &&
           flips < 1.0;
  }

  void updateVegetation(
    const sensor_msgs::msg::LaserScan & scan, const tf2::Transform & tf,
    std::vector<Cluster> & clusters, const rclcpp::Time & stamp)
  {
    const double t = stamp.seconds();
    const uint64_t seq = ++scan_seq_;
    std::vector<int64_t> touched;

    // 1) 当たった点を記録する
    std::unordered_map<int64_t, std::vector<std::pair<double, double>>> current_hits;
    for (const auto & c : clusters) {
      for (const auto & p : c.points) {
        if (std::hypot(p.lx, p.ly) > vegetation_range_) {
          continue;
        }
        current_hits[cellKey(cellIndex(p.x), cellIndex(p.y))].emplace_back(p.x, p.y);
        const int64_t key = cellKey(cellIndex(p.x), cellIndex(p.y));
        VegCell & cell = veg_cells_[key];
        if (cell.scan_seq != seq) {
          touched.push_back(key);
        }
        cell.scan_seq = seq;
        cell.scan_state = 1;
        cell.px = p.x;
        cell.py = p.y;
        cell.tx = p.tx;
        cell.ty = p.ty;
      }
    }

    // 2) 過去に点があった場所をレーザーが素通りしたかを調べる
    const tf2::Vector3 origin = tf.getOrigin();
    const double ox = origin.x();
    const double oy = origin.y();
    const double sin_min_incidence = std::sin(vegetation_min_incidence_deg_ * M_PI / 180.0);
    const double step = 0.5 * vegetation_cell_size_;
    const double beam_spacing = std::abs(scan.angle_increment);
    const double presence2 = vegetation_presence_radius_ * vegetation_presence_radius_;
    // 過去の点のすぐ近くに今回も点が見えている (= 物はまだそこにある) か
    auto stillPresent = [&](double x, double y) {
        const int ix = cellIndex(x);
        const int iy = cellIndex(y);
        for (int ddx = -1; ddx <= 1; ++ddx) {
          for (int ddy = -1; ddy <= 1; ++ddy) {
            auto it = current_hits.find(cellKey(ix + ddx, iy + ddy));
            if (it == current_hits.end()) {
              continue;
            }
            for (const auto & q : it->second) {
              const double ex = q.first - x;
              const double ey = q.second - y;
              if (ex * ex + ey * ey <= presence2) {
                return true;
              }
            }
          }
        }
        return false;
      };
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      const bool hit = std::isfinite(r) && r >= scan.range_min && r <= scan.range_max;
      if (hit && r < min_detection_range_) {
        continue;
      }
      // 反射なしのビームは「何も無かった」のか「そのフレームでその方向のデータが無かった」のか
      // 区別できない (3D LiDAR を切り出したスキャンでは後者が大半) ため、既定では使わない。
      if (!hit && !vegetation_use_no_return_) {
        continue;
      }
      double length;
      if (hit && r <= vegetation_range_ + vegetation_ray_margin_) {
        // 自己位置や計測のノイズで手前の点を素通りと誤判定しないよう、計測点の手前で止める
        length = static_cast<double>(r) - vegetation_ray_margin_;
      } else {
        length = vegetation_range_;
      }
      if (length <= min_detection_range_) {
        continue;
      }
      const double angle = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      const tf2::Vector3 dir = tf.getBasis() * tf2::Vector3(std::cos(angle), std::sin(angle), 0.0);
      const double dn = std::hypot(dir.x(), dir.y());
      if (dn < 1e-6) {
        continue;
      }
      const double dx = dir.x() / dn;
      const double dy = dir.y() / dn;

      int64_t last_key = std::numeric_limits<int64_t>::min();
      for (double s = min_detection_range_; s <= length; s += step) {
        const int64_t key = cellKey(cellIndex(ox + dx * s), cellIndex(oy + dy * s));
        if (key == last_key) {
          continue;
        }
        last_key = key;
        auto it = veg_cells_.find(key);
        if (it == veg_cells_.end()) {
          continue;
        }
        VegCell & cell = it->second;
        if (cell.state == 0 || cell.scan_seq == seq) {
          continue;  // まだ点が見えたことが無い / このスキャンで判定済み
        }
        const double wx = cell.px - ox;
        const double wy = cell.py - oy;
        const double along = wx * dx + wy * dy;
        if (along < min_detection_range_ || along > length) {
          continue;
        }
        // そのビームが過去の点を「狙って」通過した場合のみ素通りとする。
        // 隣のビームが細い柱の横をかすめただけのものを数えないよう、ビーム間隔の半分未満に制限する。
        const double perp = std::abs(wx * dy - wy * dx);
        if (perp > std::min(vegetation_perp_tolerance_, 0.5 * along * beam_spacing)) {
          continue;
        }
        // 表面をかすめるように通るレーザー(壁を斜めに見ている場合など)は素通りとしない
        const double incidence = std::abs(cell.tx * dy - cell.ty * dx);
        if (incidence < sin_min_incidence) {
          continue;
        }
        // すぐ近くに今回も点が見えているなら、物はまだそこにある (計測点のばらつき・物体の端)
        if (stillPresent(cell.px, cell.py)) {
          continue;
        }
        cell.scan_seq = seq;
        cell.scan_state = 2;
        touched.push_back(key);
      }
    }

    // 3) 統計を更新する
    for (const int64_t key : touched) {
      VegCell & cell = veg_cells_[key];
      double hits, passes, flips;
      decayedStats(cell, t, hits, passes, flips);
      if (cell.scan_state == 1) {
        hits += 1.0;
      } else {
        passes += 1.0;
      }
      if (cell.state != 0 && cell.state != cell.scan_state) {
        flips += 1.0;
      }
      cell.hits = hits;
      cell.passes = passes;
      cell.flips = flips;
      cell.state = cell.scan_state;
      cell.stamp = t;
    }

    // 古いセルを破棄する
    if (seq % 20 == 0) {
      const double expire = 5.0 * vegetation_window_;
      for (auto it = veg_cells_.begin(); it != veg_cells_.end(); ) {
        if (t - it->second.stamp > expire && it->second.veg_until < t) {
          it = veg_cells_.erase(it);
        } else {
          ++it;
        }
      }
    }

    // 4) クラスタごとに草むらか判定する
    for (auto & t_ : tracks_) {
      t_.is_vegetation = false;
    }
    for (auto & c : clusters) {
      c.is_vegetation = false;
      Track * trk = findTrackMutable(c.track_id);
      if (trk != nullptr && trk->is_dynamic) {
        continue;  // 移動障害物は草むら扱いしない
      }
      // 細い柱や人の脚など小さな物は、ビーム間隔より細いと本当に「見えたり見えなかったり」するため
      // 単独では揺れと区別できない。小さなクラスタは揺れの検出(草むら領域の登録)には使わず、
      // すでに登録済みの草むら領域の中にある場合だけ草むらとする。
      const bool can_detect =
        static_cast<int>(c.points.size()) >= vegetation_min_cluster_points_ &&
        2.0 * c.radius >= vegetation_min_cluster_size_;
      size_t veg_count = 0;
      std::vector<char> solid(c.points.size(), 0);
      for (size_t k = 0; k < c.points.size(); ++k) {
        const auto & p = c.points[k];
        const int ix = cellIndex(p.x);
        const int iy = cellIndex(p.y);
        bool flicker = false;
        bool remembered = false;
        for (int ddx = -1; ddx <= 1; ++ddx) {
          for (int ddy = -1; ddy <= 1; ++ddy) {
            auto it = veg_cells_.find(cellKey(ix + ddx, iy + ddy));
            if (it == veg_cells_.end()) {
              continue;
            }
            if (can_detect && !flicker && isFlickeringCell(it->second, t)) {
              flicker = true;
            }
            if (it->second.veg_until > t) {
              remembered = true;
            }
          }
        }
        auto own = veg_cells_.find(cellKey(ix, iy));
        solid[k] = (own != veg_cells_.end() && isSolidCell(own->second, t)) ? 1 : 0;
        if (flicker) {
          // 揺れを確認した場所は一定時間「草むら領域」として覚えておく。
          // 草は瞬間ごとには一部の点しか揺れとして捉えられないため。
          for (int ddx = -1; ddx <= 1; ++ddx) {
            for (int ddy = -1; ddy <= 1; ++ddy) {
              VegCell & nc = veg_cells_[cellKey(ix + ddx, iy + ddy)];
              nc.veg_until = t + vegetation_hold_time_;
            }
          }
        }
        if ((flicker || remembered) && !solid[k]) {
          veg_count++;
        }
      }
      if (c.points.empty() ||
        static_cast<double>(veg_count) <
        vegetation_cluster_ratio_ * static_cast<double>(c.points.size()))
      {
        continue;
      }
      // 草むらクラスタ: 安定して当たり続けている点(太い幹や草の奥の壁など)以外を除外する
      c.is_vegetation = true;
      for (size_t k = 0; k < c.points.size(); ++k) {
        c.points[k].vegetation = !solid[k];
      }
      if (trk != nullptr) {
        trk->is_vegetation = true;
      }
    }
  }

  bool passThroughActive()
  {
    if (!pass_through_) {
      return false;
    }
    if ((now() - pass_through_stamp_).seconds() > pass_through_timeout_) {
      RCLCPP_WARN(get_logger(), "通過モード指令が途絶えたため通過モードを終了します");
      pass_through_ = false;
      return false;
    }
    return true;
  }

  // 除外するビーム (草むら + 通過モード中の経路付近)
  std::vector<char> suppressedBeams(
    const sensor_msgs::msg::LaserScan & scan, const tf2::Transform & tf,
    const std::vector<Cluster> & clusters)
  {
    std::vector<char> beams(scan.ranges.size(), 0);
    for (const auto & c : clusters) {
      if (!c.is_vegetation) {
        continue;
      }
      for (const auto & p : c.points) {
        if (p.vegetation && p.beam >= 0 && static_cast<size_t>(p.beam) < beams.size()) {
          beams[p.beam] = 1;
        }
      }
    }
    if (!passThroughActive()) {
      return beams;
    }

    // 通過モード: ロボット付近のプラン (無ければロボット前方の直線) から corridor 以内の点を除く
    std::vector<std::pair<double, double>> segs;
    const tf2::Vector3 o = tf.getOrigin();
    if (path_.size() >= 2) {
      size_t start = 0;
      double best = std::numeric_limits<double>::max();
      for (size_t i = 0; i < path_.size(); ++i) {
        const double d = std::hypot(path_[i].first - o.x(), path_[i].second - o.y());
        if (d < best) {
          best = d;
          start = i;
        }
      }
      // 真横・少し後ろの点も対象にするため、最近傍点より手前から切り出す
      double back = 0.0;
      size_t first = start;
      while (first > 0 && back < 1.0) {
        back += std::hypot(
          path_[first].first - path_[first - 1].first, path_[first].second - path_[first - 1].second);
        --first;
      }
      double arc = 0.0;
      segs.push_back(path_[first]);
      for (size_t i = first + 1; i < path_.size(); ++i) {
        arc += std::hypot(path_[i].first - path_[i - 1].first, path_[i].second - path_[i - 1].second);
        segs.push_back(path_[i]);
        if (arc > pass_through_lookahead_ + back) {
          break;
        }
      }
    } else {
      const tf2::Vector3 fwd = tf.getBasis() * tf2::Vector3(1.0, 0.0, 0.0);
      const double n = std::max(1e-6, std::hypot(fwd.x(), fwd.y()));
      segs.emplace_back(o.x(), o.y());
      segs.emplace_back(
        o.x() + fwd.x() / n * pass_through_lookahead_, o.y() + fwd.y() / n * pass_through_lookahead_);
    }

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
        continue;
      }
      const double angle = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      const tf2::Vector3 v = tf * tf2::Vector3(r * std::cos(angle), r * std::sin(angle), 0.0);
      for (size_t k = 0; k + 1 < segs.size(); ++k) {
        if (distancePointToSegment(
            v.x(), v.y(), segs[k].first, segs[k].second, segs[k + 1].first, segs[k + 1].second) <=
          pass_through_corridor_)
        {
          beams[i] = 1;
          break;
        }
      }
    }
    return beams;
  }

  // local_costmap 用: 草むらのビームを +inf (= 何も無い) にしたスキャン。
  // local_costmap 側は inf_is_valid: true なので、そのビームは消去にのみ使われる。
  // 草むら点群もデバッグ用に出力する。
  void publishFilteredScan(
    const sensor_msgs::msg::LaserScan & scan, const std::vector<char> & veg_beams)
  {
    sensor_msgs::msg::LaserScan out = scan;
    std::vector<std::pair<double, double>> veg_points;
    for (size_t i = 0; i < out.ranges.size(); ++i) {
      if (!veg_beams[i]) {
        continue;
      }
      const double r = out.ranges[i];
      const double angle = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      veg_points.emplace_back(r * std::cos(angle), r * std::sin(angle));
      out.ranges[i] = std::numeric_limits<float>::infinity();
      if (i < out.intensities.size()) {
        out.intensities[i] = 0.0f;
      }
    }
    pub_filtered_scan_->publish(out);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(veg_points.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    for (const auto & p : veg_points) {
      *it_x = static_cast<float>(p.first);
      *it_y = static_cast<float>(p.second);
      *it_z = 0.0f;
      ++it_x;
      ++it_y;
      ++it_z;
    }
    pub_vegetation_cloud_->publish(cloud);
  }

  Track * findTrackMutable(int id)
  {
    for (auto & t : tracks_) {
      if (t.id == id) {
        return &t;
      }
    }
    return nullptr;
  }

  // グローバルプラン上を塞いでいる障害物を探す
  void evaluatePath(const std::vector<Cluster> & clusters, const rclcpp::Time & stamp)
  {
    dynamic_blocking_ = false;
    static_blocking_ = false;
    static_block_arc_ = std::numeric_limits<double>::max();

    if (path_.size() < 2 || passThroughActive()) {
      return;  // 通過モード中は停止・減速判定をしない
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
        if (p.vegetation) {
          continue;  // 草むらは経路を塞ぐ障害物として扱わない
        }
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
    if (passThroughActive()) {
      stop = false;  // 通過モード中は停止しない (保持時間も無視)
    } else if (dynamic_blocking_) {
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
    if (passThroughActive()) {
      status.data = "PASS_THROUGH";
    } else if (stop) {
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
      // 赤: 移動障害物, 緑: 草むら(除外), 青: 静止障害物
      m.color.a = 0.5f;
      if (t.is_dynamic) {
        m.color.r = 1.0f; m.color.g = 0.1f; m.color.b = 0.1f;
      } else if (t.is_vegetation) {
        m.color.r = 0.2f; m.color.g = 0.9f; m.color.b = 0.2f;
      } else {
        m.color.r = 0.1f; m.color.g = 0.6f; m.color.b = 1.0f;
      }
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pass_through_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_static_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clearing_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_filtered_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_vegetation_cloud_;
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
  bool vegetation_filter_enabled_;
  double vegetation_cell_size_;
  double vegetation_range_;
  double vegetation_window_;
  double vegetation_ray_margin_;
  double vegetation_perp_tolerance_;
  double vegetation_presence_radius_;
  int vegetation_min_cluster_points_;
  bool vegetation_use_no_return_;
  double vegetation_hold_time_;
  double vegetation_min_cluster_size_;
  double vegetation_min_incidence_deg_;
  double vegetation_min_flips_;
  double vegetation_max_hit_ratio_;
  double vegetation_min_observations_;
  double vegetation_cluster_ratio_;
  double vegetation_solid_hit_ratio_;
  double vegetation_solid_min_observations_;
  double vegetation_clear_extend_;
  bool publish_markers_;
  double pass_through_corridor_;
  double pass_through_lookahead_;
  double pass_through_timeout_;
  bool pass_through_{false};
  rclcpp::Time pass_through_stamp_{0, 0, RCL_ROS_TIME};

  std::unordered_map<int64_t, VegCell> veg_cells_;
  uint64_t scan_seq_{0};

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
