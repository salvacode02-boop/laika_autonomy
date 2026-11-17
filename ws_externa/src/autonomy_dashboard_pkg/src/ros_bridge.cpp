#include "ros_bridge.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

using std::placeholders::_1;

namespace {

int64_t nowNs() {
  return std::chrono::steady_clock::now().time_since_epoch().count();
}

void tick(TopicStat & s) {
  s.count.fetch_add(1, std::memory_order_relaxed);
  s.last_ns.store(nowNs(), std::memory_order_relaxed);
}

// Empaqueta los índices del voxel en 64 bits (21 bits por eje, ±52 km a 5 cm)
inline uint64_t voxelKey(float x, float y, float z, float inv) {
  constexpr int64_t off = 1 << 20;
  constexpr uint64_t mask = (1u << 21) - 1;
  const uint64_t ix = static_cast<uint64_t>(static_cast<int64_t>(std::floor(x * inv)) + off) & mask;
  const uint64_t iy = static_cast<uint64_t>(static_cast<int64_t>(std::floor(y * inv)) + off) & mask;
  const uint64_t iz = static_cast<uint64_t>(static_cast<int64_t>(std::floor(z * inv)) + off) & mask;
  return (ix << 42) | (iy << 21) | iz;
}

double yawFromQuat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

RosBridge::RosBridge()
: Node("autonomy_dashboard_node")
{
  const char * home = std::getenv("HOME");
  const std::string default_out = std::string(home ? home : "/tmp") + "/RRL-2026-Robot/maps";

  team = declare_parameter<std::string>("team_name", "RoboCup2026");
  mission = declare_parameter<std::string>("mission", "Prelim1");
  output_dir = declare_parameter<std::string>("output_dir", default_out);
  map_frame = declare_parameter<std::string>("map_frame", "map");
  max_time = declare_parameter<int>("max_mapping_time", 1200);
  auto_export_time = declare_parameter<int>("auto_export_time", -1);
  if (auto_export_time <= 0) {auto_export_time = max_time;}
  autostart = declare_parameter<bool>("autostart", false);
  auto_export_enabled = declare_parameter<bool>("auto_export_enabled", false);
  geotiff_px_per_cell = std::max(1, static_cast<int>(declare_parameter<int>("geotiff_px_per_cell", 2)));

  voxel_ = declare_parameter<double>("cloud.voxel_size", 0.05);
  min_range_ = declare_parameter<double>("cloud.min_range", 0.6);
  max_range_ = declare_parameter<double>("cloud.max_range", 15.0);
  max_map_points_ = static_cast<size_t>(declare_parameter<int>("cloud.max_map_points", 4000000));
  max_scan_points_ = static_cast<size_t>(declare_parameter<int>("cloud.max_scan_points", 20000));

  const auto t_cloud = declare_parameter<std::string>("topics.cloud", "/cloud_registered");
  const auto t_grid = declare_parameter<std::string>("topics.grid", "/projected_map");
  const auto t_odom = declare_parameter<std::string>("topics.odom", "/odom_corrected");
  const auto t_obj = declare_parameter<std::string>("topics.objects", "/autonomy/detected_objects");

  stats[0].name = t_cloud;
  stats[1].name = t_grid;
  stats[2].name = t_odom;
  stats[3].name = t_obj;

  map_xyz.reserve(3 * 500000);
  voxels_.reserve(500000);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    t_cloud, rclcpp::SensorDataQoS(), std::bind(&RosBridge::cloudCb, this, _1));
  // octomap publica el mapa "latched": transient_local + reliable
  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    t_grid, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&RosBridge::gridCb, this, _1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    t_odom, rclcpp::SensorDataQoS(), std::bind(&RosBridge::odomCb, this, _1));
  objects_sub_ = create_subscription<std_msgs::msg::String>(
    t_obj, 10, std::bind(&RosBridge::objectsCb, this, _1));

  status_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/status", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/autonomy/robot_pose", 10);
  area_pub_ = create_publisher<std_msgs::msg::Float32>("/autonomy/mapped_area_percent", 10);
  octomap_reset_ = create_client<std_srvs::srv::Empty>("/octomap_server/reset");

  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publishStatus();});

  RCLCPP_INFO(get_logger(), "Dashboard listo | team=%s mission=%s | cloud=%s grid=%s odom=%s",
    team.c_str(), mission.c_str(), t_cloud.c_str(), t_grid.c_str(), t_odom.c_str());
}

void RosBridge::cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  tick(stats[0]);

  // Offsets de x, y, z (deben ser float32)
  int ox = -1, oy = -1, oz = -1;
  for (const auto & f : msg->fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {continue;}
    if (f.name == "x") {ox = f.offset;}
    else if (f.name == "y") {oy = f.offset;}
    else if (f.name == "z") {oz = f.offset;}
  }
  if (ox < 0 || oy < 0 || oz < 0) {return;}

  // Transformación del frame de la nube al frame del mapa (map ← camera_init)
  double R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, T[3] = {0, 0, 0};
  if (msg->header.frame_id != map_frame) {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame, msg->header.frame_id, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      const double x = q.x, y = q.y, z = q.z, w = q.w;
      R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - z * w);     R[2] = 2 * (x * z + y * w);
      R[3] = 2 * (x * y + z * w);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - x * w);
      R[6] = 2 * (x * z - y * w);     R[7] = 2 * (y * z + x * w);     R[8] = 1 - 2 * (x * x + y * y);
      T[0] = tf.transform.translation.x;
      T[1] = tf.transform.translation.y;
      T[2] = tf.transform.translation.z;
      tf_ok = true;
    } catch (const tf2::TransformException &) {
      tf_ok = false;  // se usa identidad
    }
  } else {
    tf_ok = true;
  }

  // Posición del robot para quitar los puntos de su propio cuerpo
  Pose2D robot;
  {
    std::lock_guard<std::mutex> lk(pose_mx);
    robot = pose;
  }
  const double min2 = min_range_ * min_range_, max2 = max_range_ * max_range_;

  // 1) Transformar y filtrar fuera del lock
  const size_t n = static_cast<size_t>(msg->width) * msg->height;
  const uint8_t * base = msg->data.data();
  tmp_.clear();
  tmp_.reserve(n * 3);
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * p = base + i * msg->point_step;
    float px, py, pz;
    std::memcpy(&px, p + ox, 4);
    std::memcpy(&py, p + oy, 4);
    std::memcpy(&pz, p + oz, 4);
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {continue;}
    const float wx = static_cast<float>(R[0] * px + R[1] * py + R[2] * pz + T[0]);
    const float wy = static_cast<float>(R[3] * px + R[4] * py + R[5] * pz + T[1]);
    const float wz = static_cast<float>(R[6] * px + R[7] * py + R[8] * pz + T[2]);
    if (robot.valid) {
      const double dx = wx - robot.x, dy = wy - robot.y, dz = wz - robot.z;
      const double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < min2 || d2 > max2) {continue;}
    }
    tmp_.push_back(wx);
    tmp_.push_back(wy);
    tmp_.push_back(wz);
  }
  const size_t m = tmp_.size() / 3;
  last_scan_points = m;
  const size_t stride = std::max<size_t>(1, (m + max_scan_points_ - 1) / max_scan_points_);
  const float inv = static_cast<float>(1.0 / voxel_);
  const bool accumulate = mapping_active.load();

  // 2) Insertar en el mapa de voxels (lock corto)
  std::lock_guard<std::mutex> lk(cloud_mx);
  scan_xyz.clear();
  for (size_t i = 0; i < m; i += stride) {
    scan_xyz.insert(scan_xyz.end(), &tmp_[3 * i], &tmp_[3 * i] + 3);
  }
  scan_version++;

  if (!accumulate) {return;}
  size_t added = 0;
  for (size_t i = 0; i < m; ++i) {
    if (voxels_.size() >= max_map_points_) {map_full = true; break;}
    const float * p = &tmp_[3 * i];
    if (voxels_.insert(voxelKey(p[0], p[1], p[2], inv)).second) {
      // Guarda el centro del voxel: la nube queda uniforme
      map_xyz.push_back((std::floor(p[0] * inv) + 0.5f) / inv);
      map_xyz.push_back((std::floor(p[1] * inv) + 0.5f) / inv);
      map_xyz.push_back((std::floor(p[2] * inv) + 0.5f) / inv);
      zmin = std::min(zmin, p[2]);
      zmax = std::max(zmax, p[2]);
      ++added;
    }
  }
  if (added) {map_version++;}
}

void RosBridge::gridCb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  tick(stats[1]);
  GridData g;
  g.width = static_cast<int>(msg->info.width);
  g.height = static_cast<int>(msg->info.height);
  g.res = msg->info.resolution;
  g.ox = msg->info.origin.position.x;
  g.oy = msg->info.origin.position.y;
  g.data = msg->data;
  for (int8_t v : g.data) {
    if (v < 0) {continue;}
    ++g.known;
    if (v >= 65) {++g.occupied;} else {++g.free;}
  }
  std::lock_guard<std::mutex> lk(grid_mx);
  grid = std::move(g);
  grid_version++;
}

void RosBridge::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  tick(stats[2]);
  auto p = msg->pose.pose.position;
  auto q = msg->pose.pose.orientation;
  // Laika: la odometría viene en odom; la pose en el mapa sale de la TF map ← child_frame_id
  if (msg->header.frame_id != map_frame) {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame, msg->child_frame_id, tf2::TimePointZero);
      p.x = tf.transform.translation.x;
      p.y = tf.transform.translation.y;
      p.z = tf.transform.translation.z;
      q = tf.transform.rotation;
    } catch (const tf2::TransformException &) {
      return;  // sin TF todavía (SLAM/AMCL sin arrancar): no hay pose en el mapa
    }
  }
  std::lock_guard<std::mutex> lk(pose_mx);
  pose.x = p.x;
  pose.y = p.y;
  pose.z = p.z;
  pose.yaw = yawFromQuat(q.x, q.y, q.z, q.w);
  pose.valid = true;

  // Trayectoria: un punto cada 5 cm mientras se mapea
  if (mapping_active) {
    if (path.empty()) {
      path.push_back({float(p.x), float(p.y), float(p.z)});
    } else {
      const auto & b = path.back();
      const double dx = p.x - b[0], dy = p.y - b[1];
      if (dx * dx + dy * dy > 0.05 * 0.05) {path.push_back({float(p.x), float(p.y), float(p.z)});}
    }
  }
  pose_version++;
}

void RosBridge::objectsCb(const std_msgs::msg::String::ConstSharedPtr msg)
{
  tick(stats[3]);
  // Acepta un objeto o una lista: {"type","name","x","y","z"}
  const auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
  QJsonArray arr;
  if (doc.isArray()) {arr = doc.array();} else if (doc.isObject()) {arr.append(doc.object());}
  std::lock_guard<std::mutex> lk(poi_mx);
  for (const auto & v : arr) {
    const auto o = v.toObject();
    if (o.isEmpty()) {continue;}
    Poi poi;
    poi.type = o.value("type").toString("unknown").toStdString();
    poi.name = o.value("name").toString(QString::fromStdString(poi.type)).toStdString();
    poi.x = o.value("x").toDouble();
    poi.y = o.value("y").toDouble();
    poi.z = o.value("z").toDouble();
    poi.time_s = elapsed_s;
    pois.push_back(poi);
  }
}

void RosBridge::publishStatus()
{
  Pose2D p;
  {
    std::lock_guard<std::mutex> lk(pose_mx);
    p = pose;
  }
  double area_pct = 0.0;
  {
    std::lock_guard<std::mutex> lk(grid_mx);
    if (!grid.data.empty()) {area_pct = 100.0 * grid.known / grid.data.size();}
  }
  size_t cloud_pts;
  {
    std::lock_guard<std::mutex> lk(cloud_mx);
    cloud_pts = map_xyz.size() / 3;
  }
  size_t n_pois;
  {
    std::lock_guard<std::mutex> lk(poi_mx);
    n_pois = pois.size();
  }

  std::ostringstream js;
  js.precision(4);
  js << std::fixed << "{\"team_name\":\"" << team << "\",\"mission\":\"" << mission
     << "\",\"autonomy_mode\":\"" << (mapping_active ? "MAPPING" : "IDLE")
     << "\",\"robot_pose\":{\"x\":" << p.x << ",\"y\":" << p.y << ",\"theta\":" << p.yaw
     << "},\"mapped_area\":" << area_pct << ",\"cloud_points\":" << cloud_pts
     << ",\"elapsed_time\":" << elapsed_s.load() << ",\"max_time\":" << max_time
     << ",\"detected_objects\":" << n_pois << "}";
  std_msgs::msg::String s;
  s.data = js.str();
  status_pub_->publish(s);

  std_msgs::msg::Float32 a;
  a.data = static_cast<float>(area_pct);
  area_pub_->publish(a);

  if (p.valid) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = map_frame;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.position.z = p.z;
    ps.pose.orientation.z = std::sin(p.yaw / 2);
    ps.pose.orientation.w = std::cos(p.yaw / 2);
    pose_pub_->publish(ps);
  }
}

void RosBridge::clearMission()
{
  {
    std::lock_guard<std::mutex> lk(cloud_mx);
    map_xyz.clear();
    map_xyz.shrink_to_fit();
    map_xyz.reserve(3 * 500000);
    voxels_.clear();
    zmin = 1e9f;
    zmax = -1e9f;
    map_epoch++;
    map_full = false;
  }
  map_version++;
  {
    std::lock_guard<std::mutex> lk(pose_mx);
    path.clear();
  }
  pose_version++;
  {
    std::lock_guard<std::mutex> lk(poi_mx);
    pois.clear();
  }
}

void RosBridge::requestOctomapReset()
{
  if (octomap_reset_->service_is_ready()) {
    octomap_reset_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  }
}
