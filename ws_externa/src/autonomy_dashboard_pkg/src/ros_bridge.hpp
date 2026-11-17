// Nodo ROS del dashboard: recibe nube, mapa 2D y odometría, y guarda los
// datos en memoria compartida para que la interfaz los dibuje sin copiarlos.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct Pose2D {
  double x = 0, y = 0, z = 0, yaw = 0;
  bool valid = false;
};

struct GridData {
  int width = 0, height = 0;
  double res = 0.05, ox = 0, oy = 0;
  std::vector<int8_t> data;
  // Estadísticas
  size_t known = 0, free = 0, occupied = 0;
};

struct Poi {
  std::string type, name;
  double x = 0, y = 0, z = 0;
  int time_s = 0;
};

// Contador de mensajes por topic, para mostrar Hz y si está vivo
struct TopicStat {
  std::string name;
  std::atomic<uint64_t> count{0};
  std::atomic<int64_t> last_ns{0};
};

class RosBridge : public rclcpp::Node {
public:
  RosBridge();

  // --- Configuración (solo lectura después del constructor)
  std::string team, mission, output_dir, map_frame;
  int max_time = 1200;
  int auto_export_time = 1200;
  int geotiff_px_per_cell = 2;
  bool autostart = false;
  bool auto_export_enabled = false;

  // --- Nube 3D acumulada (protegida por cloud_mx)
  std::mutex cloud_mx;
  std::vector<float> map_xyz;      // x,y,z intercalados, un punto por voxel
  std::vector<float> scan_xyz;     // último escaneo (submuestreado)
  float zmin = 1e9f, zmax = -1e9f;
  uint64_t map_epoch = 0;          // cambia al hacer reset
  std::atomic<uint64_t> map_version{0};
  std::atomic<uint64_t> scan_version{0};
  std::atomic<size_t> last_scan_points{0};
  std::atomic<bool> map_full{false};

  // --- Mapa 2D (protegido por grid_mx)
  std::mutex grid_mx;
  GridData grid;
  std::atomic<uint64_t> grid_version{0};

  // --- Pose y trayectoria (protegidas por pose_mx)
  std::mutex pose_mx;
  Pose2D pose;
  std::vector<std::array<float, 3>> path;
  std::atomic<uint64_t> pose_version{0};

  // --- Objetos detectados (protegidos por poi_mx)
  std::mutex poi_mx;
  std::vector<Poi> pois;

  // --- Estado de la misión (lo escribe la UI)
  std::atomic<bool> mapping_active{false};
  std::atomic<int> elapsed_s{0};
  std::atomic<bool> tf_ok{false};

  std::array<TopicStat, 4> stats;  // cloud, grid, odom, objects

  void clearMission();
  void requestOctomapReset();

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void gridCb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void objectsCb(const std_msgs::msg::String::ConstSharedPtr msg);
  void publishStatus();

  double voxel_ = 0.05, min_range_ = 0.6, max_range_ = 15.0;
  size_t max_map_points_ = 4000000, max_scan_points_ = 20000;
  std::unordered_set<uint64_t> voxels_;  // protegido por cloud_mx
  std::vector<float> tmp_;               // buffer reutilizable (solo hilo ROS)

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr objects_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr area_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
