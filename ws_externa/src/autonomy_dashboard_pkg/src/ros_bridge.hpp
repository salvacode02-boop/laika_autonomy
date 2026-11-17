// Nodo ROS del dashboard: recibe nube, mapa 2D y odometría, y guarda los
// datos en memoria compartida para que la interfaz los dibuje sin copiarlos.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct Pose2D {
  double x = 0, y = 0, z = 0, yaw = 0;
  double qx = 0, qy = 0, qz = 0, qw = 1;  // orientación completa (modelo 3D del robot)
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

// Transformación 2D (rígida) de un frame a map
struct Tf2D {
  double x = 0, y = 0, yaw = 0;
  bool valid = false;
};

// Datos de navegación (Nav2 + puente), protegidos por nav_mx. Las rutas y la huella
// ya están en frame map; el costmap local se guarda con su frame y la UI lo transforma.
struct NavData {
  GridData global_cm, local_cm;
  std::string local_frame = "odom";
  std::vector<std::array<float, 2>> plan, local_plan, footprint;  // footprint: vacía = no hay
  double cmd_vx = 0, cmd_wz = 0;
  std::string bridge_state;
};

// Aprobación de ruta (protegida por appr_mx): misma lógica que scripts/aprobar_rutas.py
struct ApprovalData {
  std::string key, approved;
  double gx = 0, gy = 0, gyaw = 0;
  bool pending = false;   // meta nueva sin plan mostrado
  bool waiting = false;   // plan mostrado, esperando aprobación
  int n = 0;
  double length = 0;
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
  std::string laika_dir = "/home/robotica/laika";   // raíz del repo (scripts/, maps/)
  std::string model_dir;                               // go2_description (urdf/ y dae/)
  // ply_to_grid (ver mapio.hpp)
  double ply_res = 0.05, ply_floor_pct = 2.0, ply_band_min = 0.10, ply_band_max = 0.93, ply_floor_tol = 0.10;
  int ply_min_points = 2;
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

  // cloud, grid, odom, objects, global_costmap, local_costmap, plan, local_plan, cmd_vel, bridge_state
  static constexpr size_t kStats = 10;
  std::array<TopicStat, kStats> stats;

  // --- Navegación (nav_mx) y aprobación (appr_mx)
  std::vector<double> fixed_footprint;  // huella fija x,y,... en base_link (parámetro)
  std::mutex nav_mx;
  NavData nav;
  std::atomic<uint64_t> nav_version{0};
  std::atomic<double> real_vx{0}, real_wz{0};  // twist de la odometría
  std::mutex appr_mx;
  ApprovalData appr;
  std::atomic<uint64_t> appr_version{0};

  // --- Ángulos de las articulaciones (/joint_states del robot), protegidos por joint_mx
  std::mutex joint_mx;
  std::map<std::string, double> joints;
  std::atomic<uint64_t> joint_version{0};
  std::atomic<int64_t> joint_last_ns{0};   // reloj steady; 0 = nunca

  // Se llaman desde el hilo de la UI (publish es thread-safe)
  void publishGoal(double x, double y, double yaw);
  void publishInitialPose(double x, double y, double yaw);
  void publishDeadman();
  void approve();
  Tf2D lookupMapFrom(const std::string & frame);

  void clearMission();
  void requestOctomapReset();

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void gridCb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void objectsCb(const std_msgs::msg::String::ConstSharedPtr msg);
  void publishStatus();
  void goalCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void planCb(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void localPlanCb(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void costmapCb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg, bool global);
  void footprintCb(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg);
  void cmdCb(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void stateCb(const std_msgs::msg::String::ConstSharedPtr msg);
  std::vector<std::array<float, 2>> pathToMap(const nav_msgs::msg::Path & p);
  nav_msgs::msg::Path last_plan_;  // solo hilo ROS
  bool have_plan_ = false;
  void evalPlan(const nav_msgs::msg::Path & p);  // llamar con appr_mx tomado

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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_, lplan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr gcm_sub_, lcm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr fp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dead_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr appr_pub_;
  rclcpp::TimerBase::SharedPtr appr_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr area_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
