// Autonomy Dashboard (C++ / Qt5 / OpenGL)
// El nodo ROS corre en su propio hilo; la interfaz lee los datos compartidos.
#include <thread>

#include <QApplication>
#include <QSurfaceFormat>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include "main_window.hpp"
#include "ros_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QSurfaceFormat fmt;
  fmt.setVersion(2, 1);
  fmt.setDepthBufferSize(24);
  fmt.setSwapInterval(1);
  QSurfaceFormat::setDefaultFormat(fmt);

  const auto qt_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> qargv;
  for (const auto & a : qt_args) {qargv.push_back(const_cast<char *>(a.c_str()));}
  int qargc = static_cast<int>(qargv.size());
  QApplication app(qargc, qargv.data());
  app.setApplicationName("autonomy_dashboard");

  auto node = std::make_shared<RosBridge>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread ros_thread([&exec] {exec.spin();});

  MainWindow win(node);
  win.show();

  // Ctrl+C (o ros2 launch al cerrar) apaga rclcpp: cerrar también la ventana
  QTimer watchdog;
  QObject::connect(&watchdog, &QTimer::timeout, [&] {if (!rclcpp::ok()) {app.quit();}});
  watchdog.start(200);

  const int ret = app.exec();
  exec.cancel();
  ros_thread.join();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return ret;
}
