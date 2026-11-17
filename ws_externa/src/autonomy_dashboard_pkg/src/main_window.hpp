// Ventana principal: encabezado con timer, vistas 2D/3D y barra lateral
// con controles de misión, estado de topics y exportación.
#pragma once

#include <array>
#include <memory>

#include <QDateTime>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QMainWindow>
#include <QStringList>

#include "mapio.hpp"
#include "robot_model.hpp"

#include "ros_bridge.hpp"

class QCheckBox;
class QComboBox;
class QPlainTextEdit;
class QProcess;
class QLabel;
class QProgressBar;
class QPushButton;
class QTimer;
class Map2DView;
class Cloud3DView;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<RosBridge> bridge);
  // Prueba automática (DASHBOARD_SELFTEST=<carpeta>): simula clics, guarda capturas y cierra
  void runSelfTest(const QString & dir);
  // Bloques C y D: carga de mapa, arrancar/parar y modelo (DASHBOARD_SELFTEST_CD=<carpeta> + DASHBOARD_CD_*)
  void runSelfTestCD(const QString & dir);

protected:
  void closeEvent(QCloseEvent * e) override;

private slots:
  void onFrame();      // ~30 Hz: refresca vistas si hay datos nuevos
  void onSecond();     // 1 Hz: timer, estadísticas, auto-export
  void onPrimary();    // START / PAUSE / RESUME
  void onReset();
  void onExportDone();
  void onMapLoaded();
  void onDeadman();    // 5 Hz, desde el hilo de la UI: si la UI se congela, el robot se corta

private:
  enum class Mission { Idle, Mapping, Paused };

  QWidget * buildHeader();
  QWidget * buildCard(const QString & title, const QString & hint, QWidget * view, QList<QPushButton *> tools);
  QWidget * buildSidebar();
  QWidget * buildNavCard();
  QWidget * buildLayersCard();
  QWidget * buildRobotCard();
  void onStart();
  void onStop();
  void onSaveMap();
  void loadMapFile(const QString & file);
  void runScript(QProcess * proc, const QString & script, const QStringList & args, const QString & what);
  void log(const QString & text);
  void updateModel();
  void updateNav();
  void updateApproval();
  QWidget * statBox(const QString & label, QLabel ** value);
  void setMission(Mission m);
  int elapsedSeconds() const;
  void startExport(const QString & kind);
  void toast(const QString & text, const QString & kind);
  void repolish(QWidget * w);

  std::shared_ptr<RosBridge> bridge_;
  Map2DView * map2d_ = nullptr;
  Cloud3DView * cloud3d_ = nullptr;

  // Encabezado
  QLabel * timer_ = nullptr, * timer_max_ = nullptr, * mode_chip_ = nullptr;
  QProgressBar * time_bar_ = nullptr;

  // Barra lateral
  QPushButton * btn_primary_ = nullptr, * btn_reset_ = nullptr, * btn_auto_ = nullptr;
  QList<QPushButton *> export_btns_;
  QLabel * st_pose_ = nullptr, * st_area_ = nullptr, * st_map_ = nullptr;
  QLabel * st_scan_ = nullptr, * st_grid_ = nullptr, * st_obj_ = nullptr;
  std::array<QLabel *, RosBridge::kStats> topic_dot_{}, topic_hz_{};
  QLabel * tf_dot_ = nullptr, * tf_text_ = nullptr;
  QLabel * toast_ = nullptr;
  QTimer * toast_timer_ = nullptr;

  // Estado de misión
  Mission mission_ = Mission::Idle;
  QElapsedTimer run_timer_;
  qint64 accum_ms_ = 0;
  QDateTime start_time_;
  bool auto_done_ = false;

  // Frecuencias
  std::array<uint64_t, RosBridge::kStats> last_count_{};
  QElapsedTimer hz_clock_;

  // Exportación en segundo plano
  QFutureWatcher<QStringList> export_watcher_;
  bool exporting_ = false;

  uint64_t seen_grid_ = ~0ull, seen_pose_ = ~0ull, seen_nav_ = ~0ull, seen_appr_ = ~0ull;

  // Navegación
  QPushButton * btn_goal_ = nullptr, * btn_init_ = nullptr, * btn_approve_ = nullptr;
  QCheckBox * chk_deadman_ = nullptr;
  QLabel * nav_state_ = nullptr, * nav_cmd_ = nullptr, * nav_real_ = nullptr;
  QLabel * nav_route_ = nullptr, * nav_sc_ = nullptr;
  bool sc_mode_ = false;
  std::string local_frame_;

  // Panel Robot: scripts en procesos aparte (la UI no se bloquea: el hombre muerto sale de ella)
  QComboBox * cb_map_ = nullptr, * cb_cable_ = nullptr, * cb_mode_ = nullptr;
  QPushButton * btn_start_ = nullptr, * btn_stop_ = nullptr, * btn_load_ = nullptr, * btn_savemap_ = nullptr;
  QPlainTextEdit * log_ = nullptr;
  QProcess * start_proc_ = nullptr, * stop_proc_ = nullptr, * save_proc_ = nullptr;
  QFutureWatcher<LoadedMap> load_watcher_;
  bool loading_ = false;
  QString map_yaml_;            // mapa cargado (.yaml local) que usa "Arrancar" con Mapa = Cargado
  bool loaded_active_ = false;  // el 2D muestra el mapa cargado en vez de /map

  // Modelo del robot
  std::unique_ptr<RobotModel> model_;
  uint64_t seen_joint_ = ~0ull, seen_model_pose_ = ~0ull;
  bool joints_stale_ = true;
};
