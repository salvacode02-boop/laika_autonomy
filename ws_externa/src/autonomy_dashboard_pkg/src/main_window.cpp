#include "main_window.hpp"

#include <cmath>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QPlainTextEdit>
#include <QProcess>
#include <QRegularExpression>
#include <QCloseEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStyle>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>

#include "cloud3d_view.hpp"
#include "exporters.hpp"
#include "map2d_view.hpp"
#include "theme.hpp"

namespace {

QString mmss(int s)
{
  return QString("%1:%2").arg(s / 60, 2, 10, QChar('0')).arg(s % 60, 2, 10, QChar('0'));
}

QPushButton * toolButton(const QString & text, bool checkable = false)
{
  auto * b = new QPushButton(text);
  b->setObjectName("tool");
  b->setCheckable(checkable);
  b->setCursor(Qt::PointingHandCursor);
  return b;
}

QLabel * label(const QString & text, const char * obj)
{
  auto * l = new QLabel(text);
  l->setObjectName(obj);
  return l;
}

}  // namespace

MainWindow::MainWindow(std::shared_ptr<RosBridge> bridge)
: bridge_(std::move(bridge))
{
  setWindowTitle("RoboCup Rescue 2026 · Autonomy Dashboard");
  resize(1500, 900);
  setStyleSheet(theme::styleSheet());

  auto * root = new QWidget;
  root->setObjectName("root");
  auto * v = new QVBoxLayout(root);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  v->addWidget(buildHeader());

  auto * body = new QHBoxLayout;
  body->setContentsMargins(12, 12, 12, 12);
  body->setSpacing(12);

  // Vistas 2D y 3D lado a lado (redimensionables)
  map2d_ = new Map2DView;
  cloud3d_ = new Cloud3DView(bridge_.get());

  auto * fit2d = toolButton("Ajustar");
  auto * follow2d = toolButton("Seguir robot", true);
  connect(fit2d, &QPushButton::clicked, map2d_, &Map2DView::fit);
  connect(follow2d, &QPushButton::toggled, this, [this](bool on) {map2d_->setFollow(on);});

  btn_goal_ = toolButton("Meta", true);
  btn_init_ = toolButton("Pose inicial", true);
  btn_goal_->setToolTip("Clic = posición, arrastrar = orientación, soltar = publica /goal_pose");
  btn_init_->setToolTip("Clic = posición, arrastrar = orientación, soltar = publica /initialpose");
  // Excluyentes; sin ninguna activa el ratón hace pan/zoom
  connect(btn_goal_, &QPushButton::toggled, this, [this](bool on) {
      if (on) {btn_init_->setChecked(false);}
      map2d_->setTool(on ? Map2DView::Tool::Goal : (btn_init_->isChecked() ? Map2DView::Tool::InitPose : Map2DView::Tool::None));
    });
  connect(btn_init_, &QPushButton::toggled, this, [this](bool on) {
      if (on) {btn_goal_->setChecked(false);}
      map2d_->setTool(on ? Map2DView::Tool::InitPose : (btn_goal_->isChecked() ? Map2DView::Tool::Goal : Map2DView::Tool::None));
    });
  connect(map2d_, &Map2DView::poseChosen, this, [this](int tool, double x, double y, double yaw) {
      if (tool == static_cast<int>(Map2DView::Tool::Goal)) {
        bridge_->publishGoal(x, y, yaw);
        toast(QString("Meta publicada: %1, %2 m, %3°").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw * 180 / M_PI, 0, 'f', 0), "info");
      } else {
        bridge_->publishInitialPose(x, y, yaw);
        toast(QString("Pose inicial publicada: %1, %2 m, %3°").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw * 180 / M_PI, 0, 'f', 0), "info");
      }
      btn_goal_->setChecked(false);  // como RViz: una herramienta, un uso
      btn_init_->setChecked(false);
    });

  auto * reset3d = toolButton("Vista inicial");
  auto * top3d = toolButton("Planta", true);
  auto * follow3d = toolButton("Seguir robot", true);
  auto * scan3d = toolButton("Escaneo en vivo", true);
  scan3d->setChecked(true);
  connect(reset3d, &QPushButton::clicked, cloud3d_, &Cloud3DView::resetView);
  connect(top3d, &QPushButton::toggled, this, [this](bool on) {cloud3d_->setTopView(on);});
  connect(follow3d, &QPushButton::toggled, this, [this](bool on) {cloud3d_->setFollow(on);});
  connect(scan3d, &QPushButton::toggled, this, [this](bool on) {cloud3d_->setShowScan(on);});

  auto * split = new QSplitter(Qt::Horizontal);
  split->setHandleWidth(12);
  split->addWidget(buildCard("Mapa 2D", QString::fromStdString(bridge_->stats[1].name), map2d_, {btn_goal_, btn_init_, fit2d, follow2d}));
  split->addWidget(buildCard("Nube 3D", "arrastra: girar · clic der: mover · rueda: zoom",
    cloud3d_, {reset3d, top3d, follow3d, scan3d}));
  split->setSizes({1, 1});

  // Modelo del Go2 (bloque D): si no carga, quedan la flecha y el aviso
  model_ = std::make_unique<RobotModel>();
  {
    QElapsedTimer t;
    t.start();
    std::string err;
    if (model_->load(bridge_->model_dir + "/urdf/go2_description.urdf", bridge_->model_dir, &err)) {
      qInfo("Modelo del robot: %zu triangulos, %lld ms", model_->triangles(), static_cast<long long>(t.elapsed()));
      cloud3d_->setModel(model_.get());
    } else {
      qWarning("Modelo del robot no disponible: %s", err.c_str());
      model_.reset();
    }
  }
  body->addWidget(split, 1);
  body->addWidget(buildSidebar());
  v->addLayout(body, 1);
  setCentralWidget(root);

  // Timers
  auto * frame = new QTimer(this);
  connect(frame, &QTimer::timeout, this, &MainWindow::onFrame);
  frame->start(33);
  auto * sec = new QTimer(this);
  connect(sec, &QTimer::timeout, this, &MainWindow::onSecond);
  sec->start(1000);
  hz_clock_.start();
  // Hombre muerto: timer de la UI (no del hilo ROS) para que un congelamiento corte el robot
  auto * dead = new QTimer(this);
  connect(dead, &QTimer::timeout, this, &MainWindow::onDeadman);
  dead->start(200);

  connect(&export_watcher_, &QFutureWatcher<QStringList>::finished, this, &MainWindow::onExportDone);
  connect(&load_watcher_, &QFutureWatcher<LoadedMap>::finished, this, &MainWindow::onMapLoaded);

  start_time_ = QDateTime::currentDateTime();
  setMission(Mission::Idle);
  if (bridge_->autostart) {onPrimary();}
  onSecond();
}

// ---------------------------------------------------------------------------
// Construcción de la interfaz
// ---------------------------------------------------------------------------
QWidget * MainWindow::buildHeader()
{
  auto * h = new QWidget;
  h->setObjectName("header");
  auto * l = new QHBoxLayout(h);
  l->setContentsMargins(18, 10, 18, 10);
  l->setSpacing(14);

  auto * brand = new QVBoxLayout;
  brand->setSpacing(0);
  brand->addWidget(label("RoboCup Rescue 2026 · Autonomy", "brand"));
  brand->addWidget(label(QString("Equipo %1  ·  Misión %2")
    .arg(QString::fromStdString(bridge_->team), QString::fromStdString(bridge_->mission)), "brandSub"));
  l->addLayout(brand);
  l->addStretch();

  mode_chip_ = label("IDLE", "modeChip");
  l->addWidget(mode_chip_, 0, Qt::AlignVCenter);

  auto * tbox = new QVBoxLayout;
  tbox->setSpacing(4);
  auto * trow = new QHBoxLayout;
  trow->setSpacing(6);
  timer_ = label("00:00", "timer");
  timer_max_ = label("/ " + mmss(bridge_->max_time), "timerMax");
  trow->addWidget(timer_);
  trow->addWidget(timer_max_, 0, Qt::AlignBottom);
  tbox->addLayout(trow);
  time_bar_ = new QProgressBar;
  time_bar_->setTextVisible(false);
  time_bar_->setRange(0, std::max(1, bridge_->max_time));
  time_bar_->setFixedWidth(170);
  tbox->addWidget(time_bar_);
  l->addLayout(tbox);
  return h;
}

QWidget * MainWindow::buildCard(const QString & title, const QString & hint, QWidget * view, QList<QPushButton *> tools)
{
  auto * card = new QWidget;
  card->setObjectName("card");
  auto * v = new QVBoxLayout(card);
  v->setContentsMargins(10, 8, 10, 10);
  v->setSpacing(8);
  auto * top = new QHBoxLayout;
  top->setSpacing(6);
  top->addWidget(label(title, "cardTitle"));
  top->addWidget(label(hint, "cardHint"));
  top->addStretch();
  for (auto * t : tools) {top->addWidget(t);}
  v->addLayout(top);
  v->addWidget(view, 1);
  return card;
}

QWidget * MainWindow::statBox(const QString & text, QLabel ** value)
{
  auto * w = new QWidget;
  w->setObjectName("statBox");
  auto * v = new QVBoxLayout(w);
  v->setContentsMargins(10, 7, 10, 7);
  v->setSpacing(1);
  v->addWidget(label(text, "statLabel"));
  *value = label("—", "statValue");
  v->addWidget(*value);
  return w;
}

QWidget * MainWindow::buildSidebar()
{
  auto * side = new QWidget;
  side->setObjectName("side");
  side->setFixedWidth(320);
  auto * v = new QVBoxLayout(side);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(12);

  v->addWidget(buildRobotCard());
  v->addWidget(buildNavCard());
  v->addWidget(buildLayersCard());

  // --- Misión
  auto * mcard = new QWidget;
  mcard->setObjectName("card");
  auto * mv = new QVBoxLayout(mcard);
  mv->setContentsMargins(12, 10, 12, 12);
  mv->setSpacing(8);
  mv->addWidget(label("MISIÓN", "sideTitle"));
  btn_primary_ = new QPushButton;
  btn_primary_->setObjectName("primary");
  btn_primary_->setCursor(Qt::PointingHandCursor);
  connect(btn_primary_, &QPushButton::clicked, this, &MainWindow::onPrimary);
  mv->addWidget(btn_primary_);
  btn_reset_ = new QPushButton("Reiniciar misión");
  btn_reset_->setObjectName("danger");
  btn_reset_->setCursor(Qt::PointingHandCursor);
  connect(btn_reset_, &QPushButton::clicked, this, &MainWindow::onReset);
  mv->addWidget(btn_reset_);
  v->addWidget(mcard);

  // --- Datos
  auto * dcard = new QWidget;
  dcard->setObjectName("card");
  auto * dv = new QVBoxLayout(dcard);
  dv->setContentsMargins(12, 10, 12, 12);
  dv->setSpacing(8);
  dv->addWidget(label("DATOS", "sideTitle"));
  auto * grid = new QGridLayout;
  grid->setSpacing(6);
  grid->addWidget(statBox("Pose (x, y, θ)", &st_pose_), 0, 0, 1, 2);
  grid->addWidget(statBox("Área mapeada", &st_area_), 1, 0);
  grid->addWidget(statBox("Puntos 3D", &st_map_), 1, 1);
  grid->addWidget(statBox("Escaneo", &st_scan_), 2, 0);
  grid->addWidget(statBox("Grid 2D", &st_grid_), 2, 1);
  grid->addWidget(statBox("Objetos", &st_obj_), 3, 0, 1, 2);
  dv->addLayout(grid);

  // Estado de topics
  dv->addSpacing(4);
  dv->addWidget(label("TOPICS", "sideTitle"));
  auto * tg = new QGridLayout;
  tg->setHorizontalSpacing(8);
  tg->setVerticalSpacing(4);
  for (size_t i = 0; i < RosBridge::kStats; ++i) {
    topic_dot_[i] = new QLabel("●");
    topic_hz_[i] = label("—", "topicHz");
    tg->addWidget(topic_dot_[i], i, 0);
    tg->addWidget(label(QString::fromStdString(bridge_->stats[i].name), "topicName"), i, 1);
    tg->addWidget(topic_hz_[i], i, 2, Qt::AlignRight);
  }
  tf_dot_ = new QLabel("●");
  tf_text_ = label("—", "topicHz");
  tg->addWidget(tf_dot_, 4, 0);
  tg->addWidget(label(QString("TF %1 ← nube").arg(QString::fromStdString(bridge_->map_frame)), "topicName"), 4, 1);
  tg->addWidget(tf_text_, 4, 2, Qt::AlignRight);
  tg->setColumnStretch(1, 1);
  dv->addLayout(tg);
  v->addWidget(dcard);

  // --- Exportar
  auto * ecard = new QWidget;
  ecard->setObjectName("card");
  auto * ev = new QVBoxLayout(ecard);
  ev->setContentsMargins(12, 10, 12, 12);
  ev->setSpacing(6);
  ev->addWidget(label("EXPORTAR", "sideTitle"));
  auto add = [&](const QString & text, const QString & kind, const QString & tip) {
      auto * b = new QPushButton(text);
      b->setToolTip(tip);
      b->setCursor(Qt::PointingHandCursor);
      connect(b, &QPushButton::clicked, this, [this, kind] {startExport(kind);});
      export_btns_ << b;
      return b;
    };
  auto * row1 = new QHBoxLayout;
  row1->setSpacing(6);
  row1->addWidget(add("Quick map", "QUICK", "GeoTIFF + PLY + CSV (mapa rápido, < 5 min)"));
  row1->addWidget(add("Comprehensive", "COMPREHENSIVE", "GeoTIFF + PLY + CSV (mapa completo, > 15 min)"));
  ev->addLayout(row1);
  ev->addWidget(add("Nube 3D  ·  PLY", "PLY", "x y z rgb normales temp confidence"));
  ev->addWidget(add("Mapa 2D  ·  GeoTIFF", "GEOTIFF", "Mapa de /projected_map con trayectoria"));
  ev->addWidget(add("Objetos  ·  CSV", "CSV", "Lista de POIs detectados"));

  btn_auto_ = new QPushButton(QString("Auto-export a los %1").arg(mmss(bridge_->auto_export_time)));
  btn_auto_->setCheckable(true);
  btn_auto_->setCursor(Qt::PointingHandCursor);
  btn_auto_->setToolTip("Guarda todo automáticamente cuando el timer de misión llega a ese tiempo");
  btn_auto_->setChecked(bridge_->auto_export_enabled);  // antes de conectar: toast_ aún no existe
  connect(btn_auto_, &QPushButton::toggled, this, [this](bool on) {
      auto_done_ = false;
      toast(on ? "Auto-export activado" : "Auto-export desactivado", on ? "ok" : "warn");
    });
  ev->addWidget(btn_auto_);

  auto * open = new QPushButton("Abrir carpeta de mapas");
  open->setCursor(Qt::PointingHandCursor);
  open->setToolTip(QString::fromStdString(bridge_->output_dir));
  connect(open, &QPushButton::clicked, this, [this] {
      const QString dir = QString::fromStdString(bridge_->output_dir);
      QDir().mkpath(dir);
      QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
    });
  ev->addWidget(open);
  v->addWidget(ecard);

  // --- Mensajes
  toast_ = label("", "toast");
  toast_->setWordWrap(true);
  toast_->hide();
  v->addWidget(toast_);
  toast_timer_ = new QTimer(this);
  toast_timer_->setSingleShot(true);
  connect(toast_timer_, &QTimer::timeout, toast_, &QLabel::hide);

  v->addStretch();

  auto * scroll = new QScrollArea;
  scroll->setWidget(side);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setFixedWidth(332);
  return scroll;
}

// ---------------------------------------------------------------------------
// Actualización periódica
// ---------------------------------------------------------------------------
void MainWindow::onFrame()
{
  const uint64_t gv = bridge_->grid_version.load();
  if (gv != seen_grid_ && !loaded_active_) {  // con un mapa cargado en pantalla, /map no lo pisa
    seen_grid_ = gv;
    std::lock_guard<std::mutex> lk(bridge_->grid_mx);
    map2d_->setGrid(bridge_->grid);
  }
  const uint64_t pv = bridge_->pose_version.load();
  if (pv != seen_pose_) {
    seen_pose_ = pv;
    Pose2D p;
    std::vector<std::array<float, 3>> path;
    {
      std::lock_guard<std::mutex> lk(bridge_->pose_mx);
      p = bridge_->pose;
      path = bridge_->path;
    }
    map2d_->setPose(p);
    map2d_->setPath(std::move(path));
  }
  updateNav();
  updateApproval();
  updateModel();
  if (cloud3d_->isVisible() && cloud3d_->hasNewData()) {cloud3d_->update();}
}

// Cinemática del modelo con /joint_states (sin datos recientes: postura de pie) y silueta 2D
void MainWindow::updateModel()
{
  if (!model_) {return;}
  const uint64_t jv = bridge_->joint_version.load(), pv = bridge_->pose_version.load();
  const int64_t last = bridge_->joint_last_ns.load();
  const bool stale = !last || (std::chrono::steady_clock::now().time_since_epoch().count() - last) > 2000000000LL;
  if (jv == seen_joint_ && pv == seen_model_pose_ && stale == joints_stale_) {return;}
  seen_joint_ = jv;
  seen_model_pose_ = pv;
  joints_stale_ = stale;
  std::map<std::string, double> q;
  if (!stale) {
    std::lock_guard<std::mutex> lk(bridge_->joint_mx);
    q = bridge_->joints;
  }
  model_->setJoints(q);
  Pose2D p;
  {
    std::lock_guard<std::mutex> lk(bridge_->pose_mx);
    p = bridge_->pose;
  }
  if (p.valid) {
    map2d_->setSilhouette(model_->silhouette(RobotModel::poseMatrix(p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw)));
  }
  cloud3d_->update();
}

void MainWindow::updateNav()
{
  const uint64_t nv = bridge_->nav_version.load();
  if (nv != seen_nav_) {
    seen_nav_ = nv;
    NavData n;
    {
      std::lock_guard<std::mutex> lk(bridge_->nav_mx);
      n.cmd_vx = bridge_->nav.cmd_vx;
      n.cmd_wz = bridge_->nav.cmd_wz;
      n.bridge_state = bridge_->nav.bridge_state;
      n.plan = bridge_->nav.plan;
      n.local_plan = bridge_->nav.local_plan;
      n.footprint = bridge_->nav.footprint;
      local_frame_ = bridge_->nav.local_frame;
    }
    map2d_->setNavPaths(std::move(n.plan), std::move(n.local_plan));
    map2d_->setFootprint(std::move(n.footprint), bridge_->fixed_footprint);
    map2d_->setCmdVel(n.cmd_vx, n.cmd_wz);
    nav_cmd_->setText(QString("vx %1  wz %2").arg(n.cmd_vx, 0, 'f', 2).arg(n.cmd_wz, 0, 'f', 2));

    const QString st = QString::fromStdString(n.bridge_state);
    sc_mode_ = st.startsWith("SC:");
    QString col = "#64748b";  // IDLE y desconocido: gris
    if (st.startsWith("STOP")) {col = "#ef4444";}
    else if (st == "MOVING" || st == "SC:RUN") {col = "#22c55e";}
    else if (st == "ACTIVE" || st == "SC:APROBADA") {col = "#f59e0b";}
    nav_state_->setText(st.isEmpty() ? "sin puente" : st);
    nav_state_->setStyleSheet(QString("color:%1; font-weight:700;").arg(col));
    nav_sc_->setVisible(sc_mode_);
  }
  // Costmaps: solo se reconvierten a imagen cuando llegan (versión propia por cada uno)
  static uint64_t last_cm_global = ~0ull;
  const uint64_t g_cnt = bridge_->stats[4].count.load(), l_cnt = bridge_->stats[5].count.load();
  static uint64_t last_cm_local = ~0ull;
  if (g_cnt != last_cm_global) {
    last_cm_global = g_cnt;
    std::lock_guard<std::mutex> lk(bridge_->nav_mx);
    map2d_->setGlobalCostmap(bridge_->nav.global_cm);
  }
  if (l_cnt != last_cm_local) {
    last_cm_local = l_cnt;
    std::lock_guard<std::mutex> lk(bridge_->nav_mx);
    map2d_->setLocalCostmap(bridge_->nav.local_cm, bridge_->lookupMapFrom(bridge_->nav.local_frame));
  } else if (!local_frame_.empty()) {
    map2d_->setLocalTf(bridge_->lookupMapFrom(local_frame_));  // el mapa se corrige: el costmap local sigue a la TF
  }
  nav_real_->setText(QString("vx %1  wz %2").arg(bridge_->real_vx.load(), 0, 'f', 2).arg(bridge_->real_wz.load(), 0, 'f', 2));
}

void MainWindow::updateApproval()
{
  const uint64_t av = bridge_->appr_version.load();
  if (av == seen_appr_) {return;}
  seen_appr_ = av;
  ApprovalData a;
  {
    std::lock_guard<std::mutex> lk(bridge_->appr_mx);
    a = bridge_->appr;
  }
  btn_approve_->setEnabled(a.waiting);
  if (a.waiting) {
    nav_route_->setText(QString("Ruta nueva: %1 puntos, %2 m").arg(a.n).arg(a.length, 0, 'f', 2));
  } else if (!a.approved.empty()) {
    nav_route_->setText("Ruta aprobada");
  } else if (a.pending) {
    nav_route_->setText("Meta enviada, esperando ruta…");
  } else {
    nav_route_->setText("Sin meta");
  }
}

QWidget * MainWindow::buildRobotCard()
{
  auto * c = new QWidget;
  c->setObjectName("card");
  auto * v = new QVBoxLayout(c);
  v->setContentsMargins(12, 10, 12, 12);
  v->setSpacing(6);
  v->addWidget(label("ROBOT", "sideTitle"));
  auto * g = new QGridLayout;
  g->setHorizontalSpacing(8);
  g->setColumnStretch(1, 1);
  auto combo = [&](int row, const QString & name, const QStringList & items) {
      g->addWidget(label(name, "statLabel"), row, 0);
      auto * cb = new QComboBox;
      cb->addItems(items);
      g->addWidget(cb, row, 1);
      return cb;
    };
  cb_map_ = combo(0, "Mapa", {"En vivo (SLAM)", "Cargado: (ninguno)"});
  cb_cable_ = combo(1, "Cable", {"Con cable", "Sin cable"});
  cb_mode_ = combo(2, "Modo", {"Prueba", "Armado"});
  v->addLayout(g);
  connect(cb_map_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i) {
      if (i == 0 && loaded_active_) {loaded_active_ = false; seen_grid_ = ~0ull; cloud3d_->setStaticCloud({});}  // vuelve a /map
    });

  auto * row = new QHBoxLayout;
  btn_start_ = new QPushButton("Arrancar");
  btn_start_->setObjectName("primary");
  btn_stop_ = new QPushButton("Parar");
  btn_stop_->setObjectName("danger");
  for (auto * b : {btn_start_, btn_stop_}) {b->setCursor(Qt::PointingHandCursor); row->addWidget(b);}
  v->addLayout(row);
  btn_load_ = new QPushButton("Cargar mapa…");
  btn_load_->setToolTip("Mapas .yaml/.pgm, o GeoTIFF y PLY exportados por este dashboard");
  btn_savemap_ = new QPushButton("Guardar mapa (.pgm/.yaml)");
  btn_savemap_->setToolTip("scripts/guardar_mapa.sh: map_saver_cli sobre /map -> maps/<nombre>");
  for (auto * b : {btn_load_, btn_savemap_}) {b->setCursor(Qt::PointingHandCursor); v->addWidget(b);}
  log_ = new QPlainTextEdit;
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(300);
  log_->setFixedHeight(130);
  log_->setPlaceholderText("Salida de los scripts…");
  v->addWidget(log_);

  start_proc_ = new QProcess(this);
  stop_proc_ = new QProcess(this);
  save_proc_ = new QProcess(this);
  connect(btn_start_, &QPushButton::clicked, this, &MainWindow::onStart);
  connect(btn_stop_, &QPushButton::clicked, this, &MainWindow::onStop);
  connect(btn_load_, &QPushButton::clicked, this, [this] {
      const QString dir = QString::fromStdString(bridge_->laika_dir) + "/maps";
      const QString f = QFileDialog::getOpenFileName(this, "Cargar mapa", dir, "Mapas (*.yaml *.tiff *.tif *.ply)");
      if (!f.isEmpty()) {loadMapFile(f);}
    });
  connect(btn_savemap_, &QPushButton::clicked, this, &MainWindow::onSaveMap);
  return c;
}

void MainWindow::log(const QString & text)
{
  for (const QString & l : text.split('\n')) {
    const QString t = l.trimmed();
    if (!t.isEmpty()) {log_->appendPlainText(t);}
  }
}

// Los scripts corren en procesos aparte; stdout+stderr van al panel de mensajes.
void MainWindow::runScript(QProcess * proc, const QString & script, const QStringList & args, const QString & what)
{
  proc->disconnect(this);
  proc->setProcessChannelMode(QProcess::MergedChannels);
  proc->setWorkingDirectory(QString::fromStdString(bridge_->laika_dir));
  connect(proc, &QProcess::readyReadStandardOutput, this, [this, proc] {log(QString::fromUtf8(proc->readAllStandardOutput()));});
  connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this, what](int code, QProcess::ExitStatus) {
      log(QString("[%1] terminó (código %2)").arg(what).arg(code));
      toast(QString("%1: %2").arg(what, code == 0 ? "listo" : QString("falló (código %1), ver mensajes").arg(code)), code == 0 ? "ok" : "err");
    });
  connect(proc, &QProcess::errorOccurred, this, [this, what](QProcess::ProcessError) {
      log(QString("[%1] no se pudo ejecutar").arg(what));
    });
  log(QString("[%1] %2 %3").arg(what, QFileInfo(script).fileName(), args.join(' ')));
  proc->start("bash", QStringList{script} + args);
  proc->closeWriteChannel();  // que ssh/scp no esperen stdin
}

void MainWindow::onStart()
{
  if (start_proc_->state() != QProcess::NotRunning) {
    toast("Arrancar ya está en curso", "warn");
    return;
  }
  const bool armed = cb_mode_->currentIndex() == 1, loaded = cb_map_->currentIndex() == 1;
  if (loaded && map_yaml_.isEmpty()) {
    toast("Primero carga un mapa (Cargar mapa…) o elige Mapa = En vivo", "warn");
    return;
  }
  if (armed) {
    const auto r = QMessageBox::warning(this, "Modo ARMADO",
      "El robot podrá caminar solo.\n\nCuando presiones R2+X en el control y apruebes una ruta, el puente "
      "mandará movimiento real al robot.\nTenlo despejado, con el control en la mano (L2+Y lo detiene).\n\n"
      "¿Arrancar en modo ARMADO?", QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (r != QMessageBox::Yes) {return;}
  }
  QStringList a;
  if (loaded) {a << "--mapa" << map_yaml_;} else {a << "--slam";}
  if (cb_cable_->currentIndex() == 1) {a << "--sin-cable";}
  if (!armed) {a << "--prueba";}
  a << "--sin-attach" << "--sin-rviz";  // sin tmux attach ni RViz: el dashboard es la interfaz
  runScript(start_proc_, QString::fromStdString(bridge_->laika_dir) + "/scripts/laika_fase4.sh", a, "Arrancar");
}

void MainWindow::onStop()
{
  if (stop_proc_->state() != QProcess::NotRunning) {
    toast("Parar ya está en curso", "warn");
    return;
  }
  runScript(stop_proc_, QString::fromStdString(bridge_->laika_dir) + "/scripts/laika_fase4_parar.sh", {}, "Parar");
}

void MainWindow::onSaveMap()
{
  if (save_proc_->state() != QProcess::NotRunning) {return;}
  bool ok = false;
  const QString name = QInputDialog::getText(this, "Guardar mapa", "Nombre (sin extensión):", QLineEdit::Normal, "mapa", &ok);
  if (!ok) {return;}
  if (!QRegularExpression("^[A-Za-z0-9_.-]+$").match(name).hasMatch()) {
    toast("Nombre no válido: usa letras, números, _ - .", "err");
    return;
  }
  const QString file = QString::fromStdString(bridge_->laika_dir) + "/maps/" + name + ".yaml";
  if (QFileInfo::exists(file) &&
    QMessageBox::question(this, "Guardar mapa", name + ".yaml ya existe. ¿Sobrescribir?") != QMessageBox::Yes) {return;}
  runScript(save_proc_, QString::fromStdString(bridge_->laika_dir) + "/scripts/guardar_mapa.sh", {name}, "Guardar mapa");
}

// Convierte (hilo aparte) el archivo elegido a .pgm+.yaml en maps/cargados/ y lo muestra en 2D (sin publicarlo a ROS)
void MainWindow::loadMapFile(const QString & file)
{
  if (loading_) {return;}
  PlyParams pp;
  pp.res = bridge_->ply_res;
  pp.floor_percentile = bridge_->ply_floor_pct;
  pp.band_min = bridge_->ply_band_min;
  pp.band_max = bridge_->ply_band_max;
  pp.floor_tol = bridge_->ply_floor_tol;
  pp.min_points = bridge_->ply_min_points;
  loading_ = true;
  btn_load_->setEnabled(false);
  log("[Cargar] " + file);
  toast("Cargando mapa…", "info");
  const QString out = QString::fromStdString(bridge_->laika_dir) + "/maps/cargados";
  load_watcher_.setFuture(QtConcurrent::run([file, out, pp] {return loadMap(file, out, pp);}));
}

void MainWindow::onMapLoaded()
{
  loading_ = false;
  btn_load_->setEnabled(true);
  const LoadedMap r = load_watcher_.result();
  if (!r.error.isEmpty()) {
    log("[Cargar] ✗ " + r.error);
    toast("No se cargó el mapa: " + r.error, "err");
    return;
  }
  map_yaml_ = r.yaml;
  loaded_active_ = true;
  map2d_->setGrid(r.grid);
  map2d_->fit();
  cloud3d_->setStaticCloud(r.cloud);
  cb_map_->setItemText(1, "Cargado: " + QFileInfo(r.yaml).completeBaseName());
  cb_map_->setCurrentIndex(1);
  log(QString("[Cargar] ✓ %1 · %2×%3 celdas a %4 m · %5").arg(r.note).arg(r.grid.width).arg(r.grid.height)
    .arg(r.grid.res).arg(r.yaml));
  toast("Mapa cargado: " + QFileInfo(r.yaml).fileName() + " (listo para Arrancar con Mapa = Cargado)", "ok");
}

void MainWindow::onDeadman()
{
  if (chk_deadman_->isChecked() && !sc_mode_) {bridge_->publishDeadman();}
}

QWidget * MainWindow::buildNavCard()
{
  auto * c = new QWidget;
  c->setObjectName("card");
  auto * v = new QVBoxLayout(c);
  v->setContentsMargins(12, 10, 12, 12);
  v->setSpacing(6);
  v->addWidget(label("NAVEGACIÓN", "sideTitle"));
  auto * g = new QGridLayout;
  g->setHorizontalSpacing(8);
  g->addWidget(label("Puente", "statLabel"), 0, 0);
  nav_state_ = label("sin puente", "statValue");
  g->addWidget(nav_state_, 0, 1, Qt::AlignRight);
  g->addWidget(label("Comandada (/cmd_vel)", "statLabel"), 1, 0);
  nav_cmd_ = label("—", "topicHz");
  g->addWidget(nav_cmd_, 1, 1, Qt::AlignRight);
  g->addWidget(label("Real (odometría)", "statLabel"), 2, 0);
  nav_real_ = label("—", "topicHz");
  g->addWidget(nav_real_, 2, 1, Qt::AlignRight);
  v->addLayout(g);
  nav_route_ = label("Sin meta", "topicName");
  nav_route_->setWordWrap(true);
  v->addWidget(nav_route_);
  {  // Velocidad de ruta: el puente escala la salida de Nav2 (tope 0.3) a este valor
    auto * row = new QHBoxLayout;
    row->addWidget(label("Velocidad de ruta", "topicName"));
    auto * spd = new QDoubleSpinBox;
    spd->setRange(0.25, 0.50);  // < 0.25 el Go2 no da pasos; 0.5 es el tope del puente
    spd->setSingleStep(0.05);
    spd->setDecimals(2);
    spd->setValue(0.30);
    spd->setSuffix(" m/s");
    spd->setToolTip("Velocidad a la que el robot recorre la ruta (0.25–0.50 m/s). Más rápido = más distancia para frenar con L2+Y.");
    connect(spd, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double v) {bridge_->setRouteSpeed(v);});
    row->addWidget(spd);
    v->addLayout(row);
  }
  btn_approve_ = new QPushButton("Aprobar ruta");
  btn_approve_->setObjectName("primary");
  btn_approve_->setCursor(Qt::PointingHandCursor);
  btn_approve_->setEnabled(false);
  connect(btn_approve_, &QPushButton::clicked, this, [this] {
      bridge_->approve();
      toast(sc_mode_ ? "Aprobada: desconecta el cable, espera 10 s y presiona R2+X." : "Ruta aprobada", "ok");
      seen_appr_ = ~0ull;
      updateApproval();
    });
  v->addWidget(btn_approve_);
  chk_deadman_ = new QCheckBox("Hombre muerto (modo con cable)");
  chk_deadman_->setChecked(true);
  chk_deadman_->setToolTip("Publica /laika/deadman a 5 Hz desde la UI. Cerrar o congelar el dashboard detiene el robot.");
  v->addWidget(chk_deadman_);
  nav_sc_ = label("SIN CABLE: cerrar el dashboard NO detiene el robot; usa L2+Y", "toast");
  nav_sc_->setProperty("kind", "err");
  nav_sc_->setWordWrap(true);
  nav_sc_->hide();
  v->addWidget(nav_sc_);
  return c;
}

QWidget * MainWindow::buildLayersCard()
{
  auto * c = new QWidget;
  c->setObjectName("card");
  auto * v = new QVBoxLayout(c);
  v->setContentsMargins(12, 10, 12, 12);
  v->setSpacing(4);
  v->addWidget(label("CAPAS 2D", "sideTitle"));
  const struct {Map2DView::Layer l; const char * name; bool on;} defs[] = {
    {Map2DView::kGlobalCm, "Costmap global", false}, {Map2DView::kLocalCm, "Costmap local", true},
    {Map2DView::kPlan, "Ruta global (/plan)", true}, {Map2DView::kLocalPlan, "Ruta local (/local_plan)", true},
    {Map2DView::kFootprint, "Huella", true}, {Map2DView::kCmdVel, "Flecha de /cmd_vel", true}};
  for (const auto & d : defs) {
    auto * k = new QCheckBox(d.name);
    k->setChecked(d.on);
    map2d_->setLayer(d.l, d.on);
    connect(k, &QCheckBox::toggled, this, [this, l = d.l](bool on) {map2d_->setLayer(l, on);});
    v->addWidget(k);
  }
  return c;
}

void MainWindow::onSecond()
{
  // Timer de misión
  const int el = elapsedSeconds();
  bridge_->elapsed_s = el;
  timer_->setText(mmss(el));
  time_bar_->setValue(std::min(el, bridge_->max_time));
  const int remaining = bridge_->max_time - el;
  const QString warn = remaining <= 0 ? "2" : (remaining <= 120 ? "1" : "0");
  if (time_bar_->property("warn").toString() != warn) {
    time_bar_->setProperty("warn", warn);
    repolish(time_bar_);
    timer_->setStyleSheet(warn == "2" ? "color:#f87171;" : (warn == "1" ? "color:#fbbf24;" : ""));
  }

  // Auto-export
  if (btn_auto_->isChecked() && !auto_done_ && mission_ != Mission::Idle && el >= bridge_->auto_export_time) {
    auto_done_ = true;
    startExport("AUTO");
  }

  // Datos
  Pose2D p;
  {
    std::lock_guard<std::mutex> lk(bridge_->pose_mx);
    p = bridge_->pose;
  }
  st_pose_->setText(p.valid ? QString("%1, %2 m · %3°")
    .arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2).arg(p.yaw * 180.0 / M_PI, 0, 'f', 0) : "sin odometría");
  {
    std::lock_guard<std::mutex> lk(bridge_->grid_mx);
    const auto & g = bridge_->grid;
    if (g.width > 0) {
      st_area_->setText(QString("%1 m²").arg(g.known * g.res * g.res, 0, 'f', 1));
      st_grid_->setText(QString("%1×%2").arg(g.width).arg(g.height));
      st_grid_->setToolTip(QString("%1 m/celda · libre %2 · ocupado %3").arg(g.res).arg(g.free).arg(g.occupied));
    } else {
      st_area_->setText("—");
      st_grid_->setText("—");
    }
  }
  size_t map_pts;
  {
    std::lock_guard<std::mutex> lk(bridge_->cloud_mx);
    map_pts = bridge_->map_xyz.size() / 3;
  }
  st_map_->setText(map_pts >= 1000000 ? QString("%1 M").arg(map_pts / 1e6, 0, 'f', 2)
                                      : QString::number(map_pts / 1000.0, 'f', 1) + " k");
  st_map_->setStyleSheet(bridge_->map_full ? "color:#fbbf24;" : "");
  st_scan_->setText(QString::number(bridge_->last_scan_points.load() / 1000.0, 'f', 1) + " k");
  {
    std::lock_guard<std::mutex> lk(bridge_->poi_mx);
    st_obj_->setText(QString::number(bridge_->pois.size()));
  }

  // Frecuencia de cada topic
  const double dt = std::max(0.001, hz_clock_.restart() / 1000.0);
  const int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
  for (size_t i = 0; i < RosBridge::kStats; ++i) {
    const uint64_t c = bridge_->stats[i].count.load();
    const double hz = (c - last_count_[i]) / dt;
    last_count_[i] = c;
    const int64_t last = bridge_->stats[i].last_ns.load();
    const double age = last ? (now - last) / 1e9 : 1e9;
    QString color = "#475569";  // nunca recibido
    if (last && age < 3.0) {color = "#22c55e";} else if (last) {color = "#f59e0b";}
    if ((i == 3 || i >= 4) && !last) {color = "#334155";}  // objetos y navegación: opcionales
    topic_dot_[i]->setStyleSheet("color:" + color + "; font-size:11px;");
    topic_hz_[i]->setText(!last ? "sin datos" : (age < 3.0 ? QString("%1 Hz").arg(hz, 0, 'f', 1)
                                                           : QString("hace %1 s").arg(int(age))));
  }
  const bool cloud_alive = bridge_->stats[0].last_ns.load() != 0;
  const bool tf = bridge_->tf_ok.load();
  tf_dot_->setStyleSheet(QString("color:%1; font-size:11px;").arg(!cloud_alive ? "#475569" : (tf ? "#22c55e" : "#ef4444")));
  tf_text_->setText(!cloud_alive ? "—" : (tf ? "ok" : "falta"));
}

// ---------------------------------------------------------------------------
// Misión
// ---------------------------------------------------------------------------
int MainWindow::elapsedSeconds() const
{
  qint64 ms = accum_ms_;
  if (mission_ == Mission::Mapping) {ms += run_timer_.elapsed();}
  return static_cast<int>(ms / 1000);
}

void MainWindow::setMission(Mission m)
{
  if (mission_ == Mission::Mapping && m != Mission::Mapping) {accum_ms_ += run_timer_.elapsed();}
  if (m == Mission::Mapping) {run_timer_.restart();}
  mission_ = m;
  bridge_->mapping_active = (m == Mission::Mapping);

  QString mode, text, state;
  switch (m) {
    case Mission::Idle: mode = "IDLE"; text = "▶  START MAPPING"; state = "start"; break;
    case Mission::Mapping: mode = "MAPPING"; text = "❚❚  PAUSE"; state = "pause"; break;
    case Mission::Paused: mode = "PAUSED"; text = "▶  RESUME"; state = "start"; break;
  }
  mode_chip_->setText(mode);
  mode_chip_->setProperty("mode", mode);
  repolish(mode_chip_);
  btn_primary_->setText(text);
  btn_primary_->setProperty("state", state);
  repolish(btn_primary_);
}

void MainWindow::onPrimary()
{
  switch (mission_) {
    case Mission::Idle:
      accum_ms_ = 0;
      auto_done_ = false;
      start_time_ = QDateTime::currentDateTime();
      setMission(Mission::Mapping);
      toast("Mapeo iniciado: acumulando nube 3D y trayectoria", "ok");
      break;
    case Mission::Mapping:
      setMission(Mission::Paused);
      toast("Mapeo en pausa", "warn");
      break;
    case Mission::Paused:
      setMission(Mission::Mapping);
      toast("Mapeo reanudado", "ok");
      break;
  }
  onSecond();
}

void MainWindow::onReset()
{
  const auto r = QMessageBox::question(this, "Reiniciar misión",
    "¿Reiniciar la misión?\nSe borran la nube 3D, la trayectoria, los objetos y el timer.\n"
    "También se reinicia el mapa de octomap.");
  if (r != QMessageBox::Yes) {return;}
  setMission(Mission::Idle);
  accum_ms_ = 0;
  auto_done_ = false;
  bridge_->clearMission();
  bridge_->requestOctomapReset();
  start_time_ = QDateTime::currentDateTime();
  onSecond();
  cloud3d_->update();
  toast("Misión reiniciada", "info");
}

// ---------------------------------------------------------------------------
// Exportación (en un hilo aparte para no congelar la interfaz)
// ---------------------------------------------------------------------------
void MainWindow::startExport(const QString & kind)
{
  if (exporting_) {
    toast("Ya se está exportando, espera un momento…", "warn");
    return;
  }

  auto snap = std::make_shared<ExportSnapshot>();
  snap->team = QString::fromStdString(bridge_->team);
  snap->mission = QString::fromStdString(bridge_->mission);
  snap->start_time = start_time_;
  snap->px_per_cell = bridge_->geotiff_px_per_cell;
  const bool need_ply = kind != "GEOTIFF" && kind != "CSV";
  if (need_ply) {
    std::lock_guard<std::mutex> lk(bridge_->cloud_mx);
    snap->map_xyz = bridge_->map_xyz;
    // Sin START todavía: usa al menos el último escaneo
    if (snap->map_xyz.empty()) {snap->map_xyz = bridge_->scan_xyz;}
  }
  {
    std::lock_guard<std::mutex> lk(bridge_->grid_mx);
    snap->grid = bridge_->grid;
  }
  {
    std::lock_guard<std::mutex> lk(bridge_->pose_mx);
    snap->path = bridge_->path;
    snap->pose = bridge_->pose;
  }
  {
    std::lock_guard<std::mutex> lk(bridge_->poi_mx);
    snap->pois = bridge_->pois;
  }

  const QString dir = QString::fromStdString(bridge_->output_dir);
  const QString stamp = QDateTime::currentDateTime().toString("HH-mm-ss");
  QString base = QString("RoboCup2026-%1-%2").arg(snap->team, snap->mission);
  if (kind == "QUICK" || kind == "COMPREHENSIVE" || kind == "AUTO") {base += "-" + kind;}
  base += "-" + stamp;
  const QString prefix = QDir(dir).filePath(base);

  exporting_ = true;
  for (auto * b : export_btns_) {b->setEnabled(false);}
  toast("Exportando…", "info");

  export_watcher_.setFuture(QtConcurrent::run([snap, kind, dir, prefix]() {
      QStringList out;
      QDir().mkpath(dir);
      const bool all = kind == "QUICK" || kind == "COMPREHENSIVE" || kind == "AUTO";
      auto run = [&](const QString & path, QString (*fn)(const QString &, const ExportSnapshot &)) {
          const QString err = fn(path, *snap);
          out << (err.isEmpty() ? "✓ " + QFileInfo(path).fileName() : "✗ " + err);
        };
      if (all || kind == "GEOTIFF") {run(prefix + ".tiff", writeGeoTiff);}
      if (all || kind == "PLY") {run(prefix + "-3D.ply", writePly);}
      if (all || kind == "CSV") {run(prefix + "-pois.csv", writePoisCsv);}
      return out;
    }));
}

void MainWindow::onExportDone()
{
  exporting_ = false;
  for (auto * b : export_btns_) {b->setEnabled(true);}
  const QStringList res = export_watcher_.result();
  const bool any_err = std::any_of(res.begin(), res.end(), [](const QString & s) {return s.startsWith("✗");});
  toast(res.join("\n") + "\n→ " + QString::fromStdString(bridge_->output_dir), any_err ? "err" : "ok");
}

// ---------------------------------------------------------------------------
void MainWindow::toast(const QString & text, const QString & kind)
{
  toast_->setText(text);
  toast_->setProperty("kind", kind);
  repolish(toast_);
  toast_->show();
  toast_timer_->start(kind == "err" ? 15000 : 7000);
}

void MainWindow::repolish(QWidget * w)
{
  w->style()->unpolish(w);
  w->style()->polish(w);
}

void MainWindow::closeEvent(QCloseEvent * e)
{
  if (start_proc_->state() != QProcess::NotRunning || stop_proc_->state() != QProcess::NotRunning ||
    save_proc_->state() != QProcess::NotRunning || loading_)
  {
    toast("Espera a que terminen Arrancar/Parar/Guardar/Cargar antes de cerrar", "warn");
    e->ignore();
    return;
  }
  if (exporting_) {
    toast("Espera a que termine la exportación antes de cerrar", "warn");
    e->ignore();
    return;
  }
  e->accept();
}

// ---------------------------------------------------------------------------
// Prueba automática: eventos de ratón reales sobre Map2DView + capturas de la ventana
// ---------------------------------------------------------------------------
void MainWindow::runSelfTest(const QString & dir)
{
  QDir().mkpath(dir);
  auto shot = [this, dir](const QString & n) {grab().save(QDir(dir).filePath(n + ".png"));};
  // Arrastre con la herramienta activa: (x,y) mundo -> pantalla; mantiene la flecha para la captura
  auto drag = [this](double x, double y, double x2, double y2, const QString & shot_name, const QString & dir_) {
      const QPointF a = map2d_->screenAt(x, y), b = map2d_->screenAt(x2, y2);
      QMouseEvent press(QEvent::MouseButtonPress, a, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
      QApplication::sendEvent(map2d_, &press);
      QMouseEvent move(QEvent::MouseMove, b, Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
      QApplication::sendEvent(map2d_, &move);
      map2d_->grab().save(QDir(dir_).filePath(shot_name + ".png"));
      QMouseEvent rel(QEvent::MouseButtonRelease, b, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
      QApplication::sendEvent(map2d_, &rel);
    };
  const QString goal_env = qEnvironmentVariable("DASHBOARD_SELFTEST_GOAL", "1.0,0.0,1.0,0.6");  // x,y,x2,y2 en map
  const auto g = goal_env.split(',');
  const double gx = g[0].toDouble(), gy = g[1].toDouble(), gx2 = g[2].toDouble(), gy2 = g[3].toDouble();

  QTimer::singleShot(4000, this, [=] {
      for (auto * k : findChildren<QCheckBox *>()) {if (k->text() == "Costmap global") {k->setChecked(true);}}
      const QPointF c = map2d_->screenAt(0.5, 0.1);  // acerca el zoom sobre el robot
      QWheelEvent w(c, map2d_->mapToGlobal(c.toPoint()), QPoint(), QPoint(0, 450), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
      QApplication::sendEvent(map2d_, &w);
      shot("01_inicio");
      btn_goal_->click();
      drag(gx, gy, gx2, gy2, "02_arrastrando_meta", dir);
  });
  QTimer::singleShot(7000, this, [=] {shot("03_meta_ruta");});
  QTimer::singleShot(8000, this, [=] {
      if (btn_approve_->isEnabled()) {btn_approve_->click();}
      else {qWarning("SELFTEST: el boton Aprobar no se habilito");}
  });
  QTimer::singleShot(10000, this, [=] {
      shot("04_aprobada");
      btn_init_->click();
      drag(0.0, 0.0, 0.5, 0.0, "05_arrastrando_pose", dir);
  });
  QTimer::singleShot(12000, this, [=] {shot("06_final"); QApplication::quit();});
}

// ---------------------------------------------------------------------------
// Prueba automática de los bloques C y D. Variables:
//   DASHBOARD_CD_LOAD=<archivo>   carga ese mapa (como "Cargar mapa…")
//   DASHBOARD_CD_RUN=live|loaded  Arrancar en Prueba + Con cable (nunca Armado), espera, captura y Parar
//   DASHBOARD_CD_WAIT=<s>         espera con el robot arrancado (45 por defecto)
//   DASHBOARD_CD_EXPORT=1         exporta GeoTIFF y PLY (nombres fijos en maps/dashboard) antes de salir
// ---------------------------------------------------------------------------
void MainWindow::runSelfTestCD(const QString & dir)
{
  QDir().mkpath(dir);
  auto shot = [this, dir](const QString & n) {grab().save(QDir(dir).filePath(n + ".png"));};
  const QString load = qEnvironmentVariable("DASHBOARD_CD_LOAD"), run = qEnvironmentVariable("DASHBOARD_CD_RUN");
  const int wait_s = qEnvironmentVariable("DASHBOARD_CD_WAIT", "45").toInt();
  int t = 3000;
  QTimer::singleShot(t, this, [=] {
      shot("c01_inicio");
      if (!load.isEmpty()) {loadMapFile(load);}
    });
  t += 6000;
  QTimer::singleShot(t, this, [=] {
      shot("c02_mapa_cargado");
      if (run == "live") {cb_map_->setCurrentIndex(0);} else if (run == "loaded") {cb_map_->setCurrentIndex(1);}
      cb_cable_->setCurrentIndex(0);
      cb_mode_->setCurrentIndex(0);  // Prueba: la autoprueba nunca arma el robot
      if (!run.isEmpty()) {btn_start_->click();}
    });
  t += wait_s * 1000;
  QTimer::singleShot(t, this, [=] {
      // Acerca ambas vistas al robot (modelo y silueta)
      cloud3d_->setFollow(true);
      QWheelEvent w3(QPointF(300, 300), QPointF(300, 300), QPoint(), QPoint(0, 1900), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
      QApplication::sendEvent(cloud3d_, &w3);
      if (qEnvironmentVariableIsSet("DASHBOARD_CD_CAM")) {
        const auto c = qEnvironmentVariable("DASHBOARD_CD_CAM").split(',');  // yaw,pitch,dist
        cloud3d_->setCamera(c[0].toFloat(), c[1].toFloat(), c[2].toFloat());
      }
      map2d_->setFollow(true);
      QWheelEvent w2(QPointF(300, 300), QPointF(300, 300), QPoint(), QPoint(0, 900), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
      QApplication::sendEvent(map2d_, &w2);
    });
  t += 1500;
  QTimer::singleShot(t, this, [=] {
      shot("c03_corriendo");
      if (model_ && qEnvironmentVariableIsSet("DASHBOARD_CD_DEBUG")) {
        for (size_t i = 0; i < model_->linkCount(); ++i) {
          const auto c = model_->linkPose(i).column(3);
          qInfo("link %zu: %.3f %.3f %.3f", i, c.x(), c.y(), c.z());
        }
      }
      if (qEnvironmentVariableIsSet("DASHBOARD_CD_EXPORT")) {
        const QString d = QString::fromStdString(bridge_->laika_dir) + "/maps/dashboard/";
        ExportSnapshot s;
        s.team = "Laika"; s.mission = "selftest"; s.start_time = start_time_;
        s.px_per_cell = bridge_->geotiff_px_per_cell;
        { std::lock_guard<std::mutex> lk(bridge_->cloud_mx); s.map_xyz = bridge_->map_xyz; if (s.map_xyz.empty()) {s.map_xyz = bridge_->scan_xyz;} }
        { std::lock_guard<std::mutex> lk(bridge_->grid_mx); s.grid = bridge_->grid; }
        { std::lock_guard<std::mutex> lk(bridge_->pose_mx); s.path = bridge_->path; s.pose = bridge_->pose; }
        writePgmYaml(d + "selftest_orig", s.grid);  // referencia para comparar con lo que decodifica el GeoTIFF
        qInfo("EXPORT tiff: '%s'  ply: '%s'", qPrintable(writeGeoTiff(d + "selftest.tiff", s)), qPrintable(writePly(d + "selftest-3D.ply", s)));
      }
      if (!run.isEmpty()) {btn_stop_->click();}
    });
  // Espera a que Parar termine (hasta 120 s) y sale
  auto * poll = new QTimer(this);
  QTimer::singleShot(t + 2000, this, [=] {
      poll->start(1000);
    });
  connect(poll, &QTimer::timeout, this, [=] {
      if (stop_proc_->state() == QProcess::NotRunning || run.isEmpty()) {shot("c04_final"); QApplication::quit();}
    });
}
