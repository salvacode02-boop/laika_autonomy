#include "main_window.hpp"

#include <cmath>

#include <QCloseEvent>
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
  split->addWidget(buildCard("Mapa 2D", QString::fromStdString(bridge_->stats[1].name), map2d_, {fit2d, follow2d}));
  split->addWidget(buildCard("Nube 3D", "arrastra: girar · clic der: mover · rueda: zoom",
    cloud3d_, {reset3d, top3d, follow3d, scan3d}));
  split->setSizes({1, 1});
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

  connect(&export_watcher_, &QFutureWatcher<QStringList>::finished, this, &MainWindow::onExportDone);

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
  for (int i = 0; i < 4; ++i) {
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
  if (gv != seen_grid_) {
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
  if (cloud3d_->isVisible() && cloud3d_->hasNewData()) {cloud3d_->update();}
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
  for (int i = 0; i < 4; ++i) {
    const uint64_t c = bridge_->stats[i].count.load();
    const double hz = (c - last_count_[i]) / dt;
    last_count_[i] = c;
    const int64_t last = bridge_->stats[i].last_ns.load();
    const double age = last ? (now - last) / 1e9 : 1e9;
    QString color = "#475569";  // nunca recibido
    if (last && age < 3.0) {color = "#22c55e";} else if (last) {color = "#f59e0b";}
    if (i == 3 && !last) {color = "#334155";}  // objetos: opcional
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
  if (exporting_) {
    toast("Espera a que termine la exportación antes de cerrar", "warn");
    e->ignore();
    return;
  }
  e->accept();
}
