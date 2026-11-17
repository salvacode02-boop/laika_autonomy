#include "map2d_view.hpp"

#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include "theme.hpp"

Map2DView::Map2DView(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);
  setMinimumSize(300, 240);
  setAttribute(Qt::WA_OpaquePaintEvent);
}

void Map2DView::setGrid(const GridData & g)
{
  if (g.width == 0 || g.height == 0) {return;}
  // Tabla de colores: desconocido transparente, libre oscuro, ocupado claro
  static QRgb lut[256];
  static bool lut_ready = false;
  if (!lut_ready) {
    const QColor fr = theme::kMapFree, oc = theme::kMapOcc;
    for (int v = -128; v < 128; ++v) {
      QRgb c = qRgba(0, 0, 0, 0);
      if (v >= 0) {
        const double t = std::clamp((v - 20) / 50.0, 0.0, 1.0);
        c = qRgb(fr.red() + t * (oc.red() - fr.red()),
                 fr.green() + t * (oc.green() - fr.green()),
                 fr.blue() + t * (oc.blue() - fr.blue()));
      }
      lut[static_cast<uint8_t>(v)] = c;
    }
    lut_ready = true;
  }

  if (img_.width() != g.width || img_.height() != g.height) {
    img_ = QImage(g.width, g.height, QImage::Format_ARGB32_Premultiplied);
  }
  for (int r = 0; r < g.height; ++r) {
    auto * line = reinterpret_cast<QRgb *>(img_.scanLine(r));
    const int8_t * src = &g.data[static_cast<size_t>(r) * g.width];
    for (int c = 0; c < g.width; ++c) {line[c] = lut[static_cast<uint8_t>(src[c])];}
  }
  res_ = g.res;
  ox_ = g.ox;
  oy_ = g.oy;
  gw_ = g.width;
  gh_ = g.height;
  if (!user_moved_ && !follow_) {fit();}
  update();
}

namespace {

// Costo Nav2 (0-100, -1 desconocido) -> color. Libre y desconocido transparentes,
// inflación en degradado azul->amarillo, inscrito magenta, letal rojo.
void costmapToImage(const GridData & g, QImage & img)
{
  static QRgb lut[256];
  static bool ready = false;
  if (!ready) {
    for (int v = -128; v < 128; ++v) {
      QRgb c = qRgba(0, 0, 0, 0);
      if (v >= 100) {
        c = qPremultiply(qRgba(255, 30, 30, 200));
      } else if (v >= 98) {
        c = qPremultiply(qRgba(220, 40, 220, 190));
      } else if (v > 0) {
        const double t = v / 97.0;
        const int a = 40 + static_cast<int>(120 * t);
        c = qPremultiply(qRgba(40 + 215 * t, 120 + 100 * t, 255 - 215 * t, a));
      }
      lut[static_cast<uint8_t>(v)] = c;
    }
    ready = true;
  }
  if (g.width <= 0 || g.height <= 0 || g.data.size() < static_cast<size_t>(g.width) * g.height) {return;}
  if (img.width() != g.width || img.height() != g.height) {
    img = QImage(g.width, g.height, QImage::Format_ARGB32_Premultiplied);
  }
  for (int r = 0; r < g.height; ++r) {
    auto * line = reinterpret_cast<QRgb *>(img.scanLine(r));
    const int8_t * src = &g.data[static_cast<size_t>(r) * g.width];
    for (int c = 0; c < g.width; ++c) {line[c] = lut[static_cast<uint8_t>(src[c])];}
  }
}

}  // namespace

void Map2DView::setGlobalCostmap(const GridData & g)
{
  costmapToImage(g, gcm_img_);
  gcm_res_ = g.res; gcm_ox_ = g.ox; gcm_oy_ = g.oy;
  update();
}

void Map2DView::setLocalCostmap(const GridData & g, const Tf2D & to_map)
{
  costmapToImage(g, lcm_img_);
  lcm_res_ = g.res; lcm_ox_ = g.ox; lcm_oy_ = g.oy;
  local_tf_ = to_map;
  update();
}

void Map2DView::setNavPaths(std::vector<std::array<float, 2>> plan, std::vector<std::array<float, 2>> local)
{
  plan_ = std::move(plan);
  lplan_ = std::move(local);
  update();
}

void Map2DView::setFootprint(std::vector<std::array<float, 2>> poly, std::vector<double> fixed)
{
  fp_ = std::move(poly);
  fixed_fp_ = std::move(fixed);
  update();
}

void Map2DView::setTool(Tool t)
{
  tool_ = t;
  placing_ = false;
  setCursor(t == Tool::None ? Qt::ArrowCursor : Qt::CrossCursor);
  update();
}

double Map2DView::dragYaw(double x0, double y0, double x1, double y1)
{
  return std::hypot(x1 - x0, y1 - y0) < 0.1 ? 0.0 : std::atan2(y1 - y0, x1 - x0);
}

void Map2DView::setPose(const Pose2D & p)
{
  pose_ = p;
  if (follow_ && p.valid) {cx_ = p.x; cy_ = p.y;}
  update();
}

void Map2DView::setPath(std::vector<std::array<float, 3>> path)
{
  path_ = std::move(path);
  update();
}

void Map2DView::fit()
{
  user_moved_ = false;
  if (gw_ == 0) {return;}
  const double w = gw_ * res_, h = gh_ * res_;
  cx_ = ox_ + w / 2;
  cy_ = oy_ + h / 2;
  scale_ = std::min((width() - 40) / w, (height() - 40) / h);
  scale_ = std::clamp(scale_, 2.0, 400.0);
  update();
}

QPointF Map2DView::toScreen(double x, double y) const
{
  return QPointF(width() / 2.0 + (x - cx_) * scale_, height() / 2.0 - (y - cy_) * scale_);
}

QPointF Map2DView::toWorld(const QPointF & s) const
{
  return QPointF(cx_ + (s.x() - width() / 2.0) / scale_, cy_ - (s.y() - height() / 2.0) / scale_);
}

void Map2DView::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.fillRect(rect(), theme::kCanvas);

  // Rejilla de 1 m (se adapta al zoom)
  double step = 1.0;
  while (step * scale_ < 18) {step *= 2;}
  const QPointF tl = toWorld(QPointF(0, 0)), br = toWorld(QPointF(width(), height()));
  p.setPen(QPen(theme::kGridLine, 1));
  for (double x = std::floor(tl.x() / step) * step; x <= br.x(); x += step) {
    const double sx = toScreen(x, 0).x();
    p.drawLine(QPointF(sx, 0), QPointF(sx, height()));
  }
  for (double y = std::floor(br.y() / step) * step; y <= tl.y(); y += step) {
    const double sy = toScreen(0, y).y();
    p.drawLine(QPointF(0, sy), QPointF(width(), sy));
  }

  if (img_.isNull()) {
    p.setPen(theme::kMuted);
    QFont f = font();
    f.setPointSize(12);
    p.setFont(f);
    p.drawText(rect(), Qt::AlignCenter, "Esperando /projected_map…");
    return;
  }

  // Mapa: con la escala Y invertida, la fila 0 del grid (y = origen) queda abajo
  p.save();
  p.translate(width() / 2.0 - cx_ * scale_, height() / 2.0 + cy_ * scale_);
  p.scale(scale_, -scale_);
  p.setRenderHint(QPainter::SmoothPixmapTransform, false);
  p.drawImage(QRectF(ox_, oy_, gw_ * res_, gh_ * res_), img_);
  p.restore();

  // Costmaps (semitransparentes). El local viene en su frame (odom): se lleva a map con la TF
  auto drawCm = [&](const QImage & im, double res, double ox, double oy, const Tf2D & t) {
      if (im.isNull() || !t.valid) {return;}
      p.save();
      p.translate(width() / 2.0 - cx_ * scale_, height() / 2.0 + cy_ * scale_);
      p.scale(scale_, -scale_);
      p.translate(t.x, t.y);
      p.rotate(t.yaw * 180.0 / M_PI);
      p.setRenderHint(QPainter::SmoothPixmapTransform, false);
      p.drawImage(QRectF(ox, oy, im.width() * res, im.height() * res), im);
      p.restore();
    };
  Tf2D ident;
  ident.valid = true;
  if (layers_[kGlobalCm]) {drawCm(gcm_img_, gcm_res_, gcm_ox_, gcm_oy_, ident);}
  if (layers_[kLocalCm]) {drawCm(lcm_img_, lcm_res_, lcm_ox_, lcm_oy_, local_tf_);}

  p.setRenderHint(QPainter::Antialiasing);

  auto drawLine = [&](const std::vector<std::array<float, 2>> & v, const QColor & c, double w) {
      if (v.size() < 2) {return;}
      QPainterPath pp(toScreen(v[0][0], v[0][1]));
      for (size_t i = 1; i < v.size(); ++i) {pp.lineTo(toScreen(v[i][0], v[i][1]));}
      p.setPen(QPen(c, w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      p.setBrush(Qt::NoBrush);
      p.drawPath(pp);
    };
  if (layers_[kPlan]) {drawLine(plan_, QColor(34, 197, 94), 2.5);}
  if (layers_[kLocalPlan]) {drawLine(lplan_, QColor(250, 204, 21), 2.5);}

  // Trayectoria
  if (path_.size() > 1) {
    QPainterPath pp(toScreen(path_[0][0], path_[0][1]));
    for (size_t i = 1; i < path_.size(); ++i) {pp.lineTo(toScreen(path_[i][0], path_[i][1]));}
    p.setPen(QPen(theme::kPath, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(Qt::NoBrush);
    p.drawPath(pp);
    p.setPen(Qt::NoPen);
    p.setBrush(theme::kPath);
    p.drawEllipse(toScreen(path_[0][0], path_[0][1]), 4, 4);
  }

  // Robot: silueta del modelo vista desde arriba (casco XY de cada eslabón, relleno WindingFill = unión)
  if (pose_.valid && !sil_.empty()) {
    QPainterPath sp;
    sp.setFillRule(Qt::WindingFill);
    for (const auto & poly : sil_) {
      QPolygonF q;
      for (const auto & w : poly) {q << toScreen(w.x(), w.y());}
      sp.addPolygon(q);
      sp.closeSubpath();
    }
    p.setPen(QPen(QColor(8, 47, 73), 1.5));
    p.setBrush(QColor(34, 211, 238, 220));
    p.drawPath(sp);
    // Rumbo: tick desde el centro hacia el frente
    const QPointF c = toScreen(pose_.x, pose_.y);
    const QPointF f = toScreen(pose_.x + 0.45 * std::cos(pose_.yaw), pose_.y + 0.45 * std::sin(pose_.yaw));
    p.setPen(QPen(QColor(8, 47, 73), 2));
    p.drawLine(c, f);
  } else if (pose_.valid) {  // sin modelo: flecha orientada
    const QPointF c = toScreen(pose_.x, pose_.y);
    const double r = 11;
    p.save();
    p.translate(c);
    p.rotate(-pose_.yaw * 180.0 / M_PI);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(34, 211, 238, 50));
    p.drawEllipse(QPointF(0, 0), r * 1.9, r * 1.9);
    QPolygonF tri({QPointF(r * 1.3, 0), QPointF(-r * 0.8, r * 0.8), QPointF(-r * 0.4, 0), QPointF(-r * 0.8, -r * 0.8)});
    p.setBrush(theme::kAccent);
    p.setPen(QPen(QColor(8, 47, 73), 1.5));
    p.drawPolygon(tri);
    p.restore();
  }

  // Huella y flecha de /cmd_vel
  if (pose_.valid && layers_[kFootprint]) {
    QPolygonF poly;
    if (fp_.size() >= 3) {
      for (const auto & q : fp_) {poly << toScreen(q[0], q[1]);}
    } else {
      const double c = std::cos(pose_.yaw), s = std::sin(pose_.yaw);
      for (size_t i = 0; i + 1 < fixed_fp_.size(); i += 2) {
        const double x = fixed_fp_[i], y = fixed_fp_[i + 1];
        poly << toScreen(pose_.x + c * x - s * y, pose_.y + s * x + c * y);
      }
    }
    p.setPen(QPen(QColor(34, 211, 238), 1.5));
    p.setBrush(QColor(34, 211, 238, 30));
    p.drawPolygon(poly);
  }
  if (pose_.valid && layers_[kCmdVel] && (std::abs(cmd_vx_) > 0.01 || std::abs(cmd_wz_) > 0.02)) {
    // 1 m/s = 2 m de flecha; el giro se dibuja como arco de wz rad (1 s) a 0.45 m
    const QPen pen(QColor(255, 255, 255), 3);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    const QPointF a = toScreen(pose_.x, pose_.y);
    const double L = cmd_vx_ * 2.0;
    const QPointF b = toScreen(pose_.x + L * std::cos(pose_.yaw), pose_.y + L * std::sin(pose_.yaw));
    p.drawLine(a, b);
    if (std::abs(L) > 0.05) {
      const double ang = pose_.yaw + (L < 0 ? M_PI : 0);  // sentido de avance
      for (double d : {0.4, -0.4}) {
        p.drawLine(b, b - QPointF(std::cos(ang + d), -std::sin(ang + d)) * 14);
      }
    }
    if (std::abs(cmd_wz_) > 0.02) {
      const double r = 0.45 * scale_;
      p.setPen(QPen(QColor(251, 146, 60), 3));
      p.drawArc(QRectF(a.x() - r, a.y() - r, 2 * r, 2 * r),
        static_cast<int>(pose_.yaw * 180.0 / M_PI * 16), static_cast<int>(cmd_wz_ * 180.0 / M_PI * 16));
    }
  }

  // Herramienta activa: flecha de la pose que se está colocando
  if (placing_) {
    const QColor c = tool_ == Tool::Goal ? QColor(244, 63, 94) : QColor(56, 189, 248);
    const QPointF a = toScreen(place0_.x(), place0_.y());
    const double yaw = dragYaw(place0_.x(), place0_.y(), place1_.x(), place1_.y());
    const QPointF b = a + QPointF(std::cos(yaw), -std::sin(yaw)) * 40;
    p.setPen(QPen(c, 3));
    p.setBrush(c);
    p.drawEllipse(a, 6, 6);
    p.drawLine(a, b);
    p.drawLine(b, b - QPointF(std::cos(yaw + 0.5), -std::sin(yaw + 0.5)) * 12);
    p.drawLine(b, b - QPointF(std::cos(yaw - 0.5), -std::sin(yaw - 0.5)) * 12);
  }

  // Barra de escala
  double bar_m = 1.0;
  while (bar_m * scale_ < 60) {bar_m *= 2;}
  while (bar_m * scale_ > 160 && bar_m > 0.25) {bar_m /= 2;}
  const QPointF b0(16, height() - 18);
  p.setPen(QPen(theme::kText, 2));
  p.drawLine(b0, b0 + QPointF(bar_m * scale_, 0));
  p.drawLine(b0 + QPointF(0, -4), b0 + QPointF(0, 4));
  p.drawLine(b0 + QPointF(bar_m * scale_, -4), b0 + QPointF(bar_m * scale_, 4));
  p.drawText(b0 + QPointF(0, -8), QString("%1 m").arg(bar_m));

  // Coordenadas del cursor
  if (cursor_in_) {
    p.setPen(theme::kMuted);
    p.drawText(QRectF(0, 0, width() - 12, height() - 10), Qt::AlignRight | Qt::AlignBottom,
      QString("x %1  y %2 m").arg(cursor_world_.x(), 0, 'f', 2).arg(cursor_world_.y(), 0, 'f', 2));
  }
}

void Map2DView::mousePressEvent(QMouseEvent * e)
{
  if (tool_ != Tool::None && e->button() == Qt::LeftButton) {
    placing_ = true;
    place0_ = place1_ = toWorld(e->pos());
    update();
    return;
  }
  dragging_ = true;
  last_mouse_ = e->pos();
}

void Map2DView::mouseMoveEvent(QMouseEvent * e)
{
  cursor_in_ = true;
  cursor_world_ = toWorld(e->pos());
  if (placing_) {place1_ = cursor_world_;}
  if (dragging_) {
    const QPointF d = e->pos() - last_mouse_;
    cx_ -= d.x() / scale_;
    cy_ += d.y() / scale_;
    last_mouse_ = e->pos();
    user_moved_ = true;
    follow_ = false;
  }
  update();
}

void Map2DView::mouseReleaseEvent(QMouseEvent * e)
{
  dragging_ = false;
  if (placing_ && e->button() == Qt::LeftButton) {
    placing_ = false;
    place1_ = toWorld(e->pos());
    emit poseChosen(static_cast<int>(tool_), place0_.x(), place0_.y(),
      dragYaw(place0_.x(), place0_.y(), place1_.x(), place1_.y()));
    update();
  }
}

void Map2DView::mouseDoubleClickEvent(QMouseEvent *) {fit();}

void Map2DView::wheelEvent(QWheelEvent * e)
{
  const QPointF before = toWorld(e->position());
  scale_ = std::clamp(scale_ * std::pow(1.0015, e->angleDelta().y()), 2.0, 600.0);
  if (!follow_) {
    const QPointF after = toWorld(e->position());
    cx_ += before.x() - after.x();
    cy_ += before.y() - after.y();
  }
  user_moved_ = true;
  update();
}
