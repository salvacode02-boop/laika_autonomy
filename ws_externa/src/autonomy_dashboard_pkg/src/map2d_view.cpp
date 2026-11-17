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

  p.setRenderHint(QPainter::Antialiasing);

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

  // Robot: flecha orientada
  if (pose_.valid) {
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
  dragging_ = true;
  last_mouse_ = e->pos();
}

void Map2DView::mouseMoveEvent(QMouseEvent * e)
{
  cursor_in_ = true;
  cursor_world_ = toWorld(e->pos());
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

void Map2DView::mouseReleaseEvent(QMouseEvent *) {dragging_ = false;}

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
