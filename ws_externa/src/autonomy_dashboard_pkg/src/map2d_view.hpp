// Vista 2D del mapa de ocupación (/projected_map) con robot y trayectoria.
// El grid se convierte a una QImage solo cuando llega un mapa nuevo.
#pragma once

#include <array>
#include <vector>

#include <QImage>
#include <QPointF>
#include <QWidget>

#include "ros_bridge.hpp"

class Map2DView : public QWidget {
  Q_OBJECT

public:
  explicit Map2DView(QWidget * parent = nullptr);

  void setGrid(const GridData & g);
  void setPose(const Pose2D & p);
  void setPath(std::vector<std::array<float, 3>> path);
  void setFollow(bool on) {follow_ = on; update();}
  bool follow() const {return follow_;}

public slots:
  void fit();

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent * e) override;
  void mouseReleaseEvent(QMouseEvent * e) override;
  void mouseDoubleClickEvent(QMouseEvent * e) override;
  void wheelEvent(QWheelEvent * e) override;

private:
  QPointF toScreen(double x, double y) const;
  QPointF toWorld(const QPointF & s) const;

  QImage img_;
  double res_ = 0.05, ox_ = 0, oy_ = 0;
  int gw_ = 0, gh_ = 0;
  Pose2D pose_;
  std::vector<std::array<float, 3>> path_;

  // Vista: centro en metros y escala en pixeles por metro
  double cx_ = 0, cy_ = 0, scale_ = 40;
  bool user_moved_ = false, follow_ = false;
  bool dragging_ = false;
  QPointF last_mouse_, cursor_world_;
  bool cursor_in_ = false;
};
