// Vista 2D del mapa de ocupación (/projected_map) con robot y trayectoria.
// El grid se convierte a una QImage solo cuando llega un mapa nuevo.
#pragma once

#include <array>
#include <vector>

#include <QImage>
#include <QPointF>
#include <QPolygonF>
#include <QWidget>

#include "ros_bridge.hpp"

class Map2DView : public QWidget {
  Q_OBJECT

public:
  explicit Map2DView(QWidget * parent = nullptr);

  void setGrid(const GridData & g);
  void setPose(const Pose2D & p);
  void setPath(std::vector<std::array<float, 3>> path);
  enum Layer {kGlobalCm, kLocalCm, kPlan, kLocalPlan, kFootprint, kCmdVel, kNumLayers};
  enum class Tool {None, Goal, InitPose};

  void setLayer(Layer l, bool on) {layers_[l] = on; update();}
  void setGlobalCostmap(const GridData & g);
  void setLocalCostmap(const GridData & g, const Tf2D & to_map);  // to_map: frame del costmap -> map
  void setLocalTf(const Tf2D & t) {local_tf_ = t; update();}
  void setNavPaths(std::vector<std::array<float, 2>> plan, std::vector<std::array<float, 2>> local);
  // Polígono en map; vacío = usa la huella fija (x,y,... en base_link)
  void setFootprint(std::vector<std::array<float, 2>> poly, std::vector<double> fixed);
  // Silueta del modelo (polígonos en map); vacía = flecha
  void setSilhouette(std::vector<QPolygonF> s) {sil_ = std::move(s); update();}
  void setCmdVel(double vx, double wz) {cmd_vx_ = vx; cmd_wz_ = wz; update();}
  void setTool(Tool t);
  Tool tool() const {return tool_;}
  // Orientación (rad) de un arrastre de (x0,y0) a (x1,y1); 0 si casi no se movió
  static double dragYaw(double x0, double y0, double x1, double y1);
  QPointF worldAt(const QPointF & screen) const {return toWorld(screen);}
  QPointF screenAt(double x, double y) const {return toScreen(x, y);}
  void setFollow(bool on) {follow_ = on; update();}
  bool follow() const {return follow_;}

public slots:
  void fit();

signals:
  void poseChosen(int tool, double x, double y, double yaw);  // tool = Tool

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

  QImage img_, gcm_img_, lcm_img_;
  Tf2D local_tf_;
  double gcm_res_ = 0.05, gcm_ox_ = 0, gcm_oy_ = 0, lcm_res_ = 0.05, lcm_ox_ = 0, lcm_oy_ = 0;
  std::vector<std::array<float, 2>> plan_, lplan_, fp_;
  std::vector<double> fixed_fp_;
  std::vector<QPolygonF> sil_;
  double cmd_vx_ = 0, cmd_wz_ = 0;
  bool layers_[kNumLayers] = {false, true, true, true, true, true};
  Tool tool_ = Tool::None;
  bool placing_ = false;
  QPointF place0_, place1_;  // inicio y posición actual del arrastre (mundo)
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
