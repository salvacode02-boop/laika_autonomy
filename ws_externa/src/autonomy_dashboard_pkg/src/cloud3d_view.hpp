// Vista 3D de la nube acumulada con OpenGL.
// Los puntos viven en la GPU: cada frame solo se suben los puntos nuevos,
// y el color por altura se calcula en el shader.
#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QMatrix4x4>
#include <QVector3D>

#include "ros_bridge.hpp"

class Cloud3DView : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  Cloud3DView(RosBridge * bridge, QWidget * parent = nullptr);
  ~Cloud3DView() override;

  void setFollow(bool on) {follow_ = on; update();}
  void setShowScan(bool on) {show_scan_ = on; update();}
  void setTopView(bool on);
  bool hasNewData() const;

public slots:
  void resetView();

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent * e) override;
  void mouseDoubleClickEvent(QMouseEvent *) override;
  void wheelEvent(QWheelEvent * e) override;

private:
  void syncBuffers();
  void drawLines(const std::vector<float> & xyz, GLenum mode, const QColor & c, const QMatrix4x4 & mvp);

  RosBridge * bridge_;
  QOpenGLShaderProgram prog_;
  QOpenGLBuffer map_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer scan_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer line_vbo_{QOpenGLBuffer::VertexBuffer};

  size_t map_uploaded_ = 0, map_capacity_ = 0, scan_count_ = 0;
  uint64_t seen_epoch_ = ~0ull, seen_scan_ = ~0ull;
  uint64_t seen_map_version_ = ~0ull, seen_pose_version_ = ~0ull;
  float zmin_ = -1, zmax_ = 2;

  // Cámara orbital (Z arriba)
  QVector3D target_{0, 0, 0};
  float yaw_ = -2.3f, pitch_ = 0.7f, dist_ = 18.f;
  bool follow_ = false, show_scan_ = true;
  QPoint last_mouse_;
  Pose2D pose_;
};
