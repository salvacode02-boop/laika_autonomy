#include "cloud3d_view.hpp"

#include <cmath>

#include <QMouseEvent>
#include <QWheelEvent>

#include "theme.hpp"

#ifndef GL_VERTEX_PROGRAM_POINT_SIZE
#define GL_VERTEX_PROGRAM_POINT_SIZE 0x8642
#endif

namespace {

const char * kVert = R"(
#version 120
attribute vec3 aPos;
uniform mat4 uMVP;
uniform float uZmin;
uniform float uZmax;
uniform float uSize;
uniform int uMode;      // 0 = color por altura, 1 = color fijo
uniform vec4 uColor;
varying vec4 vColor;

// Aproximación polinomial del mapa de color "turbo"
vec3 turbo(float x) {
  const vec4 kR4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  const vec4 kG4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  const vec4 kB4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const vec2 kR2 = vec2(-152.94239396, 59.28637943);
  const vec2 kG2 = vec2(4.27729857, 2.82956604);
  const vec2 kB2 = vec2(-89.90310912, 27.34824973);
  x = clamp(x, 0.0, 1.0);
  vec4 v4 = vec4(1.0, x, x * x, x * x * x);
  vec2 v2 = v4.zw * v4.z;
  return vec3(dot(v4, kR4) + dot(v2, kR2), dot(v4, kG4) + dot(v2, kG2), dot(v4, kB4) + dot(v2, kB2));
}

void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
  gl_PointSize = uSize;
  if (uMode == 0) {
    float t = (aPos.z - uZmin) / max(uZmax - uZmin, 0.001);
    vColor = vec4(turbo(0.1 + 0.85 * t), 1.0);
  } else {
    vColor = uColor;
  }
}
)";

const char * kFrag = R"(
#version 120
varying vec4 vColor;
void main() { gl_FragColor = vColor; }
)";

// Modelo del robot: luz fija + color del material (aclarado para que se vea sobre fondo oscuro)
const char * kModelVert = R"(
#version 120
attribute vec3 aPos;
attribute vec3 aNrm;
uniform mat4 uMVP;
uniform mat4 uModel;
varying vec3 vN;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
  vN = mat3(uModel) * aNrm;
}
)";

const char * kModelFrag = R"(
#version 120
uniform vec3 uColor;
varying vec3 vN;
void main() {
  float d = abs(dot(normalize(vN), normalize(vec3(0.3, -0.5, 0.8))));
  vec3 base = 0.28 + 0.72 * uColor;
  gl_FragColor = vec4(base * (0.4 + 0.6 * d), 1.0);
}
)";

}  // namespace

Cloud3DView::Cloud3DView(RosBridge * bridge, QWidget * parent)
: QOpenGLWidget(parent), bridge_(bridge)
{
  setMinimumSize(300, 240);
}

Cloud3DView::~Cloud3DView()
{
  makeCurrent();
  map_vbo_.destroy();
  scan_vbo_.destroy();
  line_vbo_.destroy();
  static_vbo_.destroy();
  for (auto & m : gl_meshes_) {for (auto & g : m) {g.vbo.destroy(); g.ibo.destroy();}}
  doneCurrent();
}

void Cloud3DView::initializeGL()
{
  initializeOpenGLFunctions();
  prog_.addShaderFromSourceCode(QOpenGLShader::Vertex, kVert);
  prog_.addShaderFromSourceCode(QOpenGLShader::Fragment, kFrag);
  prog_.bindAttributeLocation("aPos", 0);
  prog_.link();

  model_prog_.addShaderFromSourceCode(QOpenGLShader::Vertex, kModelVert);
  model_prog_.addShaderFromSourceCode(QOpenGLShader::Fragment, kModelFrag);
  model_prog_.bindAttributeLocation("aPos", 0);
  model_prog_.bindAttributeLocation("aNrm", 1);
  model_prog_.link();
  static_vbo_.create();
  model_dirty_ = model_ != nullptr;

  map_vbo_.create();
  map_vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  scan_vbo_.create();
  scan_vbo_.setUsagePattern(QOpenGLBuffer::StreamDraw);
  line_vbo_.create();
  line_vbo_.setUsagePattern(QOpenGLBuffer::StreamDraw);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  map_uploaded_ = map_capacity_ = 0;
  seen_epoch_ = ~0ull;
}

void Cloud3DView::resizeGL(int, int) {}

bool Cloud3DView::hasNewData() const
{
  return bridge_->map_version.load() != seen_map_version_ ||
         (show_scan_ && bridge_->scan_version.load() != seen_scan_) ||
         bridge_->pose_version.load() != seen_pose_version_ || bridge_->joint_version.load() != seen_joint_version_;
}

void Cloud3DView::syncBuffers()
{
  seen_map_version_ = bridge_->map_version.load();
  seen_pose_version_ = bridge_->pose_version.load();
  seen_joint_version_ = bridge_->joint_version.load();
  {
    std::lock_guard<std::mutex> lk(bridge_->pose_mx);
    pose_ = bridge_->pose;
  }

  std::lock_guard<std::mutex> lk(bridge_->cloud_mx);
  const auto & m = bridge_->map_xyz;
  const size_t n = m.size() / 3;

  if (bridge_->map_epoch != seen_epoch_) {  // reset del mapa
    seen_epoch_ = bridge_->map_epoch;
    map_uploaded_ = 0;
  }
  if (n > map_uploaded_) {
    map_vbo_.bind();
    if (n > map_capacity_) {
      // Crecer al doble y subir todo una sola vez
      map_capacity_ = std::max<size_t>(n * 2, 1 << 20);
      map_vbo_.allocate(static_cast<int>(map_capacity_ * 3 * sizeof(float)));
      map_vbo_.write(0, m.data(), static_cast<int>(n * 3 * sizeof(float)));
    } else {
      // Solo los puntos nuevos
      map_vbo_.write(static_cast<int>(map_uploaded_ * 3 * sizeof(float)),
        m.data() + map_uploaded_ * 3,
        static_cast<int>((n - map_uploaded_) * 3 * sizeof(float)));
    }
    map_uploaded_ = n;
    map_vbo_.release();
  } else if (n < map_uploaded_) {
    map_uploaded_ = n;
  }
  if (bridge_->zmax > bridge_->zmin) {
    zmin_ = bridge_->zmin;
    zmax_ = bridge_->zmax;
  }

  if (show_scan_ && bridge_->scan_version.load() != seen_scan_) {
    seen_scan_ = bridge_->scan_version.load();
    scan_count_ = bridge_->scan_xyz.size() / 3;
    scan_vbo_.bind();
    scan_vbo_.allocate(bridge_->scan_xyz.data(), static_cast<int>(bridge_->scan_xyz.size() * sizeof(float)));
    scan_vbo_.release();
  }
}

void Cloud3DView::setStaticCloud(std::vector<float> xyz)
{
  static_pending_ = std::move(xyz);
  static_dirty_ = true;
  if (static_pending_.size() >= 3) {
    float zmin = 1e9f, zmax = -1e9f, cx = 0, cy = 0;
    const size_t n = static_pending_.size() / 3;
    for (size_t i = 0; i < n; ++i) {
      zmin = std::min(zmin, static_pending_[3 * i + 2]);
      zmax = std::max(zmax, static_pending_[3 * i + 2]);
      cx += static_pending_[3 * i]; cy += static_pending_[3 * i + 1];
    }
    static_zmin_ = zmin;
    static_zmax_ = std::max(zmax, zmin + 0.5f);
    target_ = QVector3D(cx / n, cy / n, zmin);  // centra la vista en la nube
    follow_ = false;
  }
  update();
}

void Cloud3DView::uploadModel()
{
  model_dirty_ = false;
  for (auto & m : gl_meshes_) {for (auto & g : m) {g.vbo.destroy(); g.ibo.destroy();}}
  gl_meshes_.clear();
  if (!model_) {return;}
  for (const auto & mesh : model_->meshes()) {
    gl_meshes_.emplace_back();
    for (const auto & s : mesh.subs) {
      GlSub g;
      g.vbo.create(); g.vbo.bind();
      g.vbo.allocate(s.vn.data(), static_cast<int>(s.vn.size() * sizeof(float)));
      g.vbo.release();
      g.ibo.create(); g.ibo.bind();
      g.ibo.allocate(s.idx.data(), static_cast<int>(s.idx.size() * sizeof(unsigned)));
      g.ibo.release();
      g.n = static_cast<int>(s.idx.size());
      gl_meshes_.back().push_back(g);
    }
  }
}

void Cloud3DView::drawModel(const QMatrix4x4 & mvp)
{
  const QMatrix4x4 base = RobotModel::poseMatrix(pose_.x, pose_.y, pose_.z, pose_.qx, pose_.qy, pose_.qz, pose_.qw);
  model_prog_.bind();
  model_prog_.enableAttributeArray(0);
  model_prog_.enableAttributeArray(1);
  for (const auto & in : model_->instances()) {
    const QMatrix4x4 m = base * model_->linkPose(in.link) * in.visual;
    model_prog_.setUniformValue("uMVP", mvp * m);
    model_prog_.setUniformValue("uModel", m);
    const auto & subs = model_->meshes()[in.mesh].subs;
    for (size_t k = 0; k < subs.size(); ++k) {
      GlSub & g = gl_meshes_[in.mesh][k];
      model_prog_.setUniformValue("uColor", subs[k].color.toVector3D());
      g.vbo.bind();
      g.ibo.bind();
      model_prog_.setAttributeBuffer(0, GL_FLOAT, 0, 3, 6 * sizeof(float));
      model_prog_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
      glDrawElements(GL_TRIANGLES, g.n, GL_UNSIGNED_INT, nullptr);
      g.ibo.release();
      g.vbo.release();
    }
  }
  model_prog_.disableAttributeArray(0);
  model_prog_.disableAttributeArray(1);
  model_prog_.release();
}

void Cloud3DView::drawLines(const std::vector<float> & xyz, GLenum mode, const QColor & c, const QMatrix4x4 & mvp)
{
  if (xyz.empty()) {return;}
  line_vbo_.bind();
  line_vbo_.allocate(xyz.data(), static_cast<int>(xyz.size() * sizeof(float)));
  prog_.setUniformValue("uMVP", mvp);
  prog_.setUniformValue("uMode", 1);
  prog_.setUniformValue("uColor", QVector4D(c.redF(), c.greenF(), c.blueF(), c.alphaF()));
  prog_.enableAttributeArray(0);
  prog_.setAttributeBuffer(0, GL_FLOAT, 0, 3);
  glDrawArrays(mode, 0, static_cast<GLsizei>(xyz.size() / 3));
  line_vbo_.release();
}

void Cloud3DView::paintGL()
{
  syncBuffers();
  if (model_dirty_) {uploadModel();}
  if (static_dirty_) {
    static_dirty_ = false;
    static_count_ = static_pending_.size() / 3;
    static_vbo_.bind();
    static_vbo_.allocate(static_pending_.data(), static_cast<int>(static_pending_.size() * sizeof(float)));
    static_vbo_.release();
    static_pending_.clear();
    static_pending_.shrink_to_fit();
  }

  const float dpr = devicePixelRatioF();
  glViewport(0, 0, static_cast<int>(width() * dpr), static_cast<int>(height() * dpr));
  glClearColor(theme::kCanvas.redF(), theme::kCanvas.greenF(), theme::kCanvas.blueF(), 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (follow_ && pose_.valid) {target_ = QVector3D(pose_.x, pose_.y, pose_.z);}

  QMatrix4x4 proj, view;
  proj.perspective(55.0f, float(width()) / std::max(1, height()), 0.05f, 500.0f);
  const QVector3D eye = target_ + dist_ * QVector3D(
    std::cos(pitch_) * std::cos(yaw_), std::cos(pitch_) * std::sin(yaw_), std::sin(pitch_));
  view.lookAt(eye, target_, QVector3D(0, 0, 1));
  const QMatrix4x4 mvp = proj * view;

  prog_.bind();

  // Piso con rejilla de 1 m alrededor del objetivo
  {
    const float gz = (zmax_ > zmin_) ? zmin_ : -0.5f;
    const int half = 25;
    const float cx = std::round(target_.x()), cy = std::round(target_.y());
    std::vector<float> g;
    g.reserve((2 * half + 1) * 12);
    for (int i = -half; i <= half; ++i) {
      g.insert(g.end(), {cx + i, cy - half, gz, cx + i, cy + half, gz});
      g.insert(g.end(), {cx - half, cy + i, gz, cx + half, cy + i, gz});
    }
    drawLines(g, GL_LINES, QColor(30, 41, 59, 160), mvp);
  }

  // Nube acumulada (color por altura)
  if (map_uploaded_ > 0) {
    map_vbo_.bind();
    prog_.setUniformValue("uMVP", mvp);
    prog_.setUniformValue("uMode", 0);
    prog_.setUniformValue("uZmin", zmin_);
    prog_.setUniformValue("uZmax", zmax_);
    prog_.setUniformValue("uSize", 2.0f * dpr);
    prog_.enableAttributeArray(0);
    prog_.setAttributeBuffer(0, GL_FLOAT, 0, 3);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(map_uploaded_));
    map_vbo_.release();
  }

  // Nube de un PLY cargado (color por altura propio)
  if (static_count_ > 0) {
    static_vbo_.bind();
    prog_.setUniformValue("uMVP", mvp);
    prog_.setUniformValue("uMode", 0);
    prog_.setUniformValue("uZmin", static_zmin_);
    prog_.setUniformValue("uZmax", static_zmax_);
    prog_.setUniformValue("uSize", 2.0f * dpr);
    prog_.enableAttributeArray(0);
    prog_.setAttributeBuffer(0, GL_FLOAT, 0, 3);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(static_count_));
    static_vbo_.release();
  }

  // Escaneo en vivo (blanco translúcido)
  if (show_scan_ && scan_count_ > 0) {
    scan_vbo_.bind();
    prog_.setUniformValue("uMVP", mvp);
    prog_.setUniformValue("uMode", 1);
    prog_.setUniformValue("uColor", QVector4D(0.85f, 0.95f, 1.0f, 0.55f));
    prog_.setUniformValue("uSize", 1.5f * dpr);
    prog_.enableAttributeArray(0);
    prog_.setAttributeBuffer(0, GL_FLOAT, 0, 3);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(scan_count_));
    scan_vbo_.release();
  }

  // Trayectoria
  {
    std::vector<float> pl;
    {
      std::lock_guard<std::mutex> lk(bridge_->pose_mx);
      pl.reserve(bridge_->path.size() * 3);
      for (const auto & p : bridge_->path) {pl.insert(pl.end(), {p[0], p[1], p[2]});}
    }
    if (pl.size() >= 6) {drawLines(pl, GL_LINE_STRIP, theme::kPath, mvp);}
  }

  // Robot: modelo del Go2 (o flecha si no cargó)
  if (pose_.valid && model_ && !gl_meshes_.empty()) {
    prog_.release();
    drawModel(mvp);
    prog_.bind();
  } else if (pose_.valid) {
    const float x = pose_.x, y = pose_.y, z = pose_.z;
    const float c = std::cos(pose_.yaw), s = std::sin(pose_.yaw);
    auto P = [&](float fx, float fy) {return std::array<float, 3>{x + c * fx - s * fy, y + s * fx + c * fy, z};};
    const auto a = P(0.45f, 0), b = P(-0.25f, 0.25f), d = P(-0.25f, -0.25f), e = P(-0.1f, 0);
    std::vector<float> r = {a[0], a[1], a[2], b[0], b[1], b[2], b[0], b[1], b[2], e[0], e[1], e[2],
                            e[0], e[1], e[2], d[0], d[1], d[2], d[0], d[1], d[2], a[0], a[1], a[2],
                            x, y, z, x, y, z + 0.6f};
    glLineWidth(2.0f);
    drawLines(r, GL_LINES, theme::kAccent, mvp);
    glLineWidth(1.0f);
  }

  prog_.release();
}

void Cloud3DView::resetView()
{
  yaw_ = -2.3f;
  pitch_ = 0.7f;
  dist_ = 18.f;
  target_ = pose_.valid ? QVector3D(pose_.x, pose_.y, pose_.z) : QVector3D(0, 0, 0);
  update();
}

void Cloud3DView::setTopView(bool on)
{
  pitch_ = on ? 1.55f : 0.7f;
  update();
}

void Cloud3DView::mousePressEvent(QMouseEvent * e) {last_mouse_ = e->pos();}

void Cloud3DView::mouseMoveEvent(QMouseEvent * e)
{
  const QPoint d = e->pos() - last_mouse_;
  last_mouse_ = e->pos();
  if (e->buttons() & Qt::LeftButton) {
    // Orbitar
    yaw_ -= d.x() * 0.008f;
    pitch_ = std::clamp(pitch_ + d.y() * 0.008f, -0.2f, 1.55f);
  } else if (e->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
    // Desplazar el objetivo en el plano de la cámara
    const float k = dist_ * 0.0015f;
    const QVector3D right(-std::sin(yaw_), std::cos(yaw_), 0);
    const QVector3D fwd(-std::cos(yaw_), -std::sin(yaw_), 0);
    target_ += (-d.x() * k) * right + (d.y() * k) * fwd;
    follow_ = false;
  } else {
    return;
  }
  update();
}

void Cloud3DView::mouseDoubleClickEvent(QMouseEvent *) {resetView();}

void Cloud3DView::wheelEvent(QWheelEvent * e)
{
  dist_ = std::clamp(dist_ * std::pow(0.999f, float(e->angleDelta().y())), 0.5f, 300.0f);
  update();
}
