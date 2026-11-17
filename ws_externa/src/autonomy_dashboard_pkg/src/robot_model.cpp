#include "robot_model.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <urdf/model.h>

namespace {

QMatrix4x4 fromUrdf(const urdf::Pose & p)
{
  QMatrix4x4 m;
  m.translate(p.position.x, p.position.y, p.position.z);
  m.rotate(QQuaternion(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z));
  return m;
}

// Casco convexo (monotone chain). Sale en sentido antihorario: así el relleno WindingFill no se cancela.
QPolygonF hull(std::vector<QPointF> p)
{
  if (p.size() < 3) {return QPolygonF();}
  std::sort(p.begin(), p.end(), [](const QPointF & a, const QPointF & b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
  auto cross = [](const QPointF & o, const QPointF & a, const QPointF & b) {
      return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
  std::vector<QPointF> h(2 * p.size());
  size_t k = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) {--k;}
    h[k++] = p[i];
  }
  for (size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0) {--k;}
    h[k++] = p[i - 1];
  }
  h.resize(k - 1);
  QPolygonF out;
  for (const auto & q : h) {out << q;}
  return out;
}

// Carga una malla con assimp: nodos horneados (PreTransformVertices), una SubMesh por material.
bool loadMesh(const std::string & path, Mesh & out, std::string * err)
{
  Assimp::Importer imp;
  const aiScene * sc = imp.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
      aiProcess_PreTransformVertices | aiProcess_GenSmoothNormals | aiProcess_ImproveCacheLocality);
  if (!sc || !sc->mNumMeshes) {*err = "assimp: " + path + ": " + imp.GetErrorString(); return false;}
  std::vector<QVector3D> all;
  for (unsigned mi = 0; mi < sc->mNumMeshes; ++mi) {
    const aiMesh * m = sc->mMeshes[mi];
    SubMesh s;
    aiColor4D c(0.7f, 0.7f, 0.7f, 1.f);
    sc->mMaterials[m->mMaterialIndex]->Get(AI_MATKEY_COLOR_DIFFUSE, c);
    s.color = QVector4D(c.r, c.g, c.b, 1.f);
    s.vn.reserve(m->mNumVertices * 6);
    for (unsigned v = 0; v < m->mNumVertices; ++v) {
      // assimp pasa los .dae Z_UP a Y_UP con una rotación en la raíz que, horneada, anula la matriz del nodo (los
      // eslabones salían girados 90 grados). Se deshace aquí: (x,y,z) -> (x,-z,y). Todos los .dae del Go2 son Z_UP.
      const aiVector3D & p0 = m->mVertices[v];
      const aiVector3D n0 = m->HasNormals() ? m->mNormals[v] : aiVector3D(0, 1, 0);
      const aiVector3D p(p0.x, -p0.z, p0.y), n(n0.x, -n0.z, n0.y);
      s.vn.insert(s.vn.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
      all.emplace_back(p.x, p.y, p.z);
    }
    for (unsigned f = 0; f < m->mNumFaces; ++f) {
      if (m->mFaces[f].mNumIndices == 3) {s.idx.insert(s.idx.end(), m->mFaces[f].mIndices, m->mFaces[f].mIndices + 3);}
    }
    out.subs.push_back(std::move(s));
  }
  // Extremos en ~150 direcciones (espiral de Fibonacci) -> aproximan el casco 3D
  const int N = 150;
  for (int i = 0; i < N; ++i) {
    const double z = 1 - 2.0 * (i + 0.5) / N, r = std::sqrt(1 - z * z), a = i * 2.399963;
    const QVector3D d(r * std::cos(a), r * std::sin(a), z);
    const QVector3D * best = &all[0];
    for (const auto & p : all) {if (QVector3D::dotProduct(p, d) > QVector3D::dotProduct(*best, d)) {best = &p;}}
    out.extremes.push_back(*best);
  }
  return true;
}

}  // namespace

QMatrix4x4 RobotModel::poseMatrix(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  QMatrix4x4 m;
  m.translate(x, y, z);
  m.rotate(QQuaternion(qw, qx, qy, qz).normalized());
  return m;
}

bool RobotModel::load(const std::string & urdf_path, const std::string & pkg_dir, std::string * err)
{
  urdf::Model model;
  if (!model.initFile(urdf_path)) {*err = "No se pudo leer el URDF " + urdf_path; return false;}
  std::map<std::string, int> mesh_of;
  std::function<bool(const urdf::LinkConstSharedPtr &, int)> walk =
    [&](const urdf::LinkConstSharedPtr & l, int parent) {
      Link k;
      k.parent = parent;
      if (const auto j = l->parent_joint) {
        k.origin = fromUrdf(j->parent_to_joint_origin_transform);
        k.revolute = j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::CONTINUOUS;
        k.axis = QVector3D(j->axis.x, j->axis.y, j->axis.z);
        k.joint = j->name;
      }
      const int id = static_cast<int>(links_.size());
      links_.push_back(k);
      if (l->visual && l->visual->geometry && l->visual->geometry->type == urdf::Geometry::MESH) {
        const auto mesh = std::static_pointer_cast<urdf::Mesh>(l->visual->geometry);
        const std::string prefix = "package://go2_description/";
        std::string rel = mesh->filename;
        if (rel.compare(0, prefix.size(), prefix) != 0) {*err = "Ruta de malla no soportada: " + rel; return false;}
        const std::string path = pkg_dir + "/" + rel.substr(prefix.size());
        auto it = mesh_of.find(path);
        if (it == mesh_of.end()) {
          Mesh m;
          if (!loadMesh(path, m, err)) {return false;}
          meshes_.push_back(std::move(m));
          it = mesh_of.emplace(path, static_cast<int>(meshes_.size()) - 1).first;
        }
        QMatrix4x4 v = fromUrdf(l->visual->origin);
        v.scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        inst_.push_back({it->second, id, v});
      }
      for (const auto & c : l->child_links) {if (!walk(c, id)) {return false;}}
      return true;
    };
  if (!model.getRoot() || !walk(model.getRoot(), -1)) {
    if (err->empty()) {*err = "URDF sin raíz";}
    return false;
  }
  link_pose_.assign(links_.size(), QMatrix4x4());
  setJoints({});
  return true;
}

size_t RobotModel::triangles() const
{
  size_t n = 0;
  for (const auto & i : inst_) {for (const auto & s : meshes_[i.mesh].subs) {n += s.idx.size() / 3;}}
  return n;
}

void RobotModel::setJoints(const std::map<std::string, double> & q)
{
  for (size_t i = 0; i < links_.size(); ++i) {
    const Link & k = links_[i];
    QMatrix4x4 m = k.parent >= 0 ? link_pose_[k.parent] : QMatrix4x4();
    m = m * k.origin;
    if (k.revolute) {
      double a = 0;
      const auto it = q.find(k.joint);
      if (it != q.end()) {a = it->second;}
      else if (k.joint.find("_thigh_") != std::string::npos) {a = 0.8;}
      else if (k.joint.find("_calf_") != std::string::npos) {a = -1.5;}
      m.rotate(static_cast<float>(a * 180.0 / M_PI), k.axis);
    }
    link_pose_[i] = m;
  }
}

std::vector<QPolygonF> RobotModel::silhouette(const QMatrix4x4 & base_in_map) const
{
  std::vector<QPolygonF> out;
  for (const auto & in : inst_) {
    const QMatrix4x4 m = base_in_map * link_pose_[in.link] * in.visual;
    std::vector<QPointF> pts;
    for (const auto & p : meshes_[in.mesh].extremes) {
      const QVector3D w = m.map(p);
      pts.emplace_back(w.x(), w.y());
    }
    QPolygonF h = hull(std::move(pts));
    if (!h.isEmpty()) {out.push_back(std::move(h));}
  }
  return out;
}
