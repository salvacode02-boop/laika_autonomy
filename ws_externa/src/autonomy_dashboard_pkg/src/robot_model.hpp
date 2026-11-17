// Modelo del Go2: URDF (urdf::Model) + mallas .dae (assimp) + cinemática con /joint_states.
// Sin OpenGL: Cloud3DView sube las mallas a la GPU y Map2DView dibuja la silueta.
#pragma once

#include <map>
#include <string>
#include <vector>

#include <QMatrix4x4>
#include <QPolygonF>
#include <QQuaternion>
#include <QVector3D>
#include <QVector4D>

struct SubMesh {
  std::vector<float> vn;        // x y z nx ny nz intercalados
  std::vector<unsigned> idx;    // triángulos
  QVector4D color;
};

struct Mesh {
  std::vector<SubMesh> subs;
  std::vector<QVector3D> extremes;  // puntos extremos en distintas direcciones: bastan para la silueta
};

struct MeshInstance {
  int mesh, link;
  QMatrix4x4 visual;  // origen del <visual> (con la escala)
};

class RobotModel {
public:
  // urdf: ruta del .urdf; pkg_dir: carpeta de go2_description (package://go2_description/ -> pkg_dir/)
  bool load(const std::string & urdf, const std::string & pkg_dir, std::string * err);
  const std::vector<Mesh> & meshes() const {return meshes_;}
  const std::vector<MeshInstance> & instances() const {return inst_;}
  size_t triangles() const;
  size_t linkCount() const {return links_.size();}

  // Ángulos por nombre de joint; los que falten quedan en la postura de pie (hip 0, thigh 0.8, calf -1.5)
  void setJoints(const std::map<std::string, double> & q);
  const QMatrix4x4 & linkPose(int link) const {return link_pose_[link];}  // en el frame base_link
  // Casco convexo XY (en map) de cada eslabón. base_in_map: pose de base_link en map.
  std::vector<QPolygonF> silhouette(const QMatrix4x4 & base_in_map) const;
  static QMatrix4x4 poseMatrix(double x, double y, double z, double qx, double qy, double qz, double qw);

private:
  struct Link {
    int parent = -1;
    QMatrix4x4 origin;      // joint origin (padre -> hijo)
    QVector3D axis{0, 0, 1};
    bool revolute = false;
    std::string joint;
  };
  std::vector<Link> links_;
  std::vector<QMatrix4x4> link_pose_;
  std::vector<Mesh> meshes_;
  std::vector<MeshInstance> inst_;
};
