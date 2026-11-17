// Escritura de archivos en formato RoboCup Rescue: PLY 3D, GeoTIFF 2D y CSV de POIs.
// Todo trabaja sobre una copia de los datos para correr en un hilo aparte.
#pragma once

#include <array>
#include <string>
#include <vector>

#include <QDateTime>
#include <QString>

#include "ros_bridge.hpp"

struct ExportSnapshot {
  QString team, mission;
  QDateTime start_time;
  std::vector<float> map_xyz;              // nube acumulada (x,y,z)
  GridData grid;
  std::vector<std::array<float, 3>> path;  // trayectoria del robot
  Pose2D pose;
  std::vector<Poi> pois;
  int px_per_cell = 2;
};

// Cada función devuelve "" si todo salió bien, o el mensaje de error.
QString writePly(const QString & path, const ExportSnapshot & s);
QString writeGeoTiff(const QString & path, const ExportSnapshot & s);
QString writePoisCsv(const QString & path, const ExportSnapshot & s);
