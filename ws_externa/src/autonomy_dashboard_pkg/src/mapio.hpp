// Carga de mapas para el modo "Mapa cargado": .yaml+.pgm (formato nativo de map_server),
// GeoTIFF y PLY exportados por ESTE dashboard. Todo se convierte a GridData y a .pgm+.yaml.
#pragma once

#include <vector>

#include <QString>

#include "ros_bridge.hpp"

// Banda de altura del PLY -> rejilla (valores de config/dashboard_laika.yaml, sección ply_to_grid)
struct PlyParams {
  double res = 0.05;
  double floor_percentile = 2.0;  // el piso = percentil bajo de z
  double band_min = 0.10;         // sobre el piso; deja fuera el piso mismo (ruido +-5 cm)
  double band_max = 0.93;         // max_height de pc2scan.yaml (0.6 sobre base_link) + piso 0.33 bajo base_link
  double floor_tol = 0.10;        // puntos a menos de esto del piso = celda libre
  int min_points = 2;             // puntos en la banda para marcar ocupado (descarta ruido suelto)
};

struct LoadedMap {
  QString error;                 // "" = ok
  GridData grid;
  QString yaml;                  // .yaml resultante (el original, o el convertido en out_dir)
  std::vector<float> cloud;      // solo PLY: x,y,z para la vista 3D
  QString note;                  // resumen para el panel de mensajes
};

QString readPgmYaml(const QString & yaml, GridData & g);
QString writePgmYaml(const QString & prefix, const GridData & g);  // crea prefix.pgm y prefix.yaml
QString geotiffToGrid(const QString & path, GridData & g);
QString plyToGrid(const QString & path, const PlyParams & p, GridData & g, std::vector<float> & cloud, QString * note);
// Elige el lector por extensión; lo convertido se escribe en out_dir. Corre en un hilo aparte.
LoadedMap loadMap(const QString & file, const QString & out_dir, const PlyParams & p);
