#include "exporters.hpp"

#include <cmath>
#include <cstdio>

#include <QFileInfo>
#include <QFont>
#include <QImage>
#include <QPainter>
#include <QPainterPath>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <geotiff/geotiffio.h>
#include <geotiff/geovalues.h>
#include <geotiff/xtiffio.h>

// ---------------------------------------------------------------------------
// PLY: x y z red green blue nx ny nz temp confidence
// ---------------------------------------------------------------------------
QString writePly(const QString & path, const ExportSnapshot & s)
{
  const size_t n = s.map_xyz.size() / 3;
  if (n == 0) {return "No hay puntos 3D (¿presionaste START?)";}

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(n);
  float zmin = 1e9f, zmax = -1e9f;
  for (size_t i = 0; i < n; ++i) {
    auto & p = (*cloud)[i];
    p.x = s.map_xyz[3 * i];
    p.y = s.map_xyz[3 * i + 1];
    p.z = s.map_xyz[3 * i + 2];
    zmin = std::min(zmin, p.z);
    zmax = std::max(zmax, p.z);
  }

  // Normales por PCA de 10 vecinos (multihilo), orientadas hacia la trayectoria
  pcl::PointCloud<pcl::Normal> normals;
  {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    ne.setKSearch(10);
    float vx = 0, vy = 0, vz = 0.5f;
    if (!s.path.empty()) {
      for (const auto & p : s.path) {vx += p[0]; vy += p[1]; vz += p[2];}
      vx /= s.path.size(); vy /= s.path.size(); vz /= s.path.size();
    }
    ne.setViewPoint(vx, vy, vz);
    ne.compute(normals);
  }

  FILE * f = std::fopen(path.toLocal8Bit().constData(), "w");
  if (!f) {return "No se pudo abrir " + path;}

  const QByteArray team = s.team.toUtf8();
  const QByteArray start = s.start_time.toString("yyyy-MM-dd HH:mm:ss").toUtf8();
  const QByteArray mission = s.mission.toUtf8();
  std::fprintf(f,
    "ply\nformat ascii 1.0\ncomment %s\ncomment %s\ncomment %s\n"
    "element vertex %zu\n"
    "property float x\nproperty float y\nproperty float z\n"
    "property uchar red\nproperty uchar green\nproperty uchar blue\n"
    "property float nx\nproperty float ny\nproperty float nz\n"
    "property float temp\nproperty float confidence\nend_header\n",
    team.constData(), start.constData(), mission.constData(), n);

  // Color por altura: azul (bajo) → verde → rojo (alto)
  const float span = (zmax - zmin) > 1e-6f ? (zmax - zmin) : 1.0f;
  std::vector<char> buf(1 << 20);
  size_t used = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto & p = (*cloud)[i];
    const float t = (p.z - zmin) / span;
    const int r = static_cast<int>(255 * t);
    const int g = static_cast<int>(255 * (1 - std::fabs(2 * t - 1)));
    const int b = static_cast<int>(255 * (1 - t));
    float nx = normals[i].normal_x, ny = normals[i].normal_y, nz = normals[i].normal_z;
    if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz)) {nx = ny = nz = 0;}
    if (buf.size() - used < 256) {
      std::fwrite(buf.data(), 1, used, f);
      used = 0;
    }
    // temp = 0 (sin sensor térmico), confidence = 1
    used += std::snprintf(buf.data() + used, buf.size() - used,
      "%.4f %.4f %.4f %d %d %d %.4f %.4f %.4f 0.00 1.00\n",
      p.x, p.y, p.z, r, g, b, nx, ny, nz);
  }
  std::fwrite(buf.data(), 1, used, f);
  const bool ok = std::fclose(f) == 0;
  return ok ? QString() : "Error al escribir " + path;
}

// ---------------------------------------------------------------------------
// GeoTIFF estilo RoboCup Rescue (como hector_geotiff):
//   - X hacia arriba, Y hacia la izquierda
//   - desconocido: tablero de 1 m gris claro
//   - libre: blanco con rejilla de 0.5 m
//   - ocupado: azul oscuro
//   - trayectoria: morado; escala de 1 m; flechas X/Y; nombre del archivo
// ---------------------------------------------------------------------------
QString writeGeoTiff(const QString & path, const ExportSnapshot & s)
{
  const GridData & g = s.grid;
  if (g.width == 0 || g.height == 0) {return "No hay mapa 2D (¿octomap publica /projected_map?)";}

  const int f = std::max(1, s.px_per_cell);
  const double px_m = g.res / f;              // metros por pixel
  const int map_w = g.height * f;             // columnas ↔ eje Y
  const int map_h = g.width * f;              // filas ↔ eje X
  const int L = 40, T = 50, Rm = 150, B = 40; // márgenes
  const int W = map_w + L + Rm, H = std::max(map_h + T + B, 260);

  const double x_max = g.ox + g.width * g.res;
  const double y_max = g.oy + g.height * g.res;

  QImage img(W, H, QImage::Format_RGB32);
  img.fill(QColor(255, 255, 255));

  const QRgb c_unk_a = qRgb(226, 226, 227), c_unk_b = qRgb(237, 237, 238);
  const QRgb c_free = qRgb(255, 255, 255), c_grid = qRgb(190, 190, 191);
  const QRgb c_occ = qRgb(0, 40, 120);

  for (int r = 0; r < map_h; ++r) {
    auto * line = reinterpret_cast<QRgb *>(img.scanLine(T + r));
    const double x = x_max - (r + 0.5) * px_m;
    const int i = g.width - 1 - r / f;
    const bool x_line = std::floor(x / 0.5) != std::floor((x + px_m) / 0.5);
    for (int c = 0; c < map_w; ++c) {
      const double y = y_max - (c + 0.5) * px_m;
      const int j = g.height - 1 - c / f;
      const int8_t v = g.data[static_cast<size_t>(j) * g.width + i];
      QRgb col;
      if (v < 0) {
        col = ((static_cast<int>(std::floor(x)) + static_cast<int>(std::floor(y))) & 1) ? c_unk_a : c_unk_b;
      } else if (v >= 65) {
        col = c_occ;
      } else {
        const bool y_line = std::floor(y / 0.5) != std::floor((y + px_m) / 0.5);
        col = (x_line || y_line) ? c_grid : c_free;
      }
      line[L + c] = col;
    }
  }

  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  auto toPx = [&](double x, double y) {
      return QPointF(L + (y_max - y) / px_m, T + (x_max - x) / px_m);
    };

  // Trayectoria del robot
  if (s.path.size() > 1) {
    QPainterPath pp(toPx(s.path[0][0], s.path[0][1]));
    for (size_t k = 1; k < s.path.size(); ++k) {pp.lineTo(toPx(s.path[k][0], s.path[k][1]));}
    p.setPen(QPen(QColor(120, 0, 140), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(pp);
  }
  // Inicio de la trayectoria
  if (!s.path.empty()) {
    const QPointF st = toPx(s.path[0][0], s.path[0][1]);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(120, 0, 140));
    p.drawEllipse(st, 4, 4);
  }
  // POIs como rombos de 30 cm
  for (const auto & poi : s.pois) {
    const QPointF c = toPx(poi.x, poi.y);
    const double d = std::max(4.0, 0.15 / px_m);
    QColor col(240, 10, 10);
    if (poi.type == "apriltag") {col = QColor(255, 200, 0);}
    else if (poi.type == "hazmat") {col = QColor(255, 100, 30);}
    p.setPen(Qt::NoPen);
    p.setBrush(col);
    p.drawPolygon(QPolygonF({c + QPointF(0, -d), c + QPointF(d, 0), c + QPointF(0, d), c + QPointF(-d, 0)}));
  }

  const QColor blue(0, 50, 140);
  QFont font("DejaVu Sans", 10);
  font.setBold(true);
  p.setFont(font);

  // Nombre del archivo arriba a la izquierda
  p.setPen(QColor(0, 44, 207));
  p.drawText(QPointF(10, 25), QFileInfo(path).completeBaseName());

  // Escala de 1 m
  const double one_m = 1.0 / px_m;
  const QPointF s0(W - Rm + 30, H - 40);
  p.setPen(QPen(blue, 3));
  p.drawLine(s0, s0 + QPointF(one_m, 0));
  p.setPen(QPen(blue, 2));
  p.drawLine(s0 + QPointF(0, -5), s0 + QPointF(0, 5));
  p.drawLine(s0 + QPointF(one_m, -5), s0 + QPointF(one_m, 5));
  p.drawText(s0 + QPointF(one_m / 2 - 8, 20), "1m");

  // Orientación: X arriba, Y izquierda
  const QPointF o(W - 50, H - 110);
  const double al = 40;
  p.setPen(QPen(blue, 2));
  p.setBrush(blue);
  p.drawLine(o, o + QPointF(0, -al));
  p.drawPolygon(QPolygonF({o + QPointF(0, -al), o + QPointF(-5, -al + 10), o + QPointF(5, -al + 10)}));
  p.drawLine(o, o + QPointF(-al, 0));
  p.drawPolygon(QPolygonF({o + QPointF(-al, 0), o + QPointF(-al + 10, -5), o + QPointF(-al + 10, 5)}));
  p.drawText(o + QPointF(6, -al - 4), "X");
  p.drawText(o + QPointF(-al - 4, 18), "Y");
  p.end();

  // Escribir TIFF con georreferencia (matriz pixel → metros del frame map)
  TIFF * tif = XTIFFOpen(path.toLocal8Bit().constData(), "w");
  if (!tif) {return "No se pudo abrir " + path;}
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, W);
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, H);
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tif, 0));

  // X = x_max + T*px - px*fila ;  Y = y_max + L*px - px*columna
  double m[16] = {
    0, -px_m, 0, x_max + T * px_m,
    -px_m, 0, 0, y_max + L * px_m,
    0, 0, 0, 0,
    0, 0, 0, 1};
  TIFFSetField(tif, TIFFTAG_GEOTRANSMATRIX, 16, m);

  GTIF * gt = GTIFNew(tif);
  if (gt) {
    GTIFKeySet(gt, GTModelTypeGeoKey, TYPE_SHORT, 1, ModelTypeProjected);
    GTIFKeySet(gt, GTRasterTypeGeoKey, TYPE_SHORT, 1, RasterPixelIsArea);
    GTIFKeySet(gt, ProjectedCSTypeGeoKey, TYPE_SHORT, 1, KvUserDefined);
    GTIFKeySet(gt, ProjLinearUnitsGeoKey, TYPE_SHORT, 1, Linear_Meter);
    GTIFKeySet(gt, GTCitationGeoKey, TYPE_ASCII, 0, "RoboCup Rescue local map frame");
    GTIFWriteKeys(gt);
    GTIFFree(gt);
  }

  const QImage rgb = img.convertToFormat(QImage::Format_RGB888);
  bool ok = true;
  for (int r = 0; r < H && ok; ++r) {
    ok = TIFFWriteScanline(tif, const_cast<uchar *>(rgb.constScanLine(r)), r, 0) >= 0;
  }
  XTIFFClose(tif);
  return ok ? QString() : "Error al escribir " + path;
}

// ---------------------------------------------------------------------------
// CSV de POIs (mismo encabezado que usaba la versión anterior)
// ---------------------------------------------------------------------------
QString writePoisCsv(const QString & path, const ExportSnapshot & s)
{
  FILE * f = std::fopen(path.toLocal8Bit().constData(), "w");
  if (!f) {return "No se pudo abrir " + path;}
  const QDateTime now = QDateTime::currentDateTime();
  std::fprintf(f, "\"pois\"\n\"1.3\"\n\"%s\"\n\"Country\"\n\"%s\"\n\"%s\"\n\"%s\"\n",
    s.team.toUtf8().constData(),
    now.toString("yyyy-MM-dd").toUtf8().constData(),
    now.toString("HH:mm:ss").toUtf8().constData(),
    s.mission.toUtf8().constData());
  std::fprintf(f, "detection,time,type,name,x,y,z,robot,mode\n");
  int k = 1;
  for (const auto & p : s.pois) {
    std::fprintf(f, "%d,%02d:%02d:%02d,%s,%s,%.3f,%.3f,%.3f,1,A\n",
      k++, p.time_s / 3600, (p.time_s / 60) % 60, p.time_s % 60,
      p.type.c_str(), p.name.c_str(), p.x, p.y, p.z);
  }
  return std::fclose(f) == 0 ? QString() : "Error al escribir " + path;
}
