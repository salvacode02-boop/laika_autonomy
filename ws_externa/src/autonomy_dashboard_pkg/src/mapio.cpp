#include "mapio.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QRegularExpression>
#include <QTextStream>

#include <tiffio.h>

namespace {

void finishStats(GridData & g)
{
  g.known = g.free = g.occupied = 0;
  for (int8_t v : g.data) {
    if (v < 0) {continue;}
    ++g.known;
    if (v >= 65) {++g.occupied;} else {++g.free;}
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// .yaml + imagen (trinary, como nav2_map_server)
// ---------------------------------------------------------------------------
QString readPgmYaml(const QString & yaml, GridData & g)
{
  QFile f(yaml);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {return "No se pudo abrir " + yaml;}
  QString image;
  double res = 0, ox = 0, oy = 0, occ_t = 0.65, free_t = 0.25;
  int negate = 0;
  const QRegularExpression kv("^\\s*([a-z_]+)\\s*:\\s*(.*?)\\s*$");
  for (const QString & line : QString::fromUtf8(f.readAll()).split('\n')) {
    const auto m = kv.match(line);
    if (!m.hasMatch()) {continue;}
    const QString k = m.captured(1), v = m.captured(2);
    if (k == "image") {image = QString(v).remove('"').remove('\'');}
    else if (k == "resolution") {res = v.toDouble();}
    else if (k == "negate") {negate = v.toInt();}
    else if (k == "occupied_thresh") {occ_t = v.toDouble();}
    else if (k == "free_thresh") {free_t = v.toDouble();}
    else if (k == "origin") {
      const auto p = QString(v).remove('[').remove(']').split(',');
      if (p.size() >= 2) {ox = p[0].toDouble(); oy = p[1].toDouble();}
    }
  }
  if (image.isEmpty() || res <= 0) {return "yaml sin image o resolution: " + yaml;}
  QImage img(QFileInfo(yaml).dir().filePath(image));
  if (img.isNull()) {return "No se pudo leer la imagen " + image;}
  img = img.convertToFormat(QImage::Format_Grayscale8);
  g = GridData();
  g.width = img.width();
  g.height = img.height();
  g.res = res;
  g.ox = ox;
  g.oy = oy;
  g.data.assign(static_cast<size_t>(g.width) * g.height, -1);
  for (int r = 0; r < g.height; ++r) {
    const uchar * line = img.constScanLine(r);
    int8_t * dst = &g.data[static_cast<size_t>(g.height - 1 - r) * g.width];  // fila 0 de la imagen = y máximo
    for (int c = 0; c < g.width; ++c) {
      const double p = negate ? line[c] / 255.0 : (255 - line[c]) / 255.0;
      dst[c] = p > occ_t ? 100 : (p < free_t ? 0 : -1);
    }
  }
  finishStats(g);
  return QString();
}

QString writePgmYaml(const QString & prefix, const GridData & g)
{
  if (g.width == 0 || g.height == 0) {return "Mapa vacío";}
  FILE * f = std::fopen((prefix + ".pgm").toLocal8Bit().constData(), "wb");
  if (!f) {return "No se pudo escribir " + prefix + ".pgm";}
  std::fprintf(f, "P5\n%d %d\n255\n", g.width, g.height);
  std::vector<uint8_t> row(g.width);
  for (int r = 0; r < g.height; ++r) {
    const int8_t * src = &g.data[static_cast<size_t>(g.height - 1 - r) * g.width];
    for (int c = 0; c < g.width; ++c) {row[c] = src[c] < 0 ? 205 : (src[c] >= 65 ? 0 : 254);}
    std::fwrite(row.data(), 1, row.size(), f);
  }
  if (std::fclose(f) != 0) {return "Error al escribir " + prefix + ".pgm";}
  QFile y(prefix + ".yaml");
  if (!y.open(QIODevice::WriteOnly | QIODevice::Text)) {return "No se pudo escribir " + prefix + ".yaml";}
  QTextStream(&y) << "image: " << QFileInfo(prefix).fileName() << ".pgm\nmode: trinary\nresolution: "
                  << QString::number(g.res, 'g', 6) << "\norigin: [" << QString::number(g.ox, 'f', 4) << ", "
                  << QString::number(g.oy, 'f', 4) << ", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  return QString();
}

// ---------------------------------------------------------------------------
// GeoTIFF del dashboard (ver writeGeoTiff). La descripción del TIFF guarda la geometría exacta:
//   laika-geotiff v1 res= f= L= T= mw= mh= xmax= ymax=
// Pintura por celda (f x f pixeles): algún pixel azul de ocupado -> ocupado; algún pixel gris del
// tablero de desconocido -> desconocido; si no -> libre (blanco, rejilla, trayectoria o POI encima).
// Límite: una trayectoria/POI que tape TODA una celda desconocida la vuelve libre (no pasa: la ruta va por libre).
// ---------------------------------------------------------------------------
QString geotiffToGrid(const QString & path, GridData & g)
{
  TIFFSetWarningHandler(nullptr);  // etiquetas GeoTIFF desconocidas para libtiff plano
  TIFF * tif = TIFFOpen(path.toLocal8Bit().constData(), "r");
  if (!tif) {return "No se pudo abrir " + path;}
  struct Closer {TIFF * t; ~Closer() {TIFFClose(t);}} closer{tif};
  uint32_t W = 0, H = 0;
  uint16_t spp = 0, bps = 0;
  TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &W);
  TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &H);
  TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);
  TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bps);
  char * desc = nullptr;
  double res = 0, xmax = 0, ymax = 0;
  int f = 0, L = 0, T = 0, mw = 0, mh = 0;
  if (!TIFFGetField(tif, TIFFTAG_IMAGEDESCRIPTION, &desc) || !desc ||
    std::sscanf(desc, "laika-geotiff v1 res=%lf f=%d L=%d T=%d mw=%d mh=%d xmax=%lf ymax=%lf",
    &res, &f, &L, &T, &mw, &mh, &xmax, &ymax) != 8)
  {
    return "Este GeoTIFF no trae la geometría de Laika (se exportó con una versión anterior del dashboard): expórtalo de nuevo.";
  }
  if (spp != 3 || bps != 8 || f < 1 || res <= 0 || mh % f || mw % f ||
    L + mw > static_cast<int>(W) || T + mh > static_cast<int>(H)) {return "GeoTIFF inconsistente: " + path;}

  g = GridData();
  g.width = mh / f;   // filas ↔ eje X
  g.height = mw / f;  // columnas ↔ eje Y
  g.res = res;
  g.ox = xmax - g.width * res;
  g.oy = ymax - g.height * res;
  g.data.assign(static_cast<size_t>(g.width) * g.height, 0);
  std::vector<uint8_t> line(TIFFScanlineSize(tif));
  const auto isOcc = [](const uint8_t * p) {return p[0] == 0 && p[1] == 40 && p[2] == 120;};
  const auto isUnk = [](const uint8_t * p) {
      return (p[0] == 226 && p[1] == 226 && p[2] == 227) || (p[0] == 237 && p[1] == 237 && p[2] == 238);
    };
  std::vector<uint8_t> occ(g.data.size(), 0), unk(g.data.size(), 0);
  for (int r = 0; r < mh; ++r) {
    if (TIFFReadScanline(tif, line.data(), T + r) < 0) {return "Error al leer " + path;}
    const int i = g.width - 1 - r / f;
    for (int c = 0; c < mw; ++c) {
      const int j = g.height - 1 - c / f;
      const uint8_t * px = &line[3 * (L + c)];
      const size_t k = static_cast<size_t>(j) * g.width + i;
      if (isOcc(px)) {occ[k] = 1;} else if (isUnk(px)) {unk[k] = 1;}
    }
  }
  for (size_t k = 0; k < g.data.size(); ++k) {g.data[k] = occ[k] ? 100 : (unk[k] ? -1 : 0);}
  finishStats(g);
  return QString();
}

// ---------------------------------------------------------------------------
// PLY del dashboard (ascii; x y z primero) -> rejilla
// ---------------------------------------------------------------------------
QString plyToGrid(const QString & path, const PlyParams & p, GridData & g, std::vector<float> & cloud, QString * note)
{
  FILE * f = std::fopen(path.toLocal8Bit().constData(), "r");
  if (!f) {return "No se pudo abrir " + path;}
  struct Closer {FILE * f; ~Closer() {std::fclose(f);}} closer{f};
  std::vector<char> buf(1 << 16);
  size_t n = 0;
  bool ascii = false, header_ok = false;
  while (std::fgets(buf.data(), buf.size(), f)) {
    if (std::strncmp(buf.data(), "format ascii", 12) == 0) {ascii = true;}
    std::sscanf(buf.data(), "element vertex %zu", &n);
    if (std::strncmp(buf.data(), "end_header", 10) == 0) {header_ok = true; break;}
  }
  if (!header_ok || !ascii || n == 0) {return "PLY no válido (solo ascii exportado por el dashboard): " + path;}
  cloud.clear();
  cloud.reserve(n * 3);
  for (size_t i = 0; i < n && std::fgets(buf.data(), buf.size(), f); ++i) {
    float x, y, z;
    if (std::sscanf(buf.data(), "%f %f %f", &x, &y, &z) == 3 && std::isfinite(x + y + z)) {
      cloud.insert(cloud.end(), {x, y, z});
    }
  }
  const size_t m = cloud.size() / 3;
  if (m < 100) {return "PLY con muy pocos puntos";}

  // Piso: percentil bajo de z
  std::vector<float> zs(m);
  for (size_t i = 0; i < m; ++i) {zs[i] = cloud[3 * i + 2];}
  const size_t kth = static_cast<size_t>(m * p.floor_percentile / 100.0);
  std::nth_element(zs.begin(), zs.begin() + kth, zs.end());
  const double floor_z = zs[kth];

  float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
  for (size_t i = 0; i < m; ++i) {
    x0 = std::min(x0, cloud[3 * i]); x1 = std::max(x1, cloud[3 * i]);
    y0 = std::min(y0, cloud[3 * i + 1]); y1 = std::max(y1, cloud[3 * i + 1]);
  }
  const double margin = 0.5;
  g = GridData();
  g.res = p.res;
  g.ox = std::floor((x0 - margin) / p.res) * p.res;
  g.oy = std::floor((y0 - margin) / p.res) * p.res;
  g.width = static_cast<int>(std::ceil((x1 + margin - g.ox) / p.res));
  g.height = static_cast<int>(std::ceil((y1 + margin - g.oy) / p.res));
  if (static_cast<double>(g.width) * g.height > 6e7) {return "El PLY abarca un área demasiado grande";}
  const size_t cells = static_cast<size_t>(g.width) * g.height;
  std::vector<uint16_t> in_band(cells, 0);
  std::vector<uint8_t> floor_c(cells, 0);
  for (size_t i = 0; i < m; ++i) {
    const int ci = static_cast<int>((cloud[3 * i] - g.ox) / p.res), cj = static_cast<int>((cloud[3 * i + 1] - g.oy) / p.res);
    if (ci < 0 || cj < 0 || ci >= g.width || cj >= g.height) {continue;}
    const double h = cloud[3 * i + 2] - floor_z;
    const size_t k = static_cast<size_t>(cj) * g.width + ci;
    if (h >= p.band_min && h <= p.band_max) {if (in_band[k] < 65535) {++in_band[k];}}
    else if (std::abs(h) <= p.floor_tol) {floor_c[k] = 1;}
  }
  g.data.assign(cells, -1);
  for (size_t k = 0; k < cells; ++k) {
    if (in_band[k] >= p.min_points) {g.data[k] = 100;} else if (floor_c[k]) {g.data[k] = 0;}
  }
  finishStats(g);
  if (note) {
    *note = QString("PLY %1 puntos, piso z=%2 m, banda %3-%4 m sobre el piso").arg(m).arg(floor_z, 0, 'f', 2)
      .arg(p.band_min).arg(p.band_max);
  }
  return QString();
}

LoadedMap loadMap(const QString & file, const QString & out_dir, const PlyParams & p)
{
  LoadedMap r;
  const QFileInfo fi(file);
  const QString ext = fi.suffix().toLower();
  const QString prefix = QDir(out_dir).filePath(fi.completeBaseName());
  QDir().mkpath(out_dir);
  if (ext == "yaml" || ext == "yml") {
    r.error = readPgmYaml(file, r.grid);
    r.yaml = file;
    r.note = "Mapa .yaml/.pgm directo";
    return r;
  }
  if (ext == "tif" || ext == "tiff") {r.error = geotiffToGrid(file, r.grid); r.note = "GeoTIFF decodificado";}
  else if (ext == "ply") {r.error = plyToGrid(file, p, r.grid, r.cloud, &r.note);}
  else {r.error = "Formato no soportado: ." + ext + " (usa .yaml, .tiff o .ply)";}
  if (!r.error.isEmpty()) {return r;}
  r.error = writePgmYaml(prefix, r.grid);
  r.yaml = prefix + ".yaml";
  return r;
}
