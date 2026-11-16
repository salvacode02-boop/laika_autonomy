# Origen (código de terceros)

- URL: https://github.com/jizhang-cmu/autonomy_stack_go2 (ruta `src/slam/point_lio_unilidar`)
- Rama: `foxy-humble`
- Commit: `43d5f54b389b251713f0097893c30fa76c870d54` (2026-02-18)
- Copiado: 2026-11-16
- Licencia: GPL-2.0 (se conserva `LICENSE`). `package.xml` declara "BSD"; se deja sin tocar.

## Cambios respecto al original

1. No se copió `Log/` ni ningún `.pcd`.
2. `CMakeLists.txt`: quitado `find_package` de PCL-ROS y PCL-ROS (2 apariciones) de `set(dependencies ...)`.
3. `package.xml`: quitados `depend` de PCL-ROS, `build_depend message_generation`, `build_export_depend message_runtime`, `exec_depend message_runtime` y `exec_depend transform_sensors`.
4. Añadido `PCD/.gitkeep`.

## Notas

- `src/laserMapping.cpp` incluye `<Python.h>` y `CMakeLists.txt` usa `PythonLibs`: se dejan (requiere `libpython3.8-dev` en el robot). Si falla, ver plan fase 2, sección 5.
- Los `launch/*.launch` y `config/*.yaml` originales no se usan; los propios van en `laika_slam` (paso 7).
