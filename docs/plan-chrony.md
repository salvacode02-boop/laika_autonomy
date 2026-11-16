# Plan: sincronización de relojes con chrony (2026-11-16, laika-advisor)

## Topología
- Externa = servidor chrony (pools de internet por WiFi cuando hay; `local stratum 10` sin internet). Robot = cliente con una sola fuente, `192.168.123.50`.
- chrony reemplaza a `systemd-timesyncd` en ambas (apt lo quita/desactiva).
- No corrige la `.161`: el offset por software de `odom_to_tf` (D3 revisada) se queda.

## Efecto del salto 1970 → 2026 (sin cambios de código)
- `odom_to_tf`, C2, `l1_to_body`, `pointcloud_to_laserscan`, `map3d_accumulator`: sin cambios (offset en enteros de Python, `map3d` usa `time.monotonic()`).
- `slam_toolbox`: si el reloj salta con el launch corriendo, reiniciar el launch.
- Bags viejos: siguen funcionando; no mezclar su `/tf` (1970) con TF en vivo (ya era así).
- Regla: revisar `chronyc tracking` antes de lanzar. Si el robot arranca sin la externa, vuelve a 1970: no compilar con `colcon` en ese estado.
- Revisar `apt-daily-upgrade.timer` / `unattended-upgrades` en el robot (solo lectura) y preguntar si se desactivan.
- Si `apt` falla por "Release file is not valid yet": `sudo date -s` a mano antes de instalar.

## Configuración
Externa, `/etc/chrony/conf.d/laika.conf`:
```
allow 192.168.123.0/24
local stratum 10
```
Robot, `/etc/chrony/chrony.conf` (chrony 3.5): comentar `pool`, reemplazar `makestep 1 3` y añadir:
```
server 192.168.123.50 iburst minpoll 2 maxpoll 4
makestep 1 -1
```

## Pasos ([U] = sudo del usuario, [S] = systemd)
0. Launches detenidos. `timedatectl` en ambas; `sudo ufw status` en la externa [U] (si está activo: abrir UDP 123 en `enP8p1s0`).
1. [U][S] Externa: instalar chrony, `laika.conf`, reiniciar chrony. Prueba: `chronyc tracking` "Leap status: Normal", timesyncd inactivo, puerto 123 abierto.
2. [U][S] Robot (internet por teléfono): instalar chrony, editar `chrony.conf`, reiniciar. Prueba: `chronyc sources` con `^* 192.168.123.50`, `date` en 2026.
3. Reiniciar el robot (aprobación). Sincroniza solo en < 1 min.
4. Externa sin WiFi + reiniciar chrony [U][S]: Stratum 10 y el robot sigue con `^*`.
5. Regresión: `/utlidar/robot_odom` sigue en 1970; `mapping.launch.py` sin "extrapolation"; `map3d bag:=true` con `cuarto1` ≈ 141 800 vóxeles, 0 descartadas.
6. Registrar en `docs/decisiones.md`.

## Terminado
Robot: `chronyc tracking` con Reference ID `192.168.123.50`, |System time| < 5 ms, Leap Normal, también tras reiniciar y sin internet en la externa. `date` coincide en ambas. Paso 5 completo.
