#!/bin/bash
# Uso: ~/grabar_bag_crudo.sh [nombre]   (bag crudo del L1 + estado del robot, para LIO / heightmap)
# Ctrl-C detiene la grabación; después se escribe LEEME.md para anotar la escena.
source ~/laika_ros_env.sh
mkdir -p ~/laika_bags
avail=$(df --output=avail -BG ~ | tail -1 | tr -dc '0-9')
if [ "${avail:-0}" -lt 10 ]; then
  echo "Disco insuficiente: ${avail} GB libres (< 10 GB). No se graba." >&2; exit 1
fi
name=${1:-crudo-$(date +%Y%m%d-%H%M%S)}
dir=~/laika_bags/$name
echo "Libre: ${avail} GB. Estimado: ~0.6 GB/min -> ~$((avail * 10 / 6)) min. Ctrl-C para parar."
trap ':' INT   # el hijo recibe SIGINT; este shell sigue vivo para escribir LEEME.md
ros2 bag record -o "$dir" /utlidar/cloud /utlidar/imu /utlidar/robot_odom /utlidar/cloud_deskewed \
  /lowstate /sportmodestate /tf /tf_static /scan /map
cat > "$dir/LEEME.md" <<EOT
# $name
- Fecha: $(date '+%Y-%m-%d %H:%M')
- Bag (B1-B6):
- Escena:
- Marcas (tiempo, evento):
- Verdad de terreno medida:
- Operador / observador:
- Notas:
EOT
echo "LEEME.md escrito en $dir (rellénalo)."
