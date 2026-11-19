# Terrain plan, batch 1 (advisor, 2026-11-19)

Status: **waiting for user approval.** Derived from `docs/plan-terreno-complejo.md`.

Sources: `docs/plan-terreno-complejo.md`, `docs/decisiones.md` (F9, IMU 2026-11-16), `l1_to_body.py/.yaml`, `point_lio.launch.py`, `map3d_accumulator.py`, `scripts/{grabar_bag_crudo,reproducir_bag,analizar_imu}.*`, dashboard `src/ros_bridge.cpp`/`main_window.cpp`, and the source of `Ericsii/FAST_LIO_ROS2` (`ros2` branch, `src/laserMapping.cpp`).

## 1. Fixes to the plan before implementation
1. **FAST_LIO_ROS2 has no parameter to turn TF off.** It always broadcasts `camera_init -> body`; frame names are hardcoded, and `livox_ros_driver2/msg/custom_msg.hpp` is included with no guard. Fix without patching upstream: remap `/tf:=/lio/tf` in the launch file, and add a messages-only stub package `livox_ros_driver2`. P5.1 should read "remap TF", not "disable TF".
2. **FAST-LIO cannot run with `zero_acc: true`** (initialisation normalises the mean accel). So the P5.1 "runs on replay" gate depends on the accel rotation from P5.2. Order: P5.2-A offline, then the P5.1 run. Accel rotation = candidate **B** gives a vertical accel (−0.10, 0, 10.13); B·A is a 180° turn about an axis near x, so the open question is discrete: B or Rz(π)·B.
3. **Bags:** topic list is missing `/utlidar/cloud_deskewed` (needed by P6 and the P5.1 fallback). `/tf`, `/tf_static`, `/scan`, `/map`, `/lowstate` from a bag collide with live topics in domain 0 unless remapped under `/bag/...`.
4. **Option B (`/lowstate`, 500 Hz) in Python costs ~70 % of a core** (measured on the bridge before `/lf/lowstate`). Option B needs a small C++ relay. Do Option A first.
5. **Clock (fact #3 is inexact):** `l1_to_body` restamps cloud and IMU to the robot clock with its own offset, so LIO output is NOT in the `.161` clock. In P5.5, `lio_to_tf` uses those stamps as they are; do not add `odom_to_tf`'s offset again.
6. **R1 safety:** clearing the local costmap during a run erases obstacles until the next scan. Enable the button only when the bridge is not moving.
7. **P6 on the robot:** numpy 1.17.4, where `np.add.at` is very slow. Use `lexsort` + `np.unique(return_index)` / `bincount`.

## 2. Batch 1 (no walking, no sudo, no bridge/odom_to_tf/systemd)
Order: **B1** R1 → **B2** bag tooling → **B3** accel analysis + `acc_R` → **B4** fetch/build FAST-LIO → **B5** FAST-LIO config + replay → **B6** heightmap prototype (independent; can run in parallel with B3-B5).
Batch 1 adds **no flag** to `laika_fase4.sh`; `--lio` arrives in batch 2 (P5.4 shadow mode). Global check: `git diff --stat main -- scripts/laika_fase4.sh ros2_ws/src/laika_slam/laika_slam/{cmd_vel_to_sport,odom_to_tf,sport_logic}.py ros2_ws/src/laika_slam/config/l1_to_body.yaml` is empty.

### B1. R1 dashboard (C++, external only)
- `ros_bridge.hpp/.cpp`:
  - `clearTrail()`: lock `pose_mx`, `path.clear()`, `pose_version++`.
  - `clearCostmaps()`: two `rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>` on `/global_costmap/clear_entirely_global_costmap` and `/local_costmap/clear_entirely_local_costmap`; `async_send_request` (never block the UI); report how many were ready and how many answered within 2 s. Follow the `octomap_reset_` pattern.
  - Add `nav2_msgs` to `CMakeLists.txt` and `package.xml`.
- `main_window.cpp`, next to "Reiniciar misión":
  - "Borrar trayectoria": no confirmation; toast.
  - "Limpiar costmaps": enabled only when `/laika/bridge_state` contains neither `ACTIVE` nor `RUN` (or nothing received); toast "Costmaps limpiados (n/2)" or "sin respuesta".
- Acceptance: (a) `colcon build --packages-select autonomy_dashboard_pkg` passes. (b) Fake nav in domain 77: "Borrar trayectoria" empties the 2D and 3D trail. (c) `laika_fase4.sh --prueba --slam` (dry-run): `ros2 service list | grep clear_entirely` shows both; the button reports 2/2 (also tests Foxy↔Humble services; if they time out, report it, do not work around it).

### B2. Bag tooling (P5.0)
- `scripts/grabar_bag_crudo.sh` (modify):
  - Topics: `/utlidar/cloud /utlidar/imu /utlidar/robot_odom /utlidar/cloud_deskewed /lowstate /sportmodestate /tf /tf_static /scan /map`.
  - Check `df --output=avail -BG ~`; refuse below 10 GB; print the estimate (~0.6 GB/min).
  - No `exec`. Use `trap ':' INT` (not `trap '' INT`).
  - After recording, write `~/laika_bags/<name>/LEEME.md` (date, bag B1-B6, scene, marks, measured ground truth, operator/spotter, notes).
- `scripts/reproducir_bag.sh` (modify): remap `/utlidar/cloud_deskewed /lowstate /sportmodestate /tf /tf_static /scan /map` to `/bag/...`. Header note: F9 = no live LIO while a bag plays.
- Acceptance: `bash -n` on both. On the robot, 10 s recording + Ctrl-C: `ros2 bag info` lists the topics, `LEEME.md` exists, then remove it. Faked low disk refuses.

### B3. P5.2-A offline: accelerometer rotation
- New `scripts/analizar_acel.py <bag>`, importing `leer`, `st`, `CAND` from `analizar_imu.py`; read-only on the `.db3`; run on `fase2_cuarto1`.
  - Candidates: `B` and `Rz(π)·B`.
  - Static segment (odom |v| < 0.02, |ω| < 0.02): mean accel, norm, angle from +z.
  - Walking: corr(accel x low-passed 2 Hz, d/dt odom `twist.linear.x`); corr(accel-implied tilt rate, gyro x/y with A).
  - Print the chosen matrix; exit 1 if no candidate has corr > 0.5 in both.
- `l1_to_body.py`: new param `acc_R` (default = `imu_R`, so behaviour is unchanged), applied to accel only. New `config/l1_to_body_lio.yaml`: copy with `zero_acc: false` and `acc_R` = result.
- Acceptance: static norm ~10.1 and angle < 3° for the chosen candidate, plus both correlations; `pytest ros2_ws/src/laika_slam/test` green. If inconclusive, record and stop (needs a tilt test with the user).

### B4. Fetch and build FAST_LIO_ROS2 (robot, no sudo)
- `scripts/traer_fast_lio.sh`: on the external, `git clone --recursive` `Ericsii/FAST_LIO_ROS2` (`ros2` branch) into `~/laika_vendor/` at a pinned SHA; apply `patches/fast_lio_foxy.patch` if present; `rsync` to `unitree@192.168.123.18:laika_ws/src/FAST_LIO_ROS2`.
- New package `ros2_ws/src/livox_ros_driver2`: `ament_cmake` + `rosidl`, `CustomPoint.msg` and `CustomMsg.msg` copied from upstream (MIT, cite source).
- On the robot, idle: `source ~/laika_ros_env.sh && cd ~/laika_ws && MAKEFLAGS=-j2 colcon build --packages-select livox_ros_driver2 fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release`. No `apt`; if a dependency is missing, stop and report.
- Foxy compile errors: minimal patch in `patches/` + one line in `decisiones.md` per change.
- Acceptance: `ros2 pkg executables fast_lio` lists `fastlio_mapping`; `unitree_go`/`unitree_api`/`laika_basics` still in `~/laika_ws/install`.

### B5. FAST-LIO config + replay (P5.1)
- `launch/fast_lio.launch.py`: `bag` arg as in `point_lio.launch.py`; `l1_to_body` with `l1_to_body_lio.yaml`; node `fast_lio/fastlio_mapping` named `laser_mapping` with `config/fast_lio_l1.yaml` (key `laser_mapping:`). Remaps: `/tf:=/lio/tf`, `/Odometry:=/lio/odometry`, `/cloud_registered:=/lio/cloud_registered`, `/cloud_registered_body:=/lio/cloud_registered_body`, `/path:=/lio/path`, `/map_save:=/lio/map_save`.
- `config/fast_lio_l1.yaml` (start point; tuning is P5.3): `lid_topic /utlidar/transformed_cloud`, `imu_topic /utlidar/transformed_imu`, `time_offset_lidar_to_imu 0.0`, `lidar_type 2`, `timestamp_unit 0`, `scan_rate 15`, `scan_line` = max(ring)+1 from the bag, `blind 0.3`, `point_filter_num 2`, `filter_size_surf 0.2`, `filter_size_map 0.2`, `cube_side_length 50`, `det_range 20`, `fov_degree 360`, `max_iteration 3`, `gyr_cov 0.5`, `acc_cov 0.5`, `b_gyr_cov 0.001`, `b_acc_cov 0.0001`, `extrinsic_est_en false`, `extrinsic_T [0,0,0]`, `extrinsic_R` identity, `dense_publish_en false`, `scan_bodyframe_pub_en false`, `map_en false`, `pcd_save_en false`.
- Acceptance on the robot (stack stopped): `fast_lio.launch.py bag:=true` + `~/reproducir_bag.sh fase2_cuarto1`; `/lio/odometry` ≈ 15 Hz (if < 12, best-effort QoS losing fragments → one-line patch to reliable); no crash for the whole bag; no `/Odometry|/cloud_registered|/path` topics and no `camera_init` on `/tf`; record CPU and final position.

### B6. P6 prototype `heightmap_node` (Python, laika_slam)
- New: `laika_slam/heightmap_node.py`, entry point in `setup.py`, `config/heightmap.yaml`, `test/test_heightmap.py`; pure functions, `map3d_accumulator` style.
- Inputs: `/utlidar/cloud_deskewed` (odom, reliable QoS) and `/utlidar/robot_odom` (same clock, no TF/offset). `odom` assumed gravity-aligned (the test confirms it).
- Grid: `odom`-aligned, 0.05 m, ±4 m (160×160); recentre in whole cells when > 1 m from the centre (new cells NaN); batches at 5 Hz; drop points above `base_z + 1.5` and inside the ±0.38×±0.18 m footprint.
- Per cell: ground = 3rd-lowest z of the batch (≥ 3 pts) as EMA α 0.3, restart if stale > 60 s; `top` = max z in (ground, ground+0.50], 2 s memory; `overhead` = min z > ground+0.50, 2 s memory; `count`, `last_seen`.
- Output: `/laika/heightmap_cloud` (PointCloud2 in `odom`, fields x, y, z=ground, top, overhead).
- Log every 10 s: ms per batch, valid cells, floor std within 1.5 m.
- Acceptance: pytest (plane 1 cm noise + 10 cm box → RMS < 1.5 cm, step 10 ± 2 cm); on the robot with `reproducir_bag.sh cuarto1`: ~5 Hz, CPU ≤ 40 % of a core, floor std ≤ 1.5 cm.

## 3. Decisions for the user (block later batches)
1. Build FAST_LIO_ROS2 + stub `livox_ros_driver2` in the robot's `~/laika_ws`, no sudo, ~20-30 min CPU with the robot idle? **Rec: yes.**
2. Record B2-B4 walking with the joystick (no autonomy); B5/B6 (drop, stairs) later with a spotter? **Rec: yes.**
3. IMU: Option A first; Option B only if A fails, with a C++ relay? **Rec: yes.**
4. If B3 is inconclusive: tilt test standing (±10° from joystick) with a bag? **Rec: yes, only if needed.**
5. `odom_to_tf`: add `publish_tf` (default true) and `/laika/reset_odom`? **Rec: yes, default unchanged.**
6. Height map as PointCloud2 now; decide `laika_msgs` vs `grid_map` in P7; no apt `grid_map`? **Rec: yes.**
7. Persistent terrain map (7c)? **Rec: postpone until after P8.4.**
8. Automatic stop on BAD health? **Rec: no, banner only.**
9. Bridge hook to forget cached goal/approval on reset (R2)? **Rec: yes, with a unit test.**
10. Stairs (P8.6) and controller reset (R4)? **Rec: postpone.**

Not checked: FAST_LIO_ROS2 on Foxy, L1 cloud field layout (`ring`), disk throughput, Foxy↔Humble services (B1-c tests it). Risk: if B3 does not resolve the accel sign, P5.1's runtime gate waits for a tilt test.
