# Terrain plan, batch 2 (advisor, 2026-11-19)

Status: **waiting for user approval.** Sources: `plan-terreno-complejo.md`, `plan-terreno-lote1.md`, the 2026-11-19 entries in `decisiones.md`, `odom_to_tf.py`, `tf/scan/mapping/localization/fast_lio.launch.py`, `fast_lio_l1.yaml`, `pc2scan.yaml`, `slam_async.yaml`, `nav2_common.yaml`, `cmd_vel_to_sport.py`.

## 1. Fixes to the plan
1. **LIO's pose is the body IMU pose; its world frame `camera_init` is the IMU frame at start-up, probably not gravity-aligned.** For `odom -> base_link`: base = p_imu + R·(0.02557, 0, −0.04232), then `R_align` for gravity. Check first on loop1: if the floor normal in `/lio/cloud_registered` is within ≤ 0.5° of +z, `R_align` = I.
2. **A gravity-aligned height band does not fix ramps** (Unitree odom was already gravity-aligned). Ramps belong to P6/P7. In P5.5 the scan keeps today's band: `z_min_rel` −0.12, `z_max_rel` 0.50, `min_dist` 0.6, 10 clouds, 5 Hz, `pc2scan.yaml` unchanged.
3. **No scan from `/lio/cloud_registered` with `dense_publish_en: false`** (0.2 m voxels). Set `dense_publish_en: true`, re-measure CPU.
4. **Replace `lio_to_tf` + `lio_scan_builder` with `lio_adapter` + a second `odom_to_tf` instance** (reuses window/TF/offset code). LIO stamps are on the robot clock; the second instance's offset ≈ latency and shifts TF and cloud equally. No extra offset.
5. **Under `--odom-lio`, `/laika/odom` stays leg odometry** (Nav2 uses only its body-frame twist). `map3d_accumulator` integrates `cloud_deskewed` (Unitree odom), inconsistent with LIO `map->odom`: don't run it with `--odom-lio` (decision 4).
6. **R2 step 1:** in cable mode the bridge stays ACTIVE after a cancel. Reset requires IDLE; otherwise FAIL "pulsa L2+Y".
7. **R2 step 5:** slam_toolbox on Foxy has no lifecycle/reset. Restart the process; no launch `respawn=True` (it would silently start a new map on a crash).
8. **`/laika/reset_odom`** must also re-express and flush the cloud window. Only the `odom_to_tf` instance that publishes TF creates the service.
9. **P5.3:** loop1 already passes. Tune only if b2 fails; no open-ended tuning.
10. **Static TF `base_link -> utlidar_lidar`** (URDF Ry 164.9°) is ~120° off in yaw. Only RViz uses it (decision 5).

## 2. Batch 2
Order: **C1** → C2 → … → C9. C8-C9 (R2) if there is time; C8 doesn't depend on C4-C7.
Rule: `laika_fase4.sh` with no new flag behaves exactly as today. New flags: `--lio` (shadow), `--odom-lio`, `--sesion`.

### C1. B2 stress bag (USER, joystick)
`~/grabar_bag_crudo.sh b2_estres`. Start/end at a tape mark. Trot 0.45-0.5, sharp turns, door threshold, mat, small bumps. At the end 3 fast spins in place and 3 abrupt starts/stops (IMU latency under fast motion). `LEEME.md` filled in.

### C2. P5.3 offline evaluation
- `scripts/evaluar_lio.py` (Foxy node on the robot; from the loop1 evaluation if it exists): subscribes to `/lio/odometry` and `/bag/utlidar/robot_odom`; on Ctrl-C prints 3D return error + yaw (LIO and leg), direction of travel vs +x (median and max, stretches ≥ 1 m), divergence (|v| > 2 m/s or z jump > 0.3 m), Hz, CPU.
- Body-IMU latency on b2 fast stretches (L1 gyro vs body gyro, as in `calib_l1_lidar.py`).
- Gates: loop1 unchanged (≤ 8 cm / 1.5°); b2 no divergence, LIO error ≤ leg error; latency ≤ 10 ms, otherwise set it in `lowstate_imu_relay.yaml`.
- If b2 fails, one change at a time, with a table: (1) `gyr_cov`/`acc_cov` 1.0; (2) `b_gyr_cov` 0.01; (3) `filter_size_surf`/`filter_size_map` 0.3; (4) `max_iteration` 4.
- Command: `ros2 launch laika_slam fast_lio.launch.py bag:=true` + `~/reproducir_bag.sh b2_estres` + `evaluar_lio`.

### C3. P5.4 shadow mode (`--lio`)
- `laika_slam/lio_compare.py` (entry `lio_compare`): `/lio/odometry` (to base_link as in fix 1) + TF `map->base_link`; first valid pair sets the 2D alignment; logs `dxy`/`dyaw` every 5 s; CSV `~/laika_bags/lio_compare_<date>.csv` (`t, lio_x, lio_y, lio_yaw, map_x, map_y, map_yaw`).
- `fast_lio.launch.py` argument `compare` (default false).
- `laika_fase4.sh --lio`: new pane `lio`: `ssh -t $ROBOT "$ENV_R && echo 'No mover el robot 5 s' && ros2 launch laika_slam fast_lio.launch.py compare:=true"`. Refuse `--lio` with `--sin-cable`.
- Acceptance without walking: `--prueba --slam --lio` → `/lio/odometry` ≈ 15 Hz, no `camera_init` on `/tf`; `laika_fase4_parar.sh` leaves no `fastlio_mapping`; without `--lio`, 6 panes and no fastlio.
- Live gate (**USER walks the loop**, dry-run): LIO return ≤ 8 cm / 1.5°; `fastlio_mapping` ≤ 1 core; total ≤ 70 % (`tegrastats`); record static init time.

### C4. `odom_to_tf` parameters (approved; default unchanged)
- `odom_topic` (default `/utlidar/robot_odom`), `cloud_topic` (default `/utlidar/cloud_deskewed`; `''` = no cloud/window), `publish_tf` (default true; false = no `/tf`, still `/laika/odom`).
- Passed as a dict in the launch (no yaml; avoids the Foxy override problem).
- Acceptance: `ros2 node info /odom_to_tf` identical with the base launch; pytest green.

### C5. `lio_adapter` + launch files
- `laika_slam/lio_adapter.py`: subscribes to `/lio/odometry`, `/lio/cloud_registered` (reliable), `/laika/body_imu` (best effort). `R_align` from the mean accel of the first 250 IMU samples (std > 0.3 m/s² → warn "robot moviéndose al iniciar LIO" and retry); parameter `gravity_align` from the fix-1 check. Publishes `/laika/lio/odom` (`odom -> base_link`, LIO stamps, twist 0) and `/laika/lio/cloud` (xyz float32 in `odom`). ≤ 15 % of a core.
- `fast_lio_l1.yaml`: `dense_publish_en: true`.
- `tf.launch.py` and `mapping.launch.py`: arguments `odom` (`leg`|`lio`, default `leg`) and `bag` (default false). With `bag`: leg `odom_to_tf` inputs from `/bag/utlidar/*`, `fast_lio bag:=true`. With `lio`: include `fast_lio.launch.py` + `lio_adapter`; leg `odom_to_tf` with `{publish_tf: false, cloud_topic: ''}`; `odom_to_tf` named `lio_odom_to_tf` with `{odom_topic: /laika/lio/odom, cloud_topic: /laika/lio/cloud}`, remap `/laika/odom` → `/laika/lio/odom_nav`. `scan.launch.py` unchanged.
- Acceptance: `mapping.launch.py odom:=lio bag:=true` + loop1 replay: `tf2_echo odom base_link` ≈ 15 Hz; `/scan` ≥ 4.8 Hz; floor normal in `/laika/lio/cloud` ≤ 0.5° from z.

### C6. P5.5 replay gate
loop1 and b2 with `odom:=lio` and `odom:=leg`; save each map with `scripts/guardar_mapa.sh lote2_<bag>_<odom>`. Gates: slam return ≤ 8 cm / 1.5°; no doubled walls on b2 (USER checks; ≤ 2 cells); `/scan` 5 Hz without gaps; `grep -i extrapolat` on the slam log empty.

### C7. `--odom-lio` live, dry-run
- `laika_fase4.sh --odom-lio`: `odom:=lio` in `CMD_BASE` (mapping and localization; `localization.launch.py` passes it to `tf.launch.py`). No `lio` pane. Refuse with `--sin-cable`.
- Gate (**USER walks the loop**, `--prueba --slam --odom-lio`): as C6, live; CPU ≤ 70 %.
- **Armed navigation on LIO is NOT in this batch** (batch 3, own approval).

### C8. R2a: approved hooks
- `odom_to_tf`: Trigger `/laika/reset_odom` (only when `publish_tf`). Stores (x, y, yaw) ref; `/tf`, `/laika/odom` and window go out as Rz(−yaw_ref)·(p − p_ref), yaw − yaw_ref; z, roll, pitch, twist untouched; flush window; `offset = None`. Parameter `test_yaw_jump_deg` (default 0); non-zero creates `/laika/inject_yaw_jump`. Pure functions + pytest roundtrip.
- `map3d_accumulator`, `heightmap_node`: `~/reset` Trigger clears state.
- Bridge (approved): subscribes to `/laika/session_id` (Int32, reliable, transient_local, depth 1). On a change after the first value: if not IDLE, L2+Y path (StopMove + IDLE); clear `goal`, `goal_msg`, `approved_key`; `sc.on_session_reset()` (new in `sport_logic.SinCable`: IDLE + approval consumed). Tests in `test_cmd_vel_to_sport.py`: approve → session change → R2+X doesn't enter RUN, goal not republished; session change while ACTIVE → StopMove.

### C9. R2b: `session_manager` (`--sesion`, only with `--slam`)
- `laika_slam/session_manager.py` owns slam_toolbox as a subprocess (`ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file .../slam_async.yaml`). `mapping.launch.py` argument `sesion` (default false; when true the launch doesn't start slam). SIGINT terminates the child.
- `/laika/reset_session` (Trigger), one PASS/FAIL line per step, < 10 s:
  1. `/laika/bridge_state` IDLE (< 1 s old), else FAIL "pulsa L2+Y".
  2. Cancel all Nav2 goals (`/navigate_to_pose/_action/cancel_goal`, zero goal_id and stamp).
  3. `map_saver_cli -f ~/laika_maps/autosave/<ts>_failed` (5 s timeout).
  4. Stop slam child (SIGINT; kill after 5 s).
  5. `/laika/reset_odom`.
  6. Clear costmaps, reset map3d/heightmap (missing services not fatal).
  7. `session_id += 1`.
  8. Restart slam; wait for `/map` and `map->base_link` (8 s).
- Dashboard: "Reiniciar sesión" button (confirmation; only when IDLE; async, 15 s timeout; shows the lines). Subscribes to `/laika/session_id`; on change: `clearMission()` and clear cached `/map`, costmaps, plan, `appr.approved`.
- `laika_fase4.sh --sesion` → `sesion:=true`; with `--prueba` also `test_yaw_jump_deg:=20`.
- Acceptance (robot standing, `--prueba --slam --sesion`, 3 times): `ros2 service call /laika/inject_yaw_jump std_srvs/srv/Trigger` → doubled map; Reiniciar sesión → clean map in < 10 s, `map->base_link` ≈ identity, `/scan` 5 Hz, dashboard caches empty, no `/goal_pose` republished.

## 3. Steps that need the user
C1 record b2 (joystick); C3 shadow walk (joystick, dry-run); C6 visual check of b2 maps; C7 walk with `--odom-lio` (joystick, dry-run). No step arms the bridge. Armed navigation on LIO and after a reset go to batch 3, each with its own approval.

## 4. Open decisions (recommendation)
1. Default odometry stays leg until an armed S1 regression on LIO passes (batch 3)? **Yes.**
2. `--lio` / `--odom-lio` incompatible with `--sin-cable` for now? **Yes.**
3. Under `--odom-lio`, `/laika/odom` (Nav2 twist) stays leg? **Yes.**
4. `map3d_accumulator` under `--odom-lio`: don't run it in batch 2; switch its input to `/laika/lio/cloud` in batch 3? **Yes.**
5. Fix the static TF `base_link -> utlidar_lidar` with the new `R_body_lidar` (RViz only)? **Yes, one-line commit.**
6. Reset only in IDLE (user presses L2+Y first)? **Yes.**
7. Dashboard "Arrancar" passes `--sesion` with Mapa = En vivo, after C9 passes? **Yes.**

Not checked: whether Ericsii aligns `camera_init` with gravity (C5 checks), CPU cost of `dense_publish_en`, `sport_logic.SinCable` internals for `on_session_reset`. Risk: LIO TF at 15 Hz could trip slam's `transform_timeout` 0.2 if LIO lags; C6's extrapolation grep detects it.
