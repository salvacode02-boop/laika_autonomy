# Plan: rough-terrain autonomy (Phases 5-8) + session-reset track

Drafted 2026-11-18 from `decisiones.md` (entries up to 2026-11-18). Status: **draft, waiting for approval**. Phase 4 (flat-ground Nav2 autonomy) is closed and is the baseline for everything below.

## 0. Ground rules

1. **Wired workflow stays as is.** Goal is sent with the cable, the cable is unplugged, R2+X starts the run. This plan does NOT touch the Tailscale / zenoh / `tf_lite` work. Anything that would need to touch it is listed in section 7 ("Ask first").
2. **Everything runs on the robot's Orin NX (4 active cores).** Gate after every phase: total CPU sustained <= 70 % of the 4 cores and no thermal throttling (check `tegrastats`). Known loads from the log: `odom_to_tf` ~35 % of a core, `map3d_accumulator` 14-31 %, bridge ~8 %.
3. **The actuator is still `cmd_vel_to_sport`** with its current safety logic (R2+X exactly 1040 to arm, L2+Y stop, `0x55 0x51` header watchdog, 90 s RUN limit, joystick ADDS to the command). No phase may weaken it.
4. **Feature flags, not replacements.** `laika_fase4.sh` with no new flag must behave exactly as today. New pieces come in as `--lio`, `--terrain`, etc. Rollback = drop the flag.
5. **Every phase ends with measurable gates and a `decisiones.md` entry** in the existing format.
6. **Reuse what exists:** bags `caminata2`, `cuarto1`, `fase2_cuarto1`; the IMU analysis (`scripts/analizar_imu.py`); `l1_to_body`; `ros2 bag play --remap` in domain 0 (decision F9: no live LIO while a bag is playing); `map3d_accumulator`; the dashboard.

## 1. What the log tells us (facts that shape the plan)

| # | Fact from the log | Consequence |
|---|---|---|
| 1 | L1 IMU: with candidate A the gyro is right (corr_z 0.987) but the static accel is (5.01, 0, -8.80), not vertical; with B the accel is vertical but the gyro sign is wrong. Point-LIO only survived the accel problem because of `zero_acc: true`. | **Accel and gyro do not share a frame.** FAST-LIO2 uses the accel for gravity alignment and propagation, so it will drift in yaw/pitch exactly like Point-LIO did (~0.25 rad/s) unless this is fixed first. This is Phase 5's main risk. |
| 2 | Gyro bias of about -0.06 rad/s appears only while walking; gyro scale is fine (slope ~1.0). | The filter must keep estimating bias online; process-noise values need to be raised for gait vibration. |
| 3 | Cloud and IMU share one clock (first cloud - nearest IMU = -54.6 ms). Cloud stamp = START of the sweep; per-point `time` 0-62.4 ms. The `.161` clock is ~2066 s behind the robot Jetson. | LIO can run entirely in the `.161` clock (cloud + IMU come from the same source). Its output TF must get the same common offset that `odom_to_tf` applies. |
| 4 | L1 sits ~0.3 m high looking down: ~10 600 pts/cloud at 15.4 Hz, 95 % within 1 m, ~190 pts above floor+0.15 m; to the sides it only sees below its own height. | Near ground is dense (good for a height map). Ground beyond ~2-3 m is sparse and grazing; **descending edges and stair treads are poorly visible.** Overhangs (table tops) are seen badly. |
| 5 | The 2D scan has 145-156 of 201 valid beams at 5 Hz over ~180° front; slam_toolbox's odometry is the closed Unitree leg odometry (`/utlidar/robot_odom`, 150 Hz). | Likely cause of the "map breaks and overlaps" failure: under vibration/slip the leg-odom yaw jumps, and a sparse scan cannot re-match. Better odometry (LIO) plus detection plus a clean reset is the fix (Phase 5 + Track R). |
| 6 | Gait dead band: vx < 0.25 does not walk reliably; spin in place needs \|wz\| >= 0.4. Speeds are 0.25-0.5 m/s. | The planner cannot "slow down for rough ground". It must **avoid** bad terrain or commit at >= 0.25 m/s. |
| 7 | `odom_to_tf` does three jobs: TF from leg odom, the common time offset, and the 10-cloud window that feeds the scan. | Swapping the odometry source means also rebuilding the scan source (see P5.5). |
| 8 | Phase 1 limitation: table tops/overhangs are missing from the 2D map; Nav2 only has a flat grid. | The height map separates ground from overhead points (Phase 6), which fixes this properly. |
| 9 | Unitree SDK documents gait types (0 idle, 1 trot, 2 run, 3 climb stair, 4 forward down stair, 9 adjust) and a foot-raise-height field; this firmware reports mode 0 / gait 0 even while walking. | Stair gaits exist in the API but must be verified on THIS firmware before any stair work (Phase 8.7). |

## 2. Phase map and order

```
P5 Better odometry/SLAM (FAST-LIO2)  ──►  P6 Height map  ──►  P7 Terrain description  ──►  P8 Terrain-aware planning
        │                                      ▲  (prototype on existing bags in parallel with P5.4-P5.6)
        └─► Track R (reset + odometry health): R1 any time, R2 after P5.4, R3 after P5.6
```

Effort: P5 = L, P6 = M, P7 = L, P8.1-8.6 = M, P8.7 (stairs) = L and optional. R1 = S, R2 = M, R3 = M.

---

## 3. Phase 5: better SLAM (FAST-LIO2 as the odometry source)

**Goal:** a 3D pose (x, y, z, roll, pitch, yaw) that does not drift in yaw under gait vibration, and that replaces leg odometry as slam_toolbox's odom when `--lio` is on.

**Gate to leave the phase:** (a) no divergence on any recorded bag, (b) return error <= 8 cm / 1.5° on the loop (equal to or better than today's 8 cm / 1.5°), (c) on the stress bag no doubled walls (wall thickness <= 2 cells), (d) CPU budget respected.

### P5.0 Datasets and ground truth
Record a standard bag set (domain 0, on the robot, topics: `/utlidar/cloud`, `/utlidar/imu`, `/utlidar/robot_odom`, `/lowstate` including IMU and foot force, `/tf`, `/sportmodestate`, `/scan`). Check disk before each (existing bags are 0.9-1.6 GB).

| Bag | Scene | Ground truth to write down |
|---|---|---|
| B1 | flat loop with return to the mark (already have `cuarto1`, `fase2_cuarto1`) | start/end marks |
| B2 | **stress:** trot at 0.45-0.5 m/s, sharp turns, door threshold, rough mat, small bumps | none (compare consistency) |
| B3 | ramp up and down (plywood, <= 10°) | length, rise, angle with a phone inclinometer |
| B4 | steps/curb of 4, 8, 12 cm (boards) | measured heights |
| B5 | edge/drop (box or landing), robot standing 1-1.5 m away, then approaching to 0.6 m, never crossing | drop height |
| B6 | short stair flight (1-2 steps) with spotter, joystick only | rise/run |

Tooling: one README per bag with the ground truth, stored in `~/laika_bags/`, plus a helper script that plays a bag with the F9 remaps.

### P5.1 Build FAST-LIO2 on the robot
- Candidate: the ROS 2 fork `Ericsii/FAST_LIO_ROS2` (supports generic PointCloud2 lidars and ARM; the Livox driver is optional). **Verify it builds on Foxy with PCL 1.10 and Eigen; if not, port the minimum.** Note the robot's apt mirror lacks some ROS packages (decision of 2026-11-16), so prefer source builds of ikd-Tree and the filter.
- Input adapter: reuse `l1_to_body` to output a Velodyne-style cloud (`x y z intensity ring time`, with `time` from the L1 per-point field, 0-62.4 ms). If the raw cloud turns out unusable, fall back to `cloud_deskewed` with `time = 0` (Unitree already deskewed it; the filter just loses its own deskew).
- **Disable the filter's own TF publishing** (it publishes `camera_init -> body` by default) so it cannot collide with the Nav2/SLAM tree. Use private frame names.
- Gate: builds, runs on `fase2_cuarto1` replay without crashing.

### P5.2 IMU conditioning (the critical step)
Evaluate two options offline with the existing IMU analysis tooling:

- **Option A: L1 IMU with separate gyro and accel rotations.** Keep the gyro rotation from candidate A (bias 0.0075, -0.0005, 0.0016; z2x 0.075, z2y -0.227). Derive a **different** accel rotation: take the static accel mean (|a| ~10.13 m/s², units confirmed m/s²) and rotate it to +z. Gravity fixes only 2 of 3 degrees of freedom, so resolve the remaining yaw about gravity by a tilt test (standing robot commanded to roll/pitch ±10°, both sensors see the same rotation) or by correlating high-passed accel with odometry acceleration while walking.
- **Option B: use the body IMU in `/lowstate`** (500 Hz, gyro and accel in the same frame by design). The L1 is rigidly mounted to the body, so only the lidar-to-body extrinsic is needed (from the URDF/measurement). Caveat: `/lowstate` has no usable source stamp, so stamp on arrival on the robot clock, apply the same common offset as `odom_to_tf`, and **measure the latency** by cross-correlating its gyro-z with the L1 gyro-z. Use `/lowstate` (500 Hz) for this and nothing else; the watchdog logic stays on `/lf/lowstate`.

Metrics for choosing: static gravity norm 9.8 ± 0.1 m/s² and vertical after the filter initialises; gyro-integrated yaw / odom yaw = 1.00 ± 0.05 during turns once bias is estimated; LIO yaw vs odom yaw over the whole bag. **Gate:** one option selected and documented, with the numbers.

### P5.3 Tuning in replay
Replay (domain 0, `--remap`, no live LIO running). Tune: surface/map filter sizes, `acc_cov`/`gyr_cov` and the bias covariances (raise for gait vibration), maximum range (15-20 m is enough indoors), self-hit filtering (legs/body can appear in the cloud: use `blind` or a box filter), iteration count, local-map cube size (memory), accel units (m/s²). Extrinsics fixed (`extrinsic_est_en: false`).
Compare on B1 and B2: yaw drift, return error, divergence. **Gate (offline):** no divergence on any bag; B1 return error <= 8 cm / 1.5°; on B2 LIO drift <= leg-odom drift.

### P5.4 Live shadow mode (`--lio`, TF untouched)
LIO runs on the robot and publishes only its own topics (`/lio/odometry`, `/lio/cloud_registered`, private frames). Log the live difference against slam_toolbox's `map -> base_link`. Walk the usual loop. **Gate:** return error <= 8 cm / 1.5°; CPU of LIO <= 1 core; total CPU <= 70 %; static initialisation takes a few seconds with the robot standing still (document it).

### P5.5 LIO as slam_toolbox's odometry (`--odom lio`)
- New node `lio_to_tf`: publishes `odom -> base_link` from LIO with the same common-offset logic as `odom_to_tf`, using the lidar-to-base extrinsic.
- New node `lio_scan_builder`: takes the LIO-registered cloud, applies the height filter in a **gravity-aligned frame** (so it works on ramps, instead of `z - z_base`), keeps the 10-cloud window at 5 Hz, and feeds the existing `pointcloud_to_laserscan` (keep `angle_max` 1.75003, 201 beams, range_min 0.6).
- Under `--odom lio`, `odom_to_tf` runs with TF publishing off (a parameter) so the two do not fight. Leg odometry is kept as a comparison topic.
- **Gate:** slam_toolbox with LIO odom on B1/B2: return error <= 8 cm / 1.5°, no overlap on the stress run, `/scan` 5 Hz with no gaps, no "extrapolation" warnings.

### P5.6 Decide the default and expose health
Document the decision. Publish `lio_health` (iteration count, effective points, residual, LIO-vs-leg yaw-rate disagreement). Keep leg odometry as the fallback.

**Risks:** (1) IMU frames (mitigated by P5.2 before anything else); (2) LIO degeneracy in a long featureless corridor (mitigation later: fuse LIO with leg-odom twist in an EKF; not planned now); (3) Foxy build problems; (4) CPU.

---

## 4. Phase 6: height map (2.5D)

**Goal:** a robot-centred elevation grid with ground height, top height, overhead clearance, observation count and age per cell, in a gravity-aligned frame.

**Prototyping can start before P5 ends,** using the existing bags (`cloud_deskewed` + leg odom) and B3-B6; final integration waits for P5.5.

### P6.1 Design (decide with the user, see section 7)
- Rolling grid ±4 m at 0.05 m (160 x 160 = 25 600 cells), axes aligned with the world frame (no resampling on rotation), shifted as the robot crosses cell boundaries.
- Per cell (numpy arrays): min z, max z, mean z, variance (Welford), count, last-seen time.
- **Ground estimate:** lowest z that has >= 3 supporting points, with outlier rejection below the floor (glass/multipath). Memory about 60 s. **Top** estimate has short memory (about 2 s) so moving people do not leave ghosts.
- **Overhead clearance:** lowest point above `ground + 0.50 m` (Go2 is 0.40 m tall plus a 0.10 m margin) is an overhang, not an obstacle. This fixes the Phase 1 table-top limitation.
- Vectorised update (`np.bincount` / `np.add.at`) on batches at 5-10 Hz. Budget <= 40 % of one core; if exceeded, move the hot loop to C++.
- Message: `grid_map` if `apt-cache` has it for Foxy on this mirror, otherwise a small custom message (`laika_msgs/HeightMap`) or a multi-channel 32FC1 image with metadata. Plus a `PointCloud2` for visualisation.

### P6.2 Implementation and dashboard layer
Node `heightmap_node` with `/laika/heightmap` (5 Hz). Dashboard: new selectable layer with a colour map and hover value (it already has a 3D robot model and loads GeoTIFF/PLY).

### P6.3 Accuracy tests (bags B3-B5, then live standing still)
| Metric | Gate |
|---|---|
| Flat floor, RMS height error | <= 1.5 cm |
| 8 cm and 12 cm steps: height error | <= 2 cm |
| Ramp slope error | <= 3° |
| Range at which an 8 cm step first appears | measure and record (expect ~1-1.5 m from the low mounting) |
| Edge/drop: range at which the missing ground is flagged | measure and record |
| CPU | <= 40 % of a core |

**Gate:** all of the above met, or limits documented with reasons.

---

## 5. Phase 7: terrain description (traversability)

**Goal:** turn the height map into "can the Go2 walk here, and how costly is it", including holes and overhangs.

### P7.1 Per-cell features
| Feature | How | Initial rule (to be calibrated in P7.3) |
|---|---|---|
| Slope | least-squares plane over 3x3-5x5 cells | cost above 6°, lethal above 15° (Unitree spec is about 30-40° climb, but the sensor noise and the dead band argue for a conservative start) |
| Step height | max \|Δground z\| to neighbours within 0.10 m | free <= 3 cm, graded 3-8 cm, lethal > 8 cm (spec is about 15-16 cm; stair gait not engaged) |
| Roughness | std of ground z in a 0.25 m window | graded cost |
| Clearance | `overhead - ground` | lethal if < 0.50 m |
| Drop / void | (a) unobserved cell inside the sensor's view, no occluder, within 1.2 m of the robot, next to observed ground; (b) ground drop > 8 cm over <= 0.10 m | **lethal, marked immediately, cleared only after ~5 consistent observations** |
| Never-seen cells | cells never observed, including the < 0.6 m blind zone | treated as unknown-dangerous. The height map keeps cells seen earlier from farther away, which already shrinks the blind-zone problem noted in the safety review. |

Noise handling: for step/slope, require 2-3 consistent observations before marking lethal; for drops be conservative as above.

### P7.2 Feeding Nav2 (staged)
- **7a (config only):** publish `/laika/terrain_obstacles` (PointCloud2 of lethal cell centres) and add it as a second `ObstacleLayer` source in the local and global costmaps (marking and clearing on). The existing `/scan` layer stays as redundancy.
- **7b (small C++ plugin):** `TraversabilityLayer` reading `/laika/traversability` and writing graded costs (1-252, lethal 254) into the master grid. Needed because Foxy 0.4.7 has no ready-made graded-cost input.
- **7c (decide later):** persistent terrain map in the `map` frame, saved next to each map (`maps/<name>_terrain.npz`), so the global planner can reason about a ramp 10 m away. The rolling grid only covers ±4 m.

### P7.3 Capability calibration (joystick, not autonomy; spotter and padding)
Walk the robot at the autonomy speeds (0.25-0.3 m/s, then 0.45) over steps of 3, 5, 8, 10, 12 cm and slopes of 5°, 8°, 12°, 15°, 20°; 5 trials each. Record success, peak pitch/roll, foot slip (commanded vs LIO velocity) and anything odd. Set each lethal threshold at about 60 % of the measured limit. Write the table to `docs/capacidad-terreno.md`. Do not test descents above 8 cm in this phase.

### P7.4 Validation
- Offline on B3-B5: classification vs ground truth; flat floor < 1 % lethal cells; zero missed steps >= 8 cm within 1.5 m.
- Live, robot standing: dashboard shows ramp = graded, 12 cm step = lethal, edge = lethal, table = free underneath.
- Dry-run Nav2: a goal across the ramp gets a plan; across a 12 cm step gets a detour or no plan.

---

## 6. Phase 8: terrain-aware planning

### P8.1 Nav2 integration and tuning
Costmap parameters for the rectangular footprint (±0.38 x ±0.18), inflation radius and cost-scaling tuned against the terrain costs, NavFn with A*. DWB/RPP maximums stay at 0.3 (dashboard up to 0.5). Because of the gait dead band, **the planner avoids bad terrain instead of slowing for it.**

### P8.2 Recovery behaviours
Spin at 0.4 rad/s next to a drop or step is risky. Edit the behaviour-tree XML so recovery is: clear costmaps, wait, short back-up, and spin only if the surrounding terrain cost is low.

### P8.3 "Why no path" feedback
When no plan exists, the dashboard should say why (step/slope/drop at a location) using the classification from P7.

### P8.4 Scenario suite (dry-run first, then armed with a spotter; 10 runs each)
| ID | Scenario | Pass |
|---|---|---|
| S1 | flat baseline (regression) | >= 9/10, same as Phase 4 |
| S2 | ramp 8° up and down | >= 8/10 |
| S3 | 4 cm threshold | >= 8/10 |
| S4 | 10 cm curb | refused or routed around 10/10 |
| S5 | edge/drop with a barrier line and spotter | 0 edge crossings |
| S6 | ramp plus obstacle | >= 8/10 |
| S7 | vibrating surface plus the Track R health monitor | no overlapped map, or detected and flagged |

Global: 0 falls, L2+Y and watchdog behave exactly as in Phase 4.

### P8.5 Decision gate on the planner
If S2-S6 pass with Nav2 plus the terrain layer, freeze it. If not, evaluate the CMU local planner and path follower (they consume the terrain cloud directly) as a drop-in that feeds the same bridge. That is a large architectural change: **ask first.**

### P8.6 Optional: stairs and multiple floors (only after P8.4 passes)
1. Detect stairs from the height map (repeating rise of ~15-18 cm and tread ~28-30 cm, >= 3 levels).
2. Operator-assisted stair mode: operator faces the stairs, R2+X, gait switched to the climb-stair gait (value 3) or forward-down-stair (value 4), fixed straight-line speed, abort on pitch/roll limits. **First verify on this firmware that the bridge can request those gaits.** Needs bridge changes: **ask first.**
3. Multi-floor: one 2D map per floor with stair transitions as waypoints; LIO keeps the 3D pose across floors.
4. Descending stairs stays out of scope until there is a depth camera, because the L1 cannot see the edge from 0.3 m.
Safety: harness or spotter, mats, 1-2 steps at first. Falls during stair tests are a known hazard for this robot (neck structure).

---

## 7. Track R: session reset and odometry health (not a priority; do when it fits)

**Problem today:** after a failed run you have to press "parar" (which takes the whole stack down) to get a clean map; and when vibration breaks the odometry, the map overlaps and stays broken.

### R1 (S, any time, zero risk): clear trail and soft clean
- Dashboard button **"Clear trail"**: purely UI, drops the drawn path history.
- Button **"Clear costmaps"**: calls Nav2's clear-entire-costmap services for the local and global costmaps (confirm exact names with `ros2 service list`; expected `/local_costmap/clear_entirely_local_costmap` and `/global_costmap/clear_entirely_global_costmap`).

### R2 (M, after P5.4): hard "Reset session" without stopping the stack
New node `session_manager` with a `std_srvs/Trigger` service `/laika/reset_session`. Sequence, with a timeout and a PASS/FAIL line per step, target < 10 s:
1. **Stop first.** Cancel the Nav2 goal (the bridge already leaves RUN when a goal's status becomes cancelled/aborted, so no bridge change is needed). Wait for `/laika/bridge_state` = IDLE; if it does not become IDLE within ~3 s, abort the reset and tell the user to press L2+Y. Never reset while the robot is moving.
2. **Autosave the failed map** (`maps/autosave/<timestamp>_failed.*`) so failures can be studied afterwards.
3. Pause slam_toolbox (`pause_new_measurements`) and clear its queue.
4. **Re-zero the odometry.** Leg odometry comes from the closed `.161` and cannot be reset there, so `odom_to_tf` gets a `/laika/reset_odom` service that stores the current pose as reference and publishes `T_ref⁻¹ · T_odom` (position and yaw; roll/pitch untouched), flushes the 10-cloud window and recomputes the time offset. Under `--odom lio`, restart the LIO node instead (needs ~3 s standing still).
5. **Reset slam_toolbox's map.** The documented service list has pause, clear-queue, save, serialize/deserialize, manual loop closure and clear-changes, but no full reset. So: check whether the node is a lifecycle node on Foxy 2.4.1 (`ros2 lifecycle get`); if yes use configure/cleanup, otherwise respawn just that process (put slam_toolbox in its own tmux pane so it can be respawned alone).
6. Clear Nav2 costmaps (R1).
7. Reset `map3d_accumulator` and the height map (add a `~/reset` Trigger service to each of our own nodes).
8. Publish `/laika/session_id` (transient_local, incremented). The dashboard clears its cached `/map` (it is latched), the 3D map, the trail, the cost layers and the approved route whenever the id changes.
9. Resume slam_toolbox. Verify: `/scan` at ~5 Hz, `map -> base_link` available and near identity, a first `/map` received. Report to the dashboard.

AMCL (saved map) variant: no slam restart and no odom re-zero. Instead save the last good `map -> base_link`, clear costmaps and trail, and re-publish it as the initial pose (optional second button "Relocalize" calls global localization).

**Stale-goal hazard:** the bridge republishes the last `/goal_pose` when it enters RUN. After a reset that goal belongs to the old map. Clean fix = a small bridge hook to forget the cached goal and the approval (**ask first**). Without touching the bridge, `session_manager` can publish a goal at the robot's current pose, which makes the cached goal harmless; it needs a fresh Enter anyway.

**Testing without breaking things:** add a dry-run parameter `inject_yaw_jump` to `odom_to_tf` to reproduce the overlapped map on demand, then verify the reset recovers it.

### R3 (M, after P5.6): odometry health monitor
Node `odom_health` publishing OK / WARN / BAD with reasons:
- map->odom jump between consecutive updates above ~0.15 m or 5° (loop closures also cause jumps, 9 appeared in one normal loop, so combine with the signals below);
- scan-to-map hit ratio below ~35 % for 3 s (AMCL runs showed 70 % mean, 50 % minimum);
- leg-vs-LIO yaw-rate disagreement above ~0.3 rad/s sustained;
- roll/pitch above ~25° (fall risk), high-passed accel RMS (vibration).
Thresholds are set from the B2 stress bag and S7. The dashboard shows a banner "map probably corrupted: reset suggested". **No automatic actions** unless approved (see below).

### R4 (optional, ask first): reset while untethered
During sin-cable runs the dashboard is unreachable, so R2 is only usable after plugging the cable back in. A controller combo to trigger a reset on the robot would need key parsing in the bridge.

**Acceptance for Track R:** reset < 10 s; three consecutive navigation tests after resets have a clean aligned map; no stale cached topics in the dashboard; normal runs unaffected; monitor CPU < 3 %.

---

## 8. Ask first (changes that touch things you said not to touch, or safety)

1. **Bridge (`cmd_vel_to_sport`):** forgetting the cached goal on reset; any controller combo for reset; gait switching for stairs; fall-abort thresholds.
2. **`odom_to_tf`:** adding a `publish_tf` parameter and `/laika/reset_odom` (small, default behaviour unchanged). Note `tf_lite` and `/laika/odom` belong to the wireless work; under `--odom lio` they would keep the leg-odometry view unless you decide otherwise.
3. **Automatic stop on BAD health** (cancel the goal when the monitor says BAD).
4. **Planner switch** to the CMU local planner/FAR if Nav2 does not pass S2-S6.
5. **Message format** for the height map (grid_map vs custom) and whether to build the persistent terrain map (7c).
6. **Any new apt installs** (mirror limits, sudo approval as in earlier phases).

## 9. Log entries to write

Each phase: a `decisiones.md` entry with the gate table filled in with measured numbers, the chosen options (IMU option, height-map format, thresholds) and the bag names used.
