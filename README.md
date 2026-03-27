# Dual Open Manipulator X with Gravity Compensation & Variable Cartesian Impedance Control

---

## 🧩 Quick Start (source/build/launch, no sudo)

```bash
# 1) Source ROS 2 + workspace overlay
source /opt/ros/humble/setup.bash
source /workspaces/omx_ros2/ws/install/setup.bash

# 2) Build (use inside container or with permissions already satisfied)
cd /workspaces/omx_ros2/ws
colcon build --symlink-install --packages-select \
  open_manipulator_x_description omx_gravity_comp_controller omx_variable_stiffness_controller omx_dual_bringup

# 3) Source workspace after build
source /workspaces/omx_ros2/ws/install/setup.bash

# 4) Launch dual variable stiffness simulation (logger+live_plot)
ros2 launch omx_variable_stiffness_controller dual_gazebo_variable_stiffness.launch.py \
  gui:=true enable_logger:=true enable_live_plot:=true start_rviz:=false
```

> If your system needs dependencies, do that out of band (admin user or pre-configured container). This README now avoids `sudo` for reproducibility.

---

## ⚠️ Mandatory Launch Rules (for all LLMs and all sessions)

> **These three rules apply to every `ros2 launch` command in this project, no exceptions.**

| # | Rule | Reason |
|---|------|---------|
| 1 | **Always source before launching** | `ros2` is only on PATH after sourcing ROS 2 + workspace setup. Without this, every launch command silently fails. |
| 2 | **Always include `enable_logger:=true`** | Hardware runs must be logged. Logger and plotter are independent — logger costs nothing and every run may be the only captured data. |
| 3 | **Always include `enable_live_plot:=true`** | Visual feedback is required during any hardware session to detect anomalies in real time. |

### Canonical source + launch pattern

```bash
# Always source BOTH: system ROS 2 first, then workspace overlay
source /opt/ros/humble/setup.bash
source /workspaces/omx_ros2/install/setup.bash

# Then launch — logger and live_plot are ALWAYS on
ros2 launch <package> <launch_file>.launch.py enable_logger:=true enable_live_plot:=true [other_args]
```

### Complete hardware launch commands (copy-paste ready)

```bash
# === SINGLE gravity comp ===
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash
ros2 launch omx_dual_bringup single_robot_hardware.launch.py \
  port:=/dev/ttyUSB0 enable_logger:=true enable_live_plot:=true start_rviz:=false

# === DUAL gravity comp ===
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  enable_logger:=true enable_live_plot:=true start_rviz:=false

# === DUAL variable stiffness ===
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash
ros2 launch omx_variable_stiffness_controller dual_hardware_variable_stiffness.launch.py \
  enable_logger:=true enable_live_plot:=true start_rviz:=false

# === DUAL variable stiffness in Gazebo (with obstacle, liveplot + force logger) ===
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash
bash /workspaces/omx_ros2/tools/dual_gazebo_force_test.sh

# === DUAL opposing push test ===
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash
python3 /workspaces/omx_ros2/tools/dual_gazebo_opposing_push.py --duration 20

# Live plot with saving screenshots (headless/WSL friendly)
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash
python3 /workspaces/omx_ros2/tools/live_plot_logs.py \
  --controller variable_stiffness \
  --namespace /robot1/robot1_variable_stiffness \
  --namespace2 /robot2/robot2_variable_stiffness \
  --screenshot-dir /tmp/live_plot_screenshots
```

 and configurations for running **two independent Open Manipulator X robots** with:

## Project Status Update (2026-03-27)

See the latest session summary and recent fixes in [project_status.md](project_status.md#L1). Key highlights:
- Fixed dual-Gazebo plugin node-identity collisions that caused controller namespace cross-binding.
- Hardened the dual Gazebo 5-stage harness and improved wrench parsing; a smoke test helper is available at [tools/sample_wrench.py](tools/sample_wrench.py#L1).

- **Gravity Compensation**: Passive gravity compensation for compliant manipulation
- **Variable Cartesian Impedance Control**: Time-varying stiffness/damping profiles for precise force control

The robots can be controlled via:
- **Hardware**: Two robots connected via separate serial ports through U2D2
- **Simulation**: Gazebo simulation with RViz2 visualization
- **Auto-detection**: Automatically detect hardware or fallback to simulation

## Testing Status

| Controller | Mode | Status | Notes |
|------------|------|--------|-------|
| **Gravity Compensation** | Dual Hardware | ✅ **TESTED** | Verified Feb 2026 and Mar 2026 (logger + live_plot confirmed, 9400+ rows/robot) |
| **Gravity Compensation** | Single Hardware | ✅ **TESTED** | Verified Feb 2026 and Mar 2026 (logger + live_plot confirmed, 8099 rows) |
| **Gravity Compensation** | Dual Gazebo | ⚠️ Partial | Builds and launches; headless mode now works with fake hardware (plugin mismatch avoided). |
| **Gravity Compensation** | Single Gazebo | ✅ **TESTED** | Verified Mar 2026 — single-robot Gazebo gravity compensation works (live-plot subscription verified). |
| **Variable Stiffness** | Single Sim | ✅ **TESTED** | Verified Feb 2026 with mock hardware |
| **Variable Stiffness** | Single Hardware | ✅ **TESTED** | Verified March 2026 |
| **Variable Stiffness** | Dual Hardware | ✅ **TESTED** | Verified Mar 2026: full state machine, logger 25k+ snapshot rows + events per robot, live_plot confirmed. |
| **Variable Stiffness** | Single Gazebo | ✅ **TESTED** | Gazebo verified Mar 2026: singularity-safe IK, joint-space homing, EE stays x>0 throughout. |
| **Variable Stiffness** | Dual Gazebo | ✅ **TESTED** | Verified Mar 2026: both robots complete full state machine cycle. Requires patched libgazebo_ros2_control.so (see tools/patches/). |

**Legend:**
- ✅ **TESTED**: Verified working on actual hardware/simulation
- ⚠️ Untested: Code exists and builds, but not verified
- 🔧 **BUILD ONLY**: Implementation complete, compiles successfully, awaiting testing

## CI Status

- **Headless Tests**: ✅ **PASSING** — Unit and launch tests using `launch_gazebo:=false` (fake‑hardware) pass in the devcontainer (Feb 2026).
- **Real Gazebo**: ⚠️ **NOT RUN** — Full Gazebo integration tests require a host with `gzserver` and `gazebo_ros2_control`; these are skipped inside the devcontainer and in CI by default.

## Latest (2026-03-11 — Logger + live plotter for all hardware modes)

### Changes

- **`enable_logger`** argument added to all 3 hardware launch files (default `false`). Logs saved to `logs/<mode>/<timestamp>/`.
- **New `gc_logger.py`** for gravity-comp modes: logs joint positions, velocities, efforts to CSV.
- **`enable_live_plot`** already wired in all 3 hardware launch files (default `true`).
- **Live plotter** splits subplots across up to 3 separate figures for better readability.
- Variable stiffness logger `output_dir` changed from `/tmp/...` to workspace-relative `logs/dual_variable_stiffness/<timestamp>/`.

### Usage

```bash
# Always source first, then launch with logger and live_plot enabled

# Single gravity comp
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash
ros2 launch omx_dual_bringup single_robot_hardware.launch.py \
  port:=/dev/ttyUSB0 enable_logger:=true enable_live_plot:=true start_rviz:=false

# Dual gravity comp
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  enable_logger:=true enable_live_plot:=true start_rviz:=false

# Dual variable stiffness
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash
ros2 launch omx_variable_stiffness_controller dual_hardware_variable_stiffness.launch.py \
  enable_logger:=true enable_live_plot:=true start_rviz:=false
```

### Status

All 3 hardware modes built and wired. Hardware testing pending (robot not connected).

## Latest (2026-03-10 — Gazebo live plot tests)

### Completed Tests

All 4 Gazebo-based launch modes tested with `enable_live_plot:=true` and the new combined single-figure subplot layout:

1. **single_robot_test (GC single Gazebo)** — ✅ PASS
  - Package: `omx_dual_bringup`
  - Live plot subscribed to `/omx/joint_states`, ran stable 30+ seconds, no crash/segfault
  - Tested earlier in session

2. **dual_gazebo_gravity_comp (GC dual Gazebo)** — ✅ PASS (live plot works)
  - Package: `omx_dual_bringup`
  - Live plot subscribed to both `/robot1/joint_states` and `/robot2/joint_states`, no crash
  - Robot2 has pre-existing controller activation failure (not caused by our changes)
  - Tested earlier in session

3. **gazebo_variable_stiffness (VS single Gazebo)** — ✅ PASS
  - Package: `omx_variable_stiffness_controller`
  - Live plot subscribed to `/omx/variable_stiffness_controller/*`, ran stable 3+ minutes
  - 210MB RSS, 46% CPU, no crash/segfault/traceback
  - Uses combined single-figure layout with 13 subplots

4. **dual_gazebo_variable_stiffness (VS dual Gazebo)** — ✅ PASS (live plot works)
  - Package: `omx_variable_stiffness_controller`
  - Live plot subscribed to both `/robot1/robot1_variable_stiffness/*` and `/robot2/robot2_variable_stiffness/*`
  - Ran stable 2+ minutes, ~95MB RSS, no crash
  - Robot2 VS controller has pre-existing activation failure (same dual-robot issue)

### Key Changes Tested

- **Combined subplot layout**: `LiveFigureManager` now creates ONE figure with all groups as subplots (GC=3, VS=13) instead of separate figures per group
- All columns within each group are overlaid as colored lines on the same axes
- Font sizes auto-scale for screen fitting
- Dual-robot differentiation: solid lines for R1, dashed for R2

### Known Pre-existing Issues

- Robot2 controller activation fails in both dual GC and dual VS Gazebo modes (gazebo_ros2_control namespace/model isolation issue, not related to live plot changes)

## Latest (2026-03-07)

- **Dual Gazebo + Dual Hardware variable stiffness — zero errors (commit `4a99c24`)**:
  - Patched `gazebo_ros2_control` v0.4.10 (vendored workspace overlay) — removed `__ns:=` from global rcl args that poisoned multi-robot plugin instances.
  - Xacro plugin name parameterized via `$(arg prefix)` for unique per-robot Gazebo plugin instances.
  - Raised `STALE_THRESHOLD` from 50→100 cycles to eliminate false bus-dead warnings during dual-USB startup.
  - Stiffness loader: 3-attempt retry with 10 s timeout (was single attempt, 5 s).
  - **All launch paths verified zero errors**: Single Gazebo VS, Dual Gazebo VS, Single Hardware VS, Dual Hardware VS.
  - Dual Hardware VS promoted to ✅ **TESTED** — both robots complete full state machine cycle independently.
- **Dual variable stiffness tip_link alignment (commit `4824fb5`)**: Updated `tip_link` in both dual-hardware variable stiffness configs from `robot*_link5` to `robot*_end_effector_link`, matching the proven single-hardware configuration.

## Previous Update (2026-03-04)

- **Hardware YAML audit & fix**: Completely rewrote single-hardware `variable_stiffness_controller.yaml` — fixed broken bare namespace (was invisible to controller), flipped `use_sim_time` to `false`, set `torque_scale: 200.0` and `update_rate: 500` for real servos, added all safety params (homing, regularization, singularity, waypoint deviation, contact force filter). Removed ~230 lines of commented-out cruft.
- **Dual-hardware YAML sync**: Updated all 6 cross-namespace blocks in `robot1_variable_stiffness.yaml` and `robot2_variable_stiffness.yaml` with `state_interfaces`, `command_interfaces`, Gazebo-tested trajectory values, and every safety parameter.
- **Single-hardware launch fix**: Added `--set-state active` to both spawner nodes in `variable_stiffness_control.launch.py`.
- **EE contact force sensor** (`tools/ee_force_sensor.py`): Standalone ROS 2 node that subscribes to the controller's `~/contact_wrench` + `~/contact_valid`, applies deadzone filtering, and republishes as `~/ee_force` (Vector3Stamped) + `~/ee_force_magnitude` (Float64) at configurable rate. Designed as the input for a future force-feedback loop. Supports CSV logging.
- Previous: singularity-safe IK, joint-space homing/regularization, waypoint deviation (6 bugs fixed), Gazebo verification.

## Overview

This repository is a ROS 2 Humble workspace for OpenMANIPULATOR-X control.

- **Primary bringup package:** `omx_dual_bringup`
- **Gravity compensation controller:** `omx_gravity_comp_controller`
- **Variable stiffness controller:** `omx_variable_stiffness_controller` (Cartesian impedance with trajectory tracking)
- **Robot description + xacro:** `open_manipulator_x_description`
- **Hardware path (Dynamixel):** `dynamixel_sdk` + `dynamixel_hardware_interface`

You can run:
- **Dual robot** (hardware or Gazebo) with namespaces `robot1` / `robot2`
- **Single robot** (hardware or Gazebo) with namespace `omx`

## Features

### Implemented & Tested (Gravity Compensation)
✅ Proper namespace isolation (`robot1` and `robot2`)
✅ Independent YAML configurations for each robot
✅ Gravity compensation controller for each robot
✅ RViz2 visualization for both robots
✅ Automatic hardware detection
✅ **Dual hardware mode verified working** (Feb 2026)

### Implemented & Verified (Variable Stiffness)
✅ Variable Cartesian Impedance Controller with time-varying stiffness profiles
✅ Hardware safety limits (stiffness ≤65 N/m, damping ≤10 Ns/m software cap; recommend ≤3 Ns/m on real servos)
✅ Manipulability metrics publishing (singular values, condition number, σ_min)
✅ Support for both hardware and Gazebo simulation — all 4 modes verified (single/dual × sim/hw, Mar 2026)
✅ Dual Gazebo with patched gazebo_ros2_control overlay (zero-config, built by colcon)

### Awaiting Testing (see Planned Testing Roadmap below)
🔧 **T0 — Data Logger & Plotter**: Logger captures 55+ columns; plotter script needed for publication-quality figures
🔧 **T1 — Runtime Waypoint Deviation**: Gazebo-tested (offset + absolute, cosine blend); hardware test pending, dual-robot tool support needed
🔧 **T2 — EE Contact Force Sensing**: Deflection-based estimate implemented; calibration, filtering, and latency characterization pending

### Runtime Waypoint Deviation (Gazebo-tested, hardware pending — see T1)

- The controller supports runtime waypoint deviation: an external node publishes a
  `PoseStamped` to `~/waypoint_command` (offset or absolute) and the controller
  cosine-blends to the target over `waypoint_blend_duration` seconds, then
  automatically returns to the normal trajectory.
- When the EE deviates from the planned position by more than
  `deviation_publish_threshold` (default 0.01 m) the controller publishes the
  actual joint state on `~/deviated_waypoint` (rate-limited to ~2 Hz).
- **Status: ✅ Gazebo-tested (Mar 2026)** — offset and absolute waypoints
  verified; blend activates, completes, and deactivates correctly.
- **Pending (T1):** Hardware verification, dual-robot namespace support in tools,
  automated test orchestrator, orientation waypoint support. See **T1** in the
  Planned Testing Roadmap below.

### EE Contact Force Sensing (uncalibrated — see T2)

- The controller publishes a deflection-based contact force estimate on
  `~/contact_wrench` (WrenchStamped) and a validity flag on `~/contact_valid`
  (Bool, false during singularity escape).
- Force estimate: $\hat{F} = K \cdot (x_{\text{desired}} - x_{\text{actual}})$ — uses the current Cartesian stiffness matrix.
- `tools/ee_force_sensor.py` subscribes to these and republishes:
  - `~/ee_force` (Vector3Stamped) — filtered xyz force in root frame
  - `~/ee_force_magnitude` (Float64) — scalar for easy thresholding
- Supports configurable namespace/controller, publish rate, deadzone, and CSV logging.
- **Pending (T2):** Bias/tare calibration, low-pass filter, gravity offset
  correction, known-mass validation, Jacobian sensitivity characterization
  (error vs σ_min), feedback latency measurement. See **T2** in the Planned
  Testing Roadmap below.

### Safety Features (Variable Stiffness Controller)
✅ **Pre-trajectory IK Validation**: Validates entire trajectory via position-only Jacobian pseudo-inverse IK (backward-walk seeding) before execution
✅ **Singularity Detection**: Uses σ_min (minimum singular value of EE Jacobian) for robust singularity measure
✅ **Joint Limits Enforcement**: Checks URDF joint limits at all waypoints
✅ **Damped Least Squares (DLS)**: Adaptive damping near singularities prevents torque spikes
✅ **Manipulability Threshold**: Rejects trajectories that pass through low-manipulability regions
✅ **Joint-space Homing**: PD homing bypasses Cartesian impedance to prevent arm flips through singular regions
✅ **Joint-space Regularization**: Keeps arm near planned configuration during compliant trajectory execution

## Planned Testing & Tooling Roadmap

The following features require structured testing before publication. Each item lists the current state, gaps, and a concrete test plan.

### T0: Data Logger Sufficiency & Publication-Quality Plotting

**Goal:** Verify that `logger.py` captures every variable needed for analysis, with correct units, and produce a standalone plotter that generates publication-ready figures.

**Current state:** Logger (`scripts/logger.py`) subscribes to 14 controller topics and writes two CSVs per run:
- **Snapshot CSV** (~55 columns at configurable rate, default 100 Hz): timestamps, actual/desired EE pose (m, quaternion), EE velocity (m/s, rad/s), joint velocities (rad/s), joint torques (Nm), Cartesian stiffness (N/m) and damping (Ns/m), manipulability metrics (σ_min, condition number, Yoshikawa index), contact wrench (N, Nm), waypoint-active flag, and optional full Jacobian (24 entries).
- **Events CSV**: one row per `deviated_waypoint` JointState message (joint positions, velocities, efforts).

**Gaps identified:**
| Gap | Impact | Fix |
|-----|--------|-----|
| CSV headers have no units row | Reader must consult this doc to interpret columns | Add a `# units:` comment row as the second line of every CSV |
| No controller state column (HOMING, MOVE_FORWARD, etc.) | Cannot segment data by phase without manual timestamp alignment | Publish state enum on a new `~/controller_state` topic (String) and log it |
| No plotter script exists | No way to generate figures from logged data | Create `tools/plot_log.py` |
| No rotational stiffness/damping units label | `Krx/Kry/Krz` columns ambiguous | Document: Nm/rad for rotational stiffness, Nms/rad for rotational damping |
| Dead code at bottom of logger.py | Maintenance burden | Remove commented-out old implementation |

**Test plan:**
1. Run single Gazebo VS for 2 full trajectory cycles with `enable_logger:=true`
2. Inspect CSV: confirm every column has data, no NaN gaps, timestamps monotonically increasing
3. Build `tools/plot_log.py` (matplotlib) producing the following publication-quality plots:
   - **EE trajectory** (3D path: actual vs desired, with start/end markers)
   - **EE position tracking error** vs time (x, y, z subplots, units: mm)
   - **Stiffness profile** vs trajectory progress (Kx, Ky, Kz)
   - **Damping profile** vs trajectory progress (Dx, Dy, Dz)
   - **Joint torques** vs time (τ₁–τ₄, units: Nm)
   - **Manipulability** (σ_min vs time, with DLS-active threshold line)
   - **Contact force magnitude** vs time (N)
   - **Controller state** vs time (color-coded background bands)
4. Plot style: bold serif font (12 pt ticks, 14 pt labels, 16 pt titles), 300 DPI PNG export, LaTeX-compatible labels, gridlines, and legend outside plot area
5. Repeat on hardware (single + dual) to confirm identical column structure
6. Log one dual-hardware run and generate comparative robot1 vs robot2 overlay plots

**Acceptance criteria:** A single command `python3 tools/plot_log.py /tmp/variable_stiffness_logs/` produces all plots from any logged session, with no manual column mapping.

## Tools: Plotter

Added `tools/plot_logs.py`: a standalone, publication-quality plotter for the logger CSVs.
- Run `python3 tools/plot_logs.py --log-dir /tmp/variable_stiffness_logs` to list and plot runs.
- Built-in tests: `python3 tools/plot_logs.py --test` (skips data-dependent tests if `pandas/numpy/matplotlib` are not installed).
- The script detects single/dual runs, supports phase-space comparisons and baseline overlays, and writes 300 DPI PNGs when `--save-dir` is provided.

Notes:
- Default log directory can be overridden with the `OMX_LOG_DIR` environment variable or `--log-dir` flag.

### T1: Waypoint Live Deviation (Full Test Loop)

**Goal:** Validate the runtime waypoint deviation feature end-to-end — from publishing a waypoint command to observing the EE physically deviate and return — in both simulation and hardware.

**Current state:**
- Controller implements `~/waypoint_command` (PoseStamped) subscription with offset and absolute modes, cosine-blend transition over `waypoint_blend_duration` seconds, and automatic return to the planned trajectory.
- `~/waypoint_active` (Bool) published while blend is in progress.
- `~/deviated_waypoint` (JointState) published when EE deviates > `deviation_publish_threshold` (default 0.01 m).
- `tools/publish_waypoint.py`: one-shot publisher, hardcoded `/omx` namespace, no orientation control.
- `tools/deviated_listener.py`: captures first `deviated_waypoint` message to CSV, then exits.
- **Status:** Gazebo-tested for offset/absolute blend activation and completion (Mar 2026). Not tested on hardware. Dual-robot namespace support missing from helper tools.

**Gaps identified:**
| Gap | Impact | Fix |
|-----|--------|-----|
| `publish_waypoint.py` hardcodes `/omx` | Cannot test on dual-robot namespaces (`/robot1`, `/robot2`) | Add `--namespace` CLI arg |
| No orientation waypoint support | Cannot test orientation deviation | Add `--rpy` CLI arg for roll/pitch/yaw |
| `deviated_listener.py` is single-shot | Cannot observe return-to-trajectory phase | Rewrite as continuous listener with timeout |
| No automated test script | Manual multi-terminal coordination required | Create `tools/test_waypoint_deviation.py` orchestrator |
| Not tested on hardware | Unknown whether EE physically deviates within torque limits | Hardware test required |
| No dual-robot deviation test | Unknown interaction between concurrent waypoints | Test both robots deviating simultaneously |

**Test plan — Simulation (single robot):**
1. Launch single Gazebo VS (`gui:=false, enable_logger:=true`)
2. Wait for MOVE_FORWARD state (trajectory active)
3. Publish offset waypoint: `+0.03 m` in x (toward extension)
4. Verify via logger CSV: `waypoint_active` transitions `0→1→0`, EE x overshoots by ~30 mm then returns
5. Publish absolute waypoint: `[0.20, 0.03, 0.18]` (off-trajectory)
6. Verify EE reaches target within 2× blend duration, then returns to trajectory
7. Verify `deviated_waypoint` messages fire during deviation and stop after return
8. Verify no singularity escape triggered (σ_min stays above threshold)

**Test plan — Simulation (dual robot):**
1. Launch dual Gazebo VS
2. Send offset waypoint to robot1 only → verify robot2 unaffected
3. Send simultaneous waypoints to both robots → verify independent deviation/return

**Test plan — Hardware (single robot):**
1. Launch single hardware VS with conservative stiffness (Kx=Ky=Kz=20 N/m)
2. During MOVE_FORWARD, publish small offset: `+0.02 m` in x
3. Physically observe EE shift forward, then return
4. Monitor `[CLAMP]` logs — torque must not saturate during deviation
5. Verify σ_min does not drop below escape threshold

**Test plan — Hardware (dual robot):**
1. Launch dual hardware VS
2. Deviate robot1 while robot2 runs normal trajectory
3. Verify namespace isolation — robot2 must be unaffected
4. Simultaneous deviation on both robots

**Acceptance criteria:**
- Offset and absolute waypoints work in all 4 modes (single/dual × sim/hw)
- Cosine blend activates and deactivates cleanly (no stuck `waypoint_active`)
- EE returns to planned trajectory within `waypoint_blend_duration + 1 s`
- No torque saturation or singularity escape during moderate deviations (≤ 0.05 m)
- Logger CSV captures the full deviation event with correct `waypoint_active` column

### T2: EE Contact Force Sensing — Calibration & Latency

**Goal:** Validate the deflection-based contact force estimate against known loads, characterize its accuracy as a function of Jacobian conditioning (σ_min), and measure feedback latency.

**Current state:**
- Controller computes contact force as $\hat{F} = K \cdot (x_{\text{desired}} - x_{\text{actual}})$ — a deflection-based estimate using the current Cartesian stiffness matrix.
- Published on `~/contact_wrench` (WrenchStamped) with a validity flag on `~/contact_valid` (suppressed during singularity escape).
- `tools/ee_force_sensor.py` applies deadzone filtering (default 0.1 N) and republishes as `~/ee_force` (Vector3Stamped) + `~/ee_force_magnitude` (Float64).
- CSV logging supported via `--csv` flag.
- **Not calibrated.** No bias removal, no low-pass filter, no gravity offset correction, no comparison to ground truth.

**Known limitations (design constraints):**
- **Jacobian sensitivity:** The force estimate depends on the Cartesian stiffness at the current configuration. Near singularities (low σ_min), the stiffness effectively `sees` deflections differently along different axes. The estimate degrades as σ_min → 0 even though `contact_valid` only suppresses during escape.
- **No direct F/T sensor:** This is purely model-based (spring-deflection). External perturbations that do not cause measurable EE deflection (e.g., forces along the stiff axis) will be underestimated.
- **Gravity coupling:** During vertical motion, gravity-induced sag creates a standing offset in the z-force estimate. This must be tared per-configuration.
- **Latency:** The estimate is computed inside the 500 Hz control loop, but `ee_force_sensor.py` republishes at a configurable rate (default 50 Hz). The total pipeline latency (sensor → controller → topic → ee_force_sensor → subscriber) determines whether force-feedback is viable.

**Gaps identified:**
| Gap | Impact | Fix |
|-----|--------|-----|
| No bias/tare calibration | Standing force reading when no contact exists | Add `--tare` mode to `ee_force_sensor.py`: average N samples at rest, subtract as offset |
| No low-pass filter | High-frequency joint noise creates force jitter | Add configurable 1st-order Butterworth or EMA filter (cutoff ~10–20 Hz) |
| No gravity offset correction | z-force offset varies with arm pose | Tare per-pose or add gravity model subtraction |
| No ground-truth validation | Unknown accuracy (N, %) | Use known masses at EE to calibrate |
| No σ_min-dependent error characterization | Unknown degradation near singularities | Sweep σ_min range and measure force error |
| Latency not measured | Unknown if fast enough for force-feedback control | Timestamp comparison: controller publish time vs ee_force_sensor receive time |
| `ee_force_sensor.py` drops torque components | Cannot detect rotational contact | Add torque passthrough or separate topic |

**Test plan — Static calibration (hardware):**
1. Launch single hardware VS in WAIT_AT_START state (arm at known rest pose)
2. Record 5 s of `~/contact_wrench` with no contact → compute bias (mean fx, fy, fz)
3. Attach known masses to EE gripper: 50 g, 100 g, 200 g
4. For each mass, record 5 s → compute mean fz, compare to expected (mass × 9.81)
5. Compute absolute error (N) and relative error (%) per mass point
6. Repeat at 3 different arm configurations (extended, mid-range, near-singular)
7. Tabulate: configuration (joint angles), σ_min, expected force, measured force, error

**Test plan — Dynamic response (hardware):**
1. During MOVE_FORWARD, tap EE with finger → observe force spike
2. Record `~/ee_force_magnitude` at 50 Hz and controller's `~/contact_wrench` at logger rate
3. Measure rise time (10%→90% of peak) and settling time
4. Measure latency: compare `contact_wrench.header.stamp` with `ee_force.header.stamp`

**Test plan — Jacobian sensitivity (Gazebo):**
1. Design a trajectory that sweeps σ_min from 0.08 (healthy) down to 0.03 (near threshold)
2. Apply a constant simulated force (Gazebo `apply_wrench` plugin) at the EE
3. Record estimated force vs applied force as a function of σ_min
4. Plot error vs σ_min → characterize the degradation curve

**Test plan — Feedback latency (hardware):**
1. Subscribe to both `~/contact_wrench` (500 Hz from controller) and `~/ee_force` (50 Hz from sensor node)
2. Introduce a step perturbation (place/remove mass)
3. Measure time difference between force step appearance on `contact_wrench` vs `ee_force`
4. Target: total latency < 40 ms for closed-loop force feedback at 25 Hz

**Acceptance criteria:**
- Static accuracy: < 15% relative error for known loads > 0.5 N at σ_min > 0.05
- Bias drift: < 0.2 N over 60 s with no contact
- Latency: `ee_force_sensor.py` publishes within 25 ms of controller's `contact_wrench` timestamp
- Force estimate validity flag correctly suppresses during singularity escape
- Documented calibration table with repeatability bounds (±1 SD across 3 trials per mass)

## Directory Structure

```
omx_dual_bringup/
├── config/
│   ├── robot1_gravity_comp.yaml    # Robot 1 controller config
│   └── robot2_gravity_comp.yaml    # Robot 2 controller config
├── launch/
│   ├── auto_dual_gravity_comp.launch.py      # Auto-detect mode
│   ├── dual_gazebo_gravity_comp.launch.py    # Gazebo simulation
│   └── dual_hardware_gravity_comp.launch.py  # Hardware control
└── rviz/
    └── dual_robots.rviz                 # RViz configuration

omx_variable_stiffness_controller/
├── config/
│   ├── variable_stiffness_controller.yaml    # Single robot config
│   ├── robot1_variable_stiffness.yaml        # Robot 1 impedance config
│   ├── robot2_variable_stiffness.yaml        # Robot 2 impedance config
│   └── *.csv                                 # Stiffness profile files
├── launch/
│   ├── variable_stiffness_control.launch.py  # Single robot launch
│   ├── dual_hardware_variable_stiffness.launch.py
│   └── dual_gazebo_variable_stiffness.launch.py
└── scripts/
    ├── load_stiffness.py             # Load CSV profiles at runtime
    └── logger.py                      # Log controller state
```

## Setup Recipes (4 Modes)

Each mode below lists:
- **Build packages/files**: which packages you need to build (and where their build files live)
- **Key dependencies**: what needs to be installed for that mode
- **Dependency locations**: which workspace folders (and/or upstream repos) you need for that mode
- **Launch file**: what to run
- **Commands**: a minimal command list to bring it up

> Note: All commands assume you are in this repo and using ROS 2 Humble.

### Minimal Initial Setup (after cloning)

Run this once after cloning (then pick one of the 4 modes below).

```bash
# From the repo root
cd /workspaces/omx_ros2/ws

# ROS environment
source /opt/ros/humble/setup.bash

# Tools + rosdep (skip if you already have these)
apt update
apt install -y python3-colcon-common-extensions python3-rosdep

# rosdep is the easiest way to pull the correct ROS/Ubuntu packages
rosdep init 2>/dev/null || true
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true
```

Notes:
- If you are using the devcontainer, most dependencies are already installed during container setup.
- Hardware modes need USB/serial access (e.g. `/dev/ttyUSB*` or `/dev/serial/by-id/*`).

### 1) Dual Hardware Setup (2 real robots) ✅ TESTED

> ✅ **Status: VERIFIED** — Tested on two physical OpenMANIPULATOR-X robots, Feb 2026.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Configs used: `ws/src/omx_dual_bringup/config/robot1_gravity_comp.yaml`, `ws/src/omx_dual_bringup/config/robot2_gravity_comp.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - ROS 2 control stack: `ros2_control`, `ros2_controllers`, `controller_manager`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
  - Serial access from Python tools/scripts: `python3-serial`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
    - `ws/src/dynamixel_sdk/`
    - `ws/src/dynamixel_hardware_interface/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
    - DynamixelSDK (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git dynamixel_sdk`
    - Dynamixel HW interface (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/dual_hardware_gravity_comp.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install OS/ROS dependencies (recommended for fresh machines)
apt update
apt install -y python3-colcon-common-extensions python3-rosdep python3-serial
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (auto-detects ports, or override)
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  robot1_port:=/dev/ttyUSB0 robot2_port:=/dev/ttyUSB1 start_rviz:=true

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
ros2 topic echo /robot1/joint_states
```

**Tuning Notes:**
- The `torque_scale` parameter in `robot*_gravity_comp.yaml` controls compensation strength (default: 200.0)
- If arms feel weak/droopy, increase `torque_scale` (try 250-400)
- If arms feel stiff or overshoot, decrease `torque_scale`
- Each robot can be tuned independently via its config file

### 2) Dual Simulation Setup (2 robots in Gazebo) -✅ TESTED

> ⚠️ **Status: UNTESTED** — Builds and launches, but not verified in simulation.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Configs used: `ws/src/omx_dual_bringup/config/robot1_gravity_comp.yaml`, `ws/src/omx_dual_bringup/config/robot2_gravity_comp.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`, `gazebo_ros_pkgs`
  - ROS 2 control stack: `controller_manager`, `ros2controlcli`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/dual_gazebo_gravity_comp.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install OS/ROS dependencies (recommended for fresh machines)
apt update
apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (single Gazebo server by default)
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=single start_rviz:=true

## Verifying Variable Stiffness in Gazebo
In a full Gazebo simulation (plugin present) the controller manager will
start automatically and, after a lengthy initialization period (~30–40 s
observed in the container), it will load the
`variable_stiffness_controller` from the YAML parameters.  Service calls
may be unresponsive during this time, so the easiest way to know the
stack is up is to watch for `/omx/joint_states` or any of the controller’s
output topics (`/omx/variable_stiffness_controller/*`).  Example workflow:

```bash
# start Gazebo (no GUI) and wait in background
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch omx_variable_stiffness_controller gazebo_variable_stiffness.launch.py \
    gui:=false launch_gazebo:=true spawn_delay:=0.1 controller_delay:=3.0 \
    stiffness_loader_delay:=4.0 enable_logger:=false &

sleep 60  # give controller_manager time to finish loading
ros2 topic echo /omx/joint_states --once          # should receive a message
ros2 control list_controllers -c /omx/controller_manager

# controllers are automatically activated by the spawners in the launch
# file.  If you need to reload a controller manually use the spawner
# instead of the low‑level `ros2 control` command:
#
# ```bash
# ros2 run controller_manager spawner joint_state_broadcaster \
#     --controller-manager /omx/controller_manager \
#     --param-file $(ros2 pkg prefix omx_variable_stiffness_controller)/share/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml \
#     --activate-as-group
# ```
#
# (or substitute `variable_stiffness_controller` for whatever controller
# you’re interested in.) The spawner handles the full lifecycle transition
# and retries automatically; the old `ros2 control load_controller` call
# only configures and often left controllers stuck in the `inactive`
# state.
```

Because startup is slow the headless CI tests avoid calling services and
simply wait for joint states instead; see the `test_simulation_launch.py`
files in the controller packages.

# Optional: full isolation using two Gazebo servers
# ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=dual

### Recommended GUI launch (what to use when you need the Gazebo client)

If you require the Gazebo GUI (gzclient) for visual debugging or interactive
tests, launch with the GUI flag and a small controller spawn delay so the
`spawn_entity` and controller_manager sequencing finish reliably. Note
that the container needs host X access (e.g. `xhost +local:docker`) and the
devcontainer must include Qt/XCB libs (this repo's devcontainer setup
attempts to install them). Example:

```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch omx_variable_stiffness_controller gazebo_variable_stiffness.launch.py \
  gui:=true launch_gazebo:=true spawn_delay:=0.1 controller_delay:=3.0 \
  stiffness_loader_delay:=4.0 enable_logger:=true
```

Why this differs from the headless examples: `gzclient` requires a working
X server and extra GUI libraries not present in CI/devcontainer images by
default. The listed command is intentionally explicit about spawn/controller
delays to avoid race conditions observed when the Gazebo factory or the
controller manager is not yet ready. Always prefer the README-recommended
command; if you need a different procedure explain why and update this file.

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager

## Variable Stiffness Controller Integration (Gazebo + ros2_control)

### Problem

The custom controller `omx_variable_stiffness_controller/OmxVariableStiffnessController`:

- Loaded successfully
- Activated successfully
- Claimed effort interfaces
- Produced non-zero torques

But:

- Robot did not move
- Controller parameters were not applied
- Manipulability metric returned `inf`
- Desired pose did not match configured trajectory
- `/omx/controller_manager` only showed `.type` parameter

Root cause:
Controller-specific parameters were **not being loaded into the controller_manager namespace**, so the controller ran with default/internal values.

### Key Fixes Applied

#### 1️⃣ Verified Controller State

Confirmed:

```bash
ros2 control list_controllers -c /omx/controller_manager
```

Result:

```
variable_stiffness_controller ... active
```

Verified effort ownership:

```bash
ros2 control list_hardware_interfaces -c /omx/controller_manager
```

Result:

```
joint1-4/effort [claimed]
```

Confirmed torque output:

```bash
ros2 topic echo /omx/variable_stiffness_controller/torque_values
```

Non-zero torque values observed.

#### 2️⃣ Diagnosed Parameter Loading Failure

Checked controller parameters:

```bash
ros2 param get /omx/controller_manager variable_stiffness_controller.start_position
```

Output:

```
Parameter not set.
```

Only parameter present:

```
variable_stiffness_controller.type
```

Conclusion:
YAML controller parameters were not being injected into controller_manager.

#### 3️⃣ Fixed Parameter Injection

Applied **controller_manager-prefixed parameter injection**:

```yaml
/omx/controller_manager:
  ros__parameters:
    variable_stiffness_controller.start_position: [...]
    variable_stiffness_controller.end_position: [...]
    variable_stiffness_controller.target_orientation: [...]
    ...
```

This guarantees parameters exist inside:

```
/omx/controller_manager
```

Which is where ros2_control controllers read them from.

#### 4️⃣ Resolved Singularity Lock

Observed:

```
manipulability_metrics → inf
```

Cause:
Jacobian σ_min ≈ 0 → singular configuration.

Temporary stability fix applied:

```yaml
variable_stiffness_controller.min_manipulability_threshold: 0.0
variable_stiffness_controller.dls_damping_factor: 0.2
```

Also ensured trajectory avoids y=0 plane.

### Final Working State

- Gazebo running
- robot_state_publisher available
- controller_manager services active
- variable_stiffness_controller:

  - Loaded
  - Configured
  - Active
  - Effort interfaces claimed
  - Publishing torque_values
  - Publishing pose, Jacobian, stiffness state
  - Running state machine

```

### 3) Single Simulation Setup (1 robot in Gazebo) ⚠️ Untested

> ⚠️ **Status: UNTESTED** — Builds and launches, but not verified in simulation.
> Add `launch_gazebo:=false` for a headless, gazebo‑free mode used by CI tests; this will start a generic "fake" hardware backend instead of the Gazebo plugin and requires longer controller delays.
> 
> **Important:** the launch recipes automatically skip the controller `spawner`
> nodes when `launch_gazebo` is false because the controllers are already
> declared in the YAML file.  The spawners tend to block the
> `/controller_manager` services in fake‑hardware/headless runs (see test
> logs), which is why the CI tests were failing earlier.  If you ever need
> to bring up a controller manually in this mode, use the `spawner` node
> (see earlier examples) rather than the raw `ros2 control load_controller`
> CLI.  The spawner performs the full lifecycle transition and retries
> internally; the direct CLI invocation only configures the controller and
> frequently leaves it stuck inactive.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Config used: `ws/src/omx_dual_bringup/config/single_robot_gravity_comp.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`, `gazebo_ros_pkgs`
  - ROS 2 control stack: `controller_manager`, `ros2controlcli`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/single_robot_test.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install OS/ROS dependencies (recommended for fresh machines)
apt update
apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch
ros2 launch omx_dual_bringup single_robot_test.launch.py

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/joint_states
```

### 4) Single Hardware Setup (1 real robot) ✅ TESTED

> ✅ **Status: VERIFIED** — Tested on physical hardware, Feb 2026.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Config used: `ws/src/omx_dual_bringup/config/single_robot_hardware_gravity_comp.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - ROS 2 control stack: `ros2_control`, `ros2_controllers`, `controller_manager`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
    - `ws/src/dynamixel_sdk/`
    - `ws/src/dynamixel_hardware_interface/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
    - DynamixelSDK (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git dynamixel_sdk`
    - Dynamixel HW interface (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/single_robot_hardware.launch.py`
**Commands**
```bash
# From the repo devcontainer or a host with ROS 2 Humble sourced
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# NOTE: In the devcontainer most OS/ROS dependencies are installed
# during container creation (no sudo/rosdep required). If you are
# running on a fresh host, follow the "Minimal Initial Setup" section
# earlier in this README to install deps once on the host.

# Build (no sudo required inside the devcontainer)
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (auto-detects port, or override). Make sure the serial device
# is exposed to the container or available to your user on the host.
ros2 launch omx_dual_bringup single_robot_hardware.launch.py port:=/dev/ttyUSB0 start_rviz:=false

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/joint_states
```

### 5) Variable Cartesian Impedance Control (Single Robot) ✅ TESTED

> ✅ **Status: VERIFIED** — Tested in simulation mode (Gazebo), March 2026. Ready for physical hardware.

The variable stiffness controller provides Cartesian impedance control with:
- Time-varying stiffness/damping profiles along trajectories
- **Pre-trajectory IK validation** with singularity detection
- **Damped Least Squares (DLS)** for safe operation near singularities
- Joint-space trajectory interpolation for guaranteed safety

#### Key Safety Features
| Feature | Description |
|---------|-------------|
| **Trajectory Validation** | IK solved for 101 waypoints before execution; rejects unsafe paths |
| **σ_min Threshold** | Uses minimum singular value (default: 0.02) as singularity measure |
| **Joint Limits** | Loads limits from URDF; rejects IK solutions outside bounds |
| **DLS Damping** | Adaptive λ² = λ₀² × (1 - σ_min/threshold)² near singularities |
| **Stiffness Limits** | 65 N/m translational, 10 Ns/m damping (software cap; ≤3 Ns/m recommended on hardware) |
| **x-axis Clamp** | `start_position` and `end_position` x clamped to ≥ 0.16 m at startup |
| **Stale-data Guard** | Zeros torques if joint positions frozen for ≥50 cycles (Dynamixel bus crash protection) |
| **Gravity-preserving Clamp** | Scales PD torque component only; gravity compensation always applied at full strength |
| **Joint-limit Barrier** | Soft repulsive torque (K=50 N·m/rad) applied 0.05 rad before URDF joint limits |
| **Torque Ramp** | 2 s linear ramp from zero on hardware activation (prevents impulse at startup) |

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Configs: `ws/src/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml`
- **Key dependencies**
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
  - Same hardware dependencies as gravity comp mode
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/variable_stiffness_control.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch in SIMULATION MODE (for testing without robot)
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py \
  sim:=true enable_logger:=false

# Launches Gazebo with the variable stiffness controller; check logs for trajectory #validation and manipulability metrics.
source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/install/setup.bash && DISPLAY=:0 ros2 launch omx_variable_stiffness_controller gazebo_variable_stiffness.launch.py gui:=true launch_gazebo:=true spawn_delay:=1.0 controller_delay:=10.0 stiffness_loader_delay:=10.0 enable_logger:=true 2>&1 | tee /tmp/gz_launch.log


# Launch with REAL HARDWARE
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py \
  port:=/dev/ttyUSB0

ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py sim:=false enable_logger:=true

# Check trajectory validation passed
# Look for: "[SAFETY] Trajectory validation passed. Min manipulability=X.XXXX"

# Monitor manipulability metrics (published at 500Hz)
ros2 topic echo /omx/variable_stiffness_controller/manipulability_metrics --once
# Output: [cond_number, ee_x, ee_y, ee_z, σ1, σ2, σ3, σ4, ...]

# Monitor torque commands
ros2 topic echo /omx/variable_stiffness_controller/torque_values --once
```

### 6) Variable Cartesian Impedance Control (Dual Hardware) ✅ TESTED

> 🔧 **Status: Ready** — Code complete, awaiting physical robot connection for testing.

The variable stiffness controller provides Cartesian impedance control with time-varying stiffness/damping profiles along trajectories.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Configs: `ws/src/omx_variable_stiffness_controller/config/robot*_variable_stiffness.yaml`
- **Key dependencies**
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
  - Same hardware dependencies as gravity comp mode
- **Hardware Safety Limits** (enforced in software)
  - Max Cartesian stiffness: **65 N/m**
  - Max Cartesian damping: **10 Ns/m** (software cap; hardware recommendation ≤3 Ns/m for XM430 servos)
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/dual_hardware_variable_stiffness.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch dual arm variable stiffness
ros2 launch omx_variable_stiffness_controller dual_hardware_variable_stiffness.launch.py

# Monitor manipulability metrics
ros2 topic echo /robot1/robot1_variable_stiffness/manipulability
# Output: [λ1, λ2, λ3, condition_number, determinant]

# Load custom stiffness profile at runtime
ros2 run omx_variable_stiffness_controller load_stiffness.py \
  --file config/robot1_stiffness_profile.csv \
  --controller /robot1/robot1_variable_stiffness
```

**Stiffness Profile CSV Format:**
```csv
s,Kx,Ky,Kz,Dx,Dy,Dz
0.0,60.0,60.0,60.0,2.5,2.5,2.5
0.5,35.0,35.0,35.0,1.4,1.4,1.4
1.0,60.0,60.0,60.0,2.5,2.5,2.5
```
Where `s` is trajectory progress (0-1), K is stiffness (N/m), D is damping (Ns/m).

**Runtime Waypoint Commands:**

The controller supports publishing waypoint deviations at runtime via the `~/waypoint_command` topic (geometry_msgs/PoseStamped). The robot will smoothly blend towards the commanded position based on the current impedance settings.

```bash
# Publish an ABSOLUTE target position (world frame)
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'absolute'}, pose: {position: {x: 0.22, y: 0.0, z: 0.18}}}"

# Publish an OFFSET from current trajectory (relative deviation)
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'offset'}, pose: {position: {x: 0.02, y: 0.0, z: -0.03}}}"

# Queue multiple waypoints (they'll be traversed in order)
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'absolute'}, pose: {position: {x: 0.20, y: 0.05, z: 0.15}}}"
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'offset'}, pose: {position: {x: 0.0, y: -0.10, z: 0.0}}}"

# Check if waypoint tracking is active
ros2 topic echo /robot1/robot1_variable_stiffness/waypoint_active
```

- **frame_id = "offset" or "relative"**: Waypoint is treated as offset from current trajectory target
- **frame_id = "absolute" or empty**: Waypoint is absolute position in world frame
- **Queue behavior**: Multiple waypoints are queued and traversed sequentially
- **Compliance**: Tracking smoothness depends on current stiffness (lower = softer tracking)

**Data Logging:**

The `logger.py` script logs controller state to CSV files for post-experiment analysis. By default, logs are saved to `/tmp/variable_stiffness_logs/` with timestamped filenames.  To keep logs inside the workspace instead, set the output directory to `ws/logs/variable_stiffness_logs/` (relative to the repository root) or any other path via the `OMX_LOG_DIR` environment variable or the `--ros-args -p output_dir:=<path>` argument.

```bash
# Run the logger (in a separate terminal)
ros2 run omx_variable_stiffness_controller logger.py

# With custom output directory
ros2 run omx_variable_stiffness_controller logger.py --ros-args \
  -p output_dir:=/path/to/persistent/folder

# Include Jacobian values (disabled by default due to size)
ros2 run omx_variable_stiffness_controller logger.py --ros-args \
  -p log_jacobian:=true
```

**Logged CSV columns:**
| Category | Columns |
|----------|---------|
| Timestamp | `timestamp` |
| Actual pose | `actual_x`, `actual_y`, `actual_z`, `actual_qx`, `actual_qy`, `actual_qz`, `actual_qw` |
| Desired pose | `desired_x`, `desired_y`, `desired_z`, `desired_qx`, `desired_qy`, `desired_qz`, `desired_qw` |
| End effector position | `ee_x`, `ee_y`, `ee_z` |
| End effector orientation | `ee_roll`, `ee_pitch`, `ee_yaw` |
| End effector velocities | `ee_vx`, `ee_vy`, `ee_vz`, `ee_wx`, `ee_wy`, `ee_wz` |
| Joint velocities | `jv1`, `jv2`, `jv3`, `jv4` |
| Commanded torques | `tau1`, `tau2`, `tau3`, `tau4` |
| Stiffness (translational) | `Ktx`, `Kty`, `Ktz` |
| Stiffness (rotational) | `Krx`, `Kry`, `Krz` |
| Damping (translational) | `Dtx`, `Dty`, `Dtz` |
| Damping (rotational) | `Drx`, `Dry`, `Drz` |
| Jacobian (optional) | `J00`...`J53` (6x4 matrix elements) |

**Saving logs:** Logs in `/tmp/` are cleared on reboot. To preserve a run:
```bash
# Copy to a persistent location
cp /tmp/variable_stiffness_logs/variable_stiffness_log_*.csv ~/saved_logs/
```

### 6) Variable Cartesian Impedance Control (Dual Simulation) ✅ TESTED - March 7, 2026

> ⚠️ **Status: BUILD ONLY** — This controller compiles and has been tested in Gazebo simulation.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Configs: `ws/src/omx_variable_stiffness_controller/config/robot*_variable_stiffness.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/dual_gazebo_variable_stiffness.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install dependencies
apt update
apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  liborocos-kdl-dev ros-humble-kdl-parser libeigen3-dev

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch dual arm simulation
ros2 launch omx_variable_stiffness_controller dual_gazebo_variable_stiffness.launch.py

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 topic echo /robot1/robot1_variable_stiffness/manipulability
```

### 7) Variable Cartesian Impedance Control (Dual Hardware) ✅ TESTED - March 7, 2026

> ⚠️ **Status: BUILD ONLY** — This controller compiles and has been tested on actual hardware.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Config: `ws/src/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/variable_stiffness_control.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install dependencies
apt update
apt install -y liborocos-kdl-dev ros-humble-kdl-parser libeigen3-dev

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch single arm hardware (auto-detects port)
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py

# Or specify port explicitly
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py \
  port:=/dev/ttyUSB0
/workspaces/omx_ros2/tools/launch_with_build.sh --no-build -- omx_variable_stiffness_controller dual_hardware_variable_stiffness.launch.py start_rviz:=false enable_logger:=true
# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/variable_stiffness_controller/manipulability
```

### 8) Variable Cartesian Impedance Control (Single Simulation) ✅ TESTED - March 7, 2026

> ⚠️ **Status: BUILD ONLY** — This controller compiles but has NOT been tested in Gazebo simulation.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Config: `ws/src/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/variable_stiffness_control.launch.py` with `sim:=true`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install dependencies
apt update
apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  liborocos-kdl-dev ros-humble-kdl-parser libeigen3-dev

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch single arm simulation
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py sim:=true

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/variable_stiffness_controller/manipulability
```

## Quick Start

### 1. Build the Workspace

```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select omx_dual_bringup omx_gravity_comp_controller
source install/setup.bash
```

### 2. Launch with Auto-Detection (Recommended)

The auto-detection launch file will automatically detect if two robots are connected. If not, it will launch Gazebo simulation:

```bash
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py
```

### Launch wrapper (build + source)

To ensure the workspace is built and the install/setup is sourced before running a launch, use the provided wrapper script. This is useful in the devcontainer where environment setup can be required before launching nodes.

```bash
# Build (optional) then launch (example)
/workspaces/omx_ros2/tools/launch_with_build.sh --build -- omx_variable_stiffness_controller \
  dual_hardware_variable_stiffness.launch.py start_rviz:=false enable_logger:=false
```

The wrapper will run `colcon build` (when `--build` is given or if no `install/setup.bash` exists), source the workspace setup, and then exec `ros2 launch` with the provided arguments. It also temporarily relaxes `nounset` while sourcing setup files to avoid failures from third-party setup scripts.


**Launch Arguments:**
- `mode`: `auto` (default), `hw`, `sim`
- `start_rviz`: `true` (default) or `false`
- `robot1_port`: Serial port for robot 1 (optional, auto-detected)
- `robot2_port`: Serial port for robot 2 (optional, auto-detected)
  - `gazebo_mode`: `single` (default) or `dual` (two Gazebo servers)
  - `start_gzclient`: `true` (default) or `false`

**Examples:**
```bash
# Auto-detect (default)
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py

# Force simulation mode
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py mode:=sim

# Force hardware mode with custom ports
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py \
  mode:=hw \
  robot1_port:=/dev/ttyUSB0 \
  robot2_port:=/dev/ttyUSB1

# Without RViz
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py start_rviz:=false
```

### 3. Launch Gazebo Simulation (Manual)

```bash
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py
```

**Launch Arguments:**
- `gazebo_mode`: `single` (default) or `dual` (two Gazebo servers)
- `start_gzclient`: `true` (default) or `false`

**Launch Arguments:**
- `start_rviz`: `true` (default) or `false`
- `robot1_x`, `robot1_y`: Position of robot 1 (default: 0.0, 0.5)
- `robot2_x`, `robot2_y`: Position of robot 2 (default: 0.0, -0.5)

**Example:**
```bash
# Position robots at custom locations
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py \
  robot1_x:=0.5 robot1_y:=0.0 \
  robot2_x:=-0.5 robot2_y:=0.0
```

### 4. Launch with Hardware (Manual)

Make sure two Open Manipulator X robots are connected via USB/serial:

```bash
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py
```

**Launch Arguments:**
- `start_rviz`: `true` (default) or `false`
- `robot1_port`: Serial port for robot 1 (auto-detected)
- `robot2_port`: Serial port for robot 2 (auto-detected)

**Example:**
```bash
# Specify custom serial ports
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  robot1_port:=/dev/ttyUSB0 \
  robot2_port:=/dev/ttyUSB1
```

## Namespace Isolation

Both robots operate in separate namespaces with proper isolation:

### Robot 1 Topics
- `/robot1/joint_states`
- `/robot1/robot_description`
- `/robot1/controller_manager/*`
- `/robot1/gravity_comp_controller/*`

### Robot 2 Topics
- `/robot2/joint_states`
- `/robot2/robot_description`
- `/robot2/controller_manager/*`
- `/robot2/gravity_comp_controller/*`

## Controller Management

### List Controllers

```bash
# Robot 1
ros2 control list_controllers -c /robot1/controller_manager

# Robot 2
ros2 control list_controllers -c /robot2/controller_manager
```

### Load/Unload Controllers

```bash
# Load & activate gravity compensation controller for robot 1 using the
# spawner helper (performs load/configure/activate in one step):
ros2 run controller_manager spawner robot1/gravity_comp_controller \
    --controller-manager /robot1/controller_manager \
    --param-file $(ros2 pkg prefix omx_gravity_comp_controller)/share/omx_gravity_comp_controller/config/gravity_comp_controller.yaml \
    --activate-as-group

# (or substitute `variable_stiffness_controller` for whatever controller
# you’re interested in.) The spawner handles the full lifecycle transition
# and retries automatically; the old `ros2 control load_controller` call
# only configures and often left controllers stuck in the `inactive`
# state.
```
## Viewing Joint States

```bash
# Robot 1 joint states
ros2 topic echo /robot1/joint_states

# Robot 2 joint states
ros2 topic echo /robot2/joint_states
```

## Tests

Run the verification script after launching Gazebo + RViz to confirm controllers and nonzero effort:

```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# In another terminal, with sim already running
./src/omx_dual_bringup/scripts/verify_dual_setup.sh
```

The script checks:
- `gravity_comp_controller` is active for both robots
- Effort command interfaces are available
- `robot_description` is present
- `/robot*/joint_states` has nonzero effort values
- RViz2 and Gazebo nodes are running (best-effort)

## Configuration Details

### Gravity Compensation Parameters

Each robot has independent gravity compensation configuration:

**Gazebo Simulation:**
- Update rate: 500 Hz
- Torque limits: 2.0 - 4.0 Nm (per joint)
- Damping: 0.02 - 0.05 (lower for simulation)

**Hardware:**
- Update rate: 500 Hz
- Current limits: 0.4 - 0.8 A (Dynamixel safety limits)
- Damping: 0.05 - 0.10 (higher for real hardware)

### YAML Configuration Example

```yaml
robot1/controller_manager:
  ros__parameters:
    update_rate: 500
    use_sim_time: true

    robot1/joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    robot1/gravity_comp_controller:
      type: omx_gravity_comp_controller/OmxGravityCompController

robot1/gravity_comp_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4]
    root_link: robot1_world
    tip_link: end_effector_link
    torque_scale: 1.0
    robot_description_node: /robot1/controller_manager
```

## Troubleshooting

### Serial Port Detection Issues

If hardware is not detected:

```bash
# List available serial ports
ls -la /dev/ttyUSB* /dev/ttyACM*
ls -la /dev/serial/by-id/

# Check port permissions
# If your user is not in the dialout group, add it via your system admin.
#   usermod -a -G dialout $USER
# Then log out/in or restart your session.

# Manually specify ports
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  robot1_port:=/dev/ttyUSB0 \
  robot2_port:=/dev/ttyUSB1
```

### Controllers Not Loading

```bash
# Check controller manager status
ros2 control list_controllers -c /robot1/controller_manager

# Manually load controllers with the spawner utility
ros2 run controller_manager spawner joint_state_broadcaster \
    --controller-manager /robot1/controller_manager \
    --param-file $(ros2 pkg prefix omx_variable_stiffness_controller)/share/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml \
    --activate-as-group
ros2 run controller_manager spawner robot1/gravity_comp_controller \
    --controller-manager /robot1/controller_manager \
    --param-file $(ros2 pkg prefix omx_gravity_comp_controller)/share/omx_gravity_comp_controller/config/gravity_comp_controller.yaml \
    --activate-as-group
```
### Gazebo Not Starting

Make sure Gazebo is installed:

```bash
apt-get update
apt-get install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### Gazebo "Address already in use"

If you see `EXCEPTION: Unable to start server[bind: Address already in use]`, a previous `gzserver` is still running:

```bash
pkill -f gzserver || true
pkill -f gzclient || true
```

### Relaunching After Previous Session

If you see errors like `Controller already loaded` or `Entity already exists`, clean up first:

```bash
./src/omx_dual_bringup/scripts/cleanup.sh
```

Then relaunch:
```bash
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py
```

### "Stereo is NOT SUPPORTED" Warning

The RViz2 message `Stereo is NOT SUPPORTED` is **informational only** - RViz2 works fine without stereo display support. This is normal in containers or VMs.

### Multiple Gazebo Windows or Black Screen

If you see multiple Gazebo windows or a blank client:

```bash
# Use a single Gazebo server for both robots (recommended for visualization)
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=single

# Or disable auto-starting the client and launch it manually
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py start_gzclient:=false
gzclient
```

## Detailed File Reference — Variable Stiffness Single-Hardware

This section complements `project_status.md` with a concise, actionable
reference for the variable stiffness single-hardware configuration,
launch behavior, and controller implementation.

- `ws/src/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml`
  - Purpose: central controller parameters for `/omx/controller_manager`.
  - Key parameters (type : meaning : recommended/default):
    - `update_rate` (int) : controller loop frequency in Hz — **500** recommended
    - `use_sim_time` (bool) : `false` for hardware, `true` for Gazebo
    - `joints` (seq[string]) : e.g. `[joint1, joint2, joint3, joint4]`
    - `state_interfaces` / `command_interfaces` (seq[string]) : include `position, velocity, effort` and `effort` respectively
    - Cartesian waypoints: `start_position`, `end_position` (3 floats, meters)
    - `stiffness_homing`, `stiffness_rot`, `damping_default` (3 floats — N/m & Ns/m)
    - `stiffness_profile_*` / `damping_profile_*` : optional sequences or CSV-loaded profile
    - `max_joint_torque_command` (double) : raw servo units clamp (0 disables). For XM430: **<=1100** to avoid servo-side clipping
    - `contact_force_filter_alpha` (double) : 0.02 recommended (low-pass filter alpha)
    - `num_trajectory_samples` (int) : IK preflight resolution (default 50)

  - Practical notes:
    - Ensure the entire `ros__parameters` block is registered under the same node name used by the controller_manager (default `/omx/controller_manager`). If not, the controller will run with defaults.
    - Use absolute serial paths `/dev/serial/by-id/...` in hardware launches to avoid port reordering.

- `ws/src/omx_variable_stiffness_controller/launch/variable_stiffness_control.launch.py`
  - Responsibilities: auto-detect serial port, set `robot_namespace`, start RSP, start `ros2_control_node` (or let Gazebo plugin spawn CM), load controller YAML into CM, and start spawner nodes with controlled delays.
  - Useful launch args: `robot_namespace`, `serial_port`, `sim` (bool), `enable_logger`, `controller_delay`, `spawn_delay`.

- C++ controller files (`src/*.cpp` & `include/*.hpp`)
  - The controller implements:
    - Lifecycle hooks with `on_init()` declaring defaults
    - `state_interface_configuration()` and `command_interface_configuration()` returning `INDIVIDUAL` names like `jointN/position`
    - Preflight IK trajectory validation (DLS + KDL)
    - Singularity detection using σ_min, with escape via `singularity_escape_K/D`
    - Per-joint torque clamping plus proportional scaling to preserve Cartesian direction
    - Homing convergence gate and joint-space regularization
    - Contact force estimator using EMA with `contact_force_filter_alpha`

  - Useful runtime logs: `Got robot_description from topic`, `[SAFETY] Trajectory validated. Min sigma=`, `[CLAMP]`, `[HOMING] Extending homing:`. Monitor these to confirm expected behavior.

  - Tuning quick tips:
    - If EE sags in z: increase `damping_default` on `z` (example: 3 → 6 Ns/m)
    - If per-joint clamping occurs often: reduce commanded stiffness or set `max_joint_torque_command` slightly lower and rely on proportional scaling
    - Increase `num_trajectory_samples` for longer trajectories; preflight time grows linearly

Refer to `project_status.md` for the exhaustive historical rationale and example CSV formats for stiffness/damping profiles.


### TF Frame Issues

If you see TF warnings about missing frames:

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Visualize TF in RViz
ros2 run rqt_tf_tree rqt_tf_tree
```

## Hardware Setup

### Physical Robot Connection

1. Connect both Open Manipulator X robots via USB to separate ports
2. Power on both robots
3. Verify serial connections: `ls -la /dev/ttyUSB*`
4. Launch with auto-detection or specify ports explicitly

### Port Identification

```bash
# List ports by ID (recommended)
ls -la /dev/serial/by-id/

# Create udev rules for persistent naming (optional)
# Example: /etc/udev/rules.d/99-omx.rules
# SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="XXXXX", SYMLINK+="omx_robot1"
```

## Advanced Usage

### Running with MoveIt2 (Future)

The namespace isolation allows integration with MoveIt2 for motion planning:

```python
# Example: Planning for robot1
from moveit_py import MoveGroupInterface

robot1_group = MoveGroupInterface(node, "robot1/arm")
robot1_group.set_joint_value_target([0.0, 0.5, 0.3, 0.0])
robot1_group.move()
```

### Custom Controller Parameters

Edit the YAML files in `config/` to adjust:
- Damping coefficients
- Torque/current limits
- Update rates
- Safety parameters

## Dependencies

Required ROS2 packages:
- `controller_manager`
- `ros2_control`
- `ros2controlcli`
- `gazebo_ros`
- `gazebo_ros2_control`
- `robot_state_publisher`
- `rviz2`
- `omx_gravity_comp_controller`

## License

Apache-2.0

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review ROS2 control documentation
3. Check controller manager logs: `ros2 control list_controllers -c /robot1/controller_manager`


### TL;DR:

### Build:
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select omx_dual_bringup omx_gravity_comp_controller
source install/setup.bash



source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py start_rviz:=false 2>&1





pkill -9 ros2_control_node 2>/dev/null; pkill -9 robot_state_publisher 2>/dev/null; sleep 2; source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py sim:=false






The following shell recipe reproduces the typical development workflow
by rebuilding the controller package (or the robot description after an
URDF/xacro change) and launching the Gazebo‑based simulation with RViz.
This is the same sequence used in our tests and the troubleshooting
examples above.

```bash
# Window 1: build & start simulation (GUI enabled)
# NOTE: when the Gazebo ros2_control plugin is available we no longer
# start a separate ``ros2_control_node`` in the launch file.  The plugin
# uses its own controller_manager instance which must be configured via the
# ``robot_description`` published by the
# ``robot_state_publisher``.  In the devcontainer the Gazebo server often
# crashes shortly after startup, so the full end‑to‑end simulation is only
# reliable on a host with a working Gazebo installation.  To exercise the
# controller lifecycle automatically we added an opaque function in
# ``gazebo_variable_stiffness.launch.py`` that repeatedly calls
# ``switch_controller`` until the variable stiffness controller becomes
# active (or a timeout elapses).  When running tests, this logic is pulled
# in automatically by the launch file.
#
 itself will spawn a controller_manager under `/omx/controller_manager`.
# If you want to force the standalone node (e.g. for headless testing) set
# ``launch_gazebo:=false`` or ``use_fake_hardware:=true`` so the URDF does
# not include the gazebo plugin; otherwise the two managers conflict and
# the node aborts with a pluginlib error about ``GazeboSystem``.
#
# In addition, the launch recipes will skip the controller "spawner" nodes
# whenever ``launch_gazebo`` is false.  Those spawners are harmless in a
# real Gazebo run but on fake‑hardware they frequently block the
# ``/omx/controller_manager`` services and cause timeouts, which broke the
# earlier headless integration tests.  The YAML configuration alone is
# sufficient to load the controllers on startup, and the tests now also
# attempt to load missing controllers directly via service calls.
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f ros2_control_node || true
sleep 1

source /opt/ros/humble/setup.bash
source install/setup.bash || true

# rebuild after editing xacro/config or controller code
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_variable_stiffness_controller
source install/setup.bash
clear

ros2 launch omx_variable_stiffness_controller \
  gazebo_variable_stiffness.launch.py \
  csv_file:=ws/src/omx_variable_stiffness_controller/config/stiffness_aniso.csv \
  gui:=true
```

```bash
# Window 2: inspect parameters and manually configure the controller
source /opt/ros/humble/setup.bash
source install/setup.bash || true
ros2 param get /omx/variable_stiffness_controller joints
ros2 param get /omx/variable_stiffness_controller robot_description_node
ros2 service call /omx/controller_manager/configure_controller \
  controller_manager_msgs/srv/ConfigureController "{name: 'variable_stiffness_controller'}"
ros2 control list_controllers -c /omx/controller_manager
```

*Note:* the variable stiffness controller requires **effort command
interfaces** on each joint.  If you encounter ``Failed to configure
controller`` or ``Not acceptable command interfaces`` errors during
startup, rebuild after adding `<command_interface name="effort"/>`
entries to the URDF (see the description package xacro).  This fix was
recently merged so a clean build is necessary.
