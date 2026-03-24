# Fix Plans: LivePlot GUI, Controller Speedup, Tool Consolidation

> Date: 2026-03-24

---

## Issue 1: LivePlot GUI window doesn't appear alongside Gazebo GUI

**Root cause:** [live_plot_logs.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/live_plot_logs.py#L58-L80) defaults to Agg (headless) backend unless env var `LIVEPLOT_USE_GUI=1` is set. The [launch file](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/ws/src/omx_variable_stiffness_controller/launch/dual_gazebo_variable_stiffness.launch.py#L158-L169) never sets this env var — so the plotter always runs headless even with `gui:=true enable_live_plot:=true`.

**Fixes:**

### 1a. Launch file — set `LIVEPLOT_USE_GUI`

#### [MODIFY] [dual_gazebo_variable_stiffness.launch.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/ws/src/omx_variable_stiffness_controller/launch/dual_gazebo_variable_stiffness.launch.py)

Add to `env_actions` list (gated on GUI):

```diff
 env_actions = [
     SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
+    SetEnvironmentVariable('LIVEPLOT_USE_GUI', '1'),  # enable matplotlib GUI backend
     ...
 ]
```

And pass `--screenshot-dir` for headless fallback in the `live_plot` `ExecuteProcess`:

```diff
 live_plot = ExecuteProcess(
     cmd=[
         'python3', '/workspaces/omx_ros2/tools/live_plot_logs.py',
         '--controller', 'variable_stiffness',
         '--namespace', '/robot1',
         '--namespace2', '/robot2',
+        '--screenshot-dir', '/tmp/live_plot_screenshots',
         '--window', '30.0',
         '--interval', '0.5',
     ],
```

### 1b. Add `--gui` CLI flag to live_plot_logs.py

#### [MODIFY] [live_plot_logs.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/live_plot_logs.py)

```diff
 # In build_cli():
+p.add_argument('--gui', action='store_true', default=False,
+               help='Force GUI matplotlib backend (overrides LIVEPLOT_USE_GUI env var)')

 # In _choose_matplotlib_backend():
-use_gui = os.environ.get('LIVEPLOT_USE_GUI', '0') in ('1', 'true', 'True')
+use_gui = getattr(_cli_args, 'gui', False) or os.environ.get('LIVEPLOT_USE_GUI', '0') in ('1', 'true', 'True')
```

> [!IMPORTANT]
> Apply the same `LIVEPLOT_USE_GUI` fix to all 7 launch files that support `enable_live_plot`: `single_robot_test`, `gazebo_variable_stiffness`, `dual_hardware_variable_stiffness`, `dual_hardware_gravity_comp`, `single_robot_hardware`, `dual_gazebo_gravity_comp`.

---

## Issue 2: controller_manager loads very slowly in 5-stage test

**Root causes:**

| Factor | Impact | Location |
|--------|--------|----------|
| `TimerAction(period=60.0)` before spawners | 120s wasted (60s × 2 robots) | [launch L147, L153](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/ws/src/omx_variable_stiffness_controller/launch/dual_gazebo_variable_stiffness.launch.py#L144-L155) |
| Subprocess per poll with full bash source chain | ~3–5s per poll cycle | [5stage L30–33](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_5stage_test.py#L30-L33) |
| Redundant [wait_for_service()](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_5stage_test.py#56-64) inside [check_controller_active()](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_5stage_test.py#126-137) | Repeated 2s timeouts | [5stage L129](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_5stage_test.py#L126-L136) |

**Fixes:**

### 2a. Reduce timer delay

#### [MODIFY] [dual_gazebo_variable_stiffness.launch.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/ws/src/omx_variable_stiffness_controller/launch/dual_gazebo_variable_stiffness.launch.py)

```diff
-TimerAction(period=60.0, actions=[load_jsb_r1, load_vs_r1_default, load_vs_r1_press])
+TimerAction(period=15.0, actions=[load_jsb_r1, load_vs_r1_default, load_vs_r1_press])
 ...
-TimerAction(period=60.0, actions=[load_jsb_r2, load_vs_r2_default, load_vs_r2_press])
+TimerAction(period=15.0, actions=[load_jsb_r2, load_vs_r2_default, load_vs_r2_press])
```

The `spawner` node already has `--controller-manager-timeout 120` retry logic — the launch doesn't need to pre-delay.

### 2b. Cache sourced environment in test harness

#### [MODIFY] [dual_gazebo_5stage_test.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_5stage_test.py)

```diff
+# Cache the sourced ROS environment once at startup
+import shlex
+_CACHED_ENV = None
+def get_ros_env():
+    global _CACHED_ENV
+    if _CACHED_ENV is None:
+        result = subprocess.run(
+            'bash -lc "source /opt/ros/humble/setup.bash && '
+            'source /workspaces/omx_ros2/ws/install/setup.bash && env"',
+            shell=True, capture_output=True, text=True, timeout=30)
+        _CACHED_ENV = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
+    return _CACHED_ENV

 def run_ros2_cmd(cmd, timeout=20, check=True):
-    env_source = 'source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && '
-    wrapped = f"bash -lc '{env_source}{cmd}'"
-    return run_cmd(wrapped, timeout=timeout, check=check)
+    try:
+        return subprocess.run(cmd, shell=True, timeout=timeout, check=check,
+                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
+                              text=True, env=get_ros_env())
+    except subprocess.TimeoutExpired as e:
+        return subprocess.CompletedProcess(e.cmd, 1, stdout='', stderr=str(e))
```

### 2c. Remove redundant service checks in Stage 2

Stage 1 already confirms `controller_manager/list_controllers` exists. Stage 2 should skip [wait_for_service()](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_5stage_test.py#56-64) and directly call `ListControllers`.

**Expected improvement:** ~5–8 min → ~2–3 min total test time.

---

## Issue 3: [dual_press_coordinator.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_press_coordinator.py) vs [dual_gazebo_opposing_push.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_opposing_push.py)

### Functional comparison

| Capability | [dual_press_coordinator](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_press_coordinator.py) | [dual_gazebo_opposing_push](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_opposing_push.py) |
|---|---|---|
| **Purpose** | One-shot waypoint publisher | Full scenario orchestrator |
| **Box spawning** | ❌ | ✅ (SDF + `/spawn_entity`) |
| **Trajectory** | Single waypoint, same XYZ to both | Cyclic opposing push, mirrored Y |
| **Controller readiness** | ✅ `ListControllers` polling | ✅ `ListControllers` polling (identical) |
| **Bus health** | ✅ `/bus_healthy` subscription | ✅ `/bus_healthy` subscription (identical) |
| **Duration control** | ❌ Fire-and-forget | ✅ `--duration` |
| **Line interpolation** | ❌ | ✅ [publish_line()](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_opposing_push.py#203-210) |
| **Intended use** | Launch file companion / CLI | Standalone Gazebo test |

### Recommendation: **Extract shared utilities, keep both tools**

Do NOT merge — they serve different purposes. Instead:

#### [NEW] `tools/dual_robot_utils.py`

Extract these identical patterns from both files:

```python
"""Shared utilities for dual-robot coordination tools."""

def wait_for_controllers(node, ns1, ns2, ctrl1, ctrl2, timeout=60.0):
    """Poll ListControllers until both controllers are active."""
    ...  # identical logic from both files

class BusHealthMonitor:
    """Mixin: subscribe to /bus_healthy topics, expose health1/health2 flags."""
    ...  # identical pattern from both files

def publish_waypoint_pair(pub1, pub2, pose1, pose2, health1, health2, retries=3, interval=0.05):
    """Publish a PoseStamped to both robots with retry and health-gate."""
    ...  # shared publish logic
```

#### [MODIFY] [tools/dual_press_coordinator.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_press_coordinator.py)
- Import from `dual_robot_utils.py`, remove duplicated [wait_for_active_controllers](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_press_coordinator.py#82-119) and bus health code

#### [MODIFY] [tools/dual_gazebo_opposing_push.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_opposing_push.py)
- Import from `dual_robot_utils.py`, remove duplicated [wait_for_controllers](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_opposing_push.py#114-146) and bus health code
- Optionally add `--mode opposing|coordinated` flag

### Priority

| Step | Effort |
|---|---|
| Create `tools/dual_robot_utils.py` | 1h |
| Refactor both tools to use it | 1h |
| Unit test shared utilities | 1h |

---

## Verification Plan

1. Launch `dual_gazebo_variable_stiffness.launch.py gui:=true enable_live_plot:=true` → confirm matplotlib window appears
2. Run [dual_gazebo_5stage_test.py](file:///home/vrcontrollers/omx_ros2_varstiff/omx-ros2-gravity-comp/tools/dual_gazebo_5stage_test.py) → confirm total time < 3 min
3. Run both coordinator tools → confirm they still work after refactor
