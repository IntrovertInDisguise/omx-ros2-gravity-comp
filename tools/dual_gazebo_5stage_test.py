#!/usr/bin/env python3
"""
5-stage functional test for dual-robot variable-stiffness system in Gazebo.

Stages:
1. both robots+box spawned
2. both robots controlled and can move
3. synced behaviour pressing box (contact_wrench appears for both)
4. box pressed with negligible moment (torque norm low)
5. force signature recorded (non-zero force magnitude)
"""

import argparse
import subprocess
import sys
import time
import math
import re
import os


def run_cmd(cmd, timeout=20, check=True):
    try:
        return subprocess.run(cmd, shell=True, timeout=timeout, check=check,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True)
    except subprocess.TimeoutExpired as e:
        return subprocess.CompletedProcess(e.cmd, 1, stdout='', stderr=str(e))


_ROS_ENV = None

def get_ros_env():
    global _ROS_ENV
    if _ROS_ENV is None:
        out = subprocess.run(
            'bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && env"',
            shell=True, capture_output=True, text=True, timeout=30)
        env = {}
        for line in out.stdout.splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                env[k] = v
        # Keep system env to avoid losing PATH etc
        env.update(os.environ)
        _ROS_ENV = env
    return _ROS_ENV

def run_ros2_cmd(cmd, timeout=20, check=True):
    try:
        return subprocess.run(cmd, shell=True, timeout=timeout, check=check,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True, env=get_ros_env())
    except subprocess.TimeoutExpired as e:
        return subprocess.CompletedProcess(e.cmd, 1, stdout='', stderr=str(e))


def gzserver_alive():
    val = subprocess.run('pgrep -f gzserver', shell=True, check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return val.returncode == 0


def wait_for_topic(name, timeout=120):
    start = time.time()
    while time.time() - start < timeout:
        r = run_ros2_cmd(f"ros2 topic list | grep -x '{name}'", timeout=5, check=False)
        if r.returncode == 0:
            return True
        # Fallback: try to see if any message can be got from the topic directly
        r2 = run_ros2_cmd(f"ros2 topic echo {name} --once", timeout=5, check=False)
        if r2.returncode == 0 or (r2.stdout and r2.stdout.strip()):
            return True
        time.sleep(1)
    return False


def wait_for_service(name, timeout=120):
    start = time.time()
    while time.time() - start < timeout:
        r = run_ros2_cmd(f"ros2 service list | grep -x '{name}'", timeout=5, check=False)
        if r.returncode == 0:
            return True
        time.sleep(1)
    return False


def get_model_states():
    out = run_ros2_cmd('ros2 topic echo /gazebo/model_states --once', timeout=15, check=False)
    return out.stdout if out.returncode == 0 else ''


def get_model_list():
    try:
        out = run_ros2_cmd("ros2 service call /get_model_list gazebo_msgs/srv/GetModelList '{}'", timeout=8, check=False)
    except subprocess.TimeoutExpired:
        return []
    if out.returncode != 0 or not out.stdout:
        print('[get_model_list] no output or failed return code:', out.returncode, 'stderr:', out.stderr.strip())
        return []

    print('[get_model_list] stdout:', out.stdout.strip())

    # Preferred parsing: model_names=['ground_plane', 'robot1', ...]
    m = re.search(r'model_names\s*=\s*\[([^\]]*)\]', out.stdout, re.DOTALL)
    if m:
        raw = m.group(1)
        names = re.findall(r"'([^']+)'", raw)
        if names:
            return names

    # fallback: avoid capturing empty frame_id or other metadata
    names = re.findall(r"'([^']*)'", out.stdout) or re.findall(r'\"([^\"]*)\"', out.stdout)
    return [n for n in names if n.strip() and n != 'frame_id']


def get_joint_positions(robot, timeout=20):
    try:
        out = run_ros2_cmd(f"ros2 topic echo /{robot}/joint_states --once", timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        return None
    if out.returncode != 0 or not out.stdout:
        return None
    m = re.search(r'position:\s*\[\s*([-+0-9.eE]+)', out.stdout)
    return float(m.group(1)) if m else None


def wait_for_joint_position(robot, timeout=60):
    start = time.time()
    while time.time() - start < timeout:
        pos = get_joint_positions(robot, timeout=5)
        if pos is not None:
            return pos
        time.sleep(1)
    return None


def wait_for_joint_data(robot, timeout=60):
    start = time.time()
    while time.time() - start < timeout:
        pos = get_joint_positions(robot, timeout=5)
        if pos is not None:
            return pos
        time.sleep(1)
    return None


def check_controller_active(namespace, controller, timeout=60):
    # Assume service is already present in Stage1; avoid repeated name-service polling.
    start = time.time()
    while time.time() - start < timeout:
        r = run_ros2_cmd(
            f"ros2 service call /{namespace}/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{{}}'",
            timeout=10, check=False)
        if r.returncode == 0 and f"name: '{controller}'" in r.stdout and 'state: active' in r.stdout:
            return True
        time.sleep(1)
    return False

def ensure_controller_active(robot, controller_name, timeout=60):
    """
    Ensure a controller is active for a given robot.
    If not active, try to load and then activate it.
    Returns True if active within timeout.
    """
    start = time.time()
    while time.time() - start < timeout:
        r = run_ros2_cmd(
            f"ros2 service call /{robot}/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{{}}'",
            timeout=10, check=False)
        if r.returncode == 0:
            if f"name: '{controller_name}'" in r.stdout and 'state: active' in r.stdout:
                print(f'  {robot}: {controller_name} already active')
                return True
            if f"name: '{controller_name}'" not in r.stdout:
                print(f'  {robot}: loading {controller_name}...')
                run_ros2_cmd(
                    f"ros2 service call /{robot}/controller_manager/load_controller controller_manager_msgs/srv/LoadController '{{name: {controller_name}}}'",
                    timeout=10, check=False)
            else:
                print(f'  {robot}: activating {controller_name}...')
                run_ros2_cmd(
                    f"ros2 service call /{robot}/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{{activate_controllers: [{controller_name}], deactivate_controllers: [], strictness: 1}}'",
                    timeout=10, check=False)
        else:
            print(f'  {robot}: list_controllers call failed with code {r.returncode}, retrying...')
        time.sleep(1)
    return False


def send_joint_command(robot, value):
    topic = f'/{robot}/{robot}_variable_stiffness/commands'
    cmd = f"ros2 topic pub {topic} std_msgs/msg/Float64MultiArray '{{layout: {{dim: []}}, data: [{value}]}}' --once"
    run_ros2_cmd(cmd, timeout=5, check=False)


def get_contact_wrench(robot):
    topic = f'/{robot}/{robot}_variable_stiffness/contact_wrench'
    out = run_ros2_cmd(f"ros2 topic echo {topic} --once", timeout=10, check=False)
    if out.returncode != 0 or not out.stdout:
        return None, None
    force = torque = None
    fm = re.search(r'force:\s*\[\s*([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)\s*\]', out.stdout)
    tm = re.search(r'torque:\s*\[\s*([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)\s*\]', out.stdout)
    if fm:
        force = [float(fm.group(i)) for i in range(1, 4)]
    if tm:
        torque = [float(tm.group(i)) for i in range(1, 4)]
    return force, torque


def run_test(max_iterations=3):
    for trial in range(1, max_iterations + 1):
        print(f"===== TRIAL {trial}/{max_iterations} =====")

        # cleanup
        subprocess.run('pkill -f dual_gazebo_variable_stiffness.launch.py || true', shell=True)
        subprocess.run('pkill -f gzserver || true', shell=True)
        subprocess.run('pkill -f gzclient || true', shell=True)
        time.sleep(2)

        launch_proc = subprocess.Popen(
            'bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && '
            'ros2 launch omx_variable_stiffness_controller dual_gazebo_variable_stiffness.launch.py gui:=false launch_gazebo:=true enable_live_plot:=false start_rviz:=false"',
            shell=True,
            stdout=open('/tmp/dual_gazebo_5stage_launch.log', 'w'),
            stderr=subprocess.STDOUT,
        )

        # ========== STAGE 1: Spawn verification ==========
        print('Stage 1: checking robot1/robot2/joint_states and box in world')
        stage1_ok = False
        t0 = time.time()
        # Allow time for launch to start Gazebo and spawn scripts
        time.sleep(5)
        while time.time() - t0 < 300:  # longer overall timeout
            if not gzserver_alive():
                print('  gzserver died during stage1')
                break

            # ---- 1. Check model list via /get_model_list or /gazebo/model_states ----
            models_ok = False
            use_model_states_fallback = False

            if wait_for_service('/get_model_list', timeout=2):
                models = get_model_list()
                print(f'  get_model_list returned: {models}')
                if 'robot1' in models and 'robot2' in models and 'opposing_push_box' in models:
                    models_ok = True
                    print('  Models present in /get_model_list')
                else:
                    use_model_states_fallback = True
            else:
                print('  /get_model_list service not available yet')
                use_model_states_fallback = True

            if use_model_states_fallback:
                if wait_for_topic('/gazebo/model_states', timeout=2):
                    model_states = get_model_states()
                    print(f'  /gazebo/model_states payload contains robot1: {"robot1" in model_states}, robot2: {"robot2" in model_states}, box: {"opposing_push_box" in model_states}')
                    if 'robot1' in model_states and 'robot2' in model_states and 'opposing_push_box' in model_states:
                        models_ok = True
                        print('  Models present in /gazebo/model_states (fallback)')

            if not models_ok:
                time.sleep(1)
                continue

            # ---- 2. Wait for controller manager services ----
            all_services = run_ros2_cmd('ros2 service list', timeout=5, check=False).stdout.splitlines()
            relevant = [s for s in all_services if 'controller_manager' in s or 'robot1' in s or 'robot2' in s]
            print(f'  services present: {relevant}')
            if not (wait_for_service('/robot1/controller_manager/list_controllers', timeout=5) and
                    wait_for_service('/robot2/controller_manager/list_controllers', timeout=5)):
                time.sleep(1)
                continue

            # ---- 3. Ensure joint_state_broadcaster is active ----
            if not ensure_controller_active('robot1', 'joint_state_broadcaster', timeout=30):
                print('  robot1 joint_state_broadcaster activation failed')
                time.sleep(1)
                continue
            if not ensure_controller_active('robot2', 'joint_state_broadcaster', timeout=30):
                print('  robot2 joint_state_broadcaster activation failed')
                time.sleep(1)
                continue

            # ---- 4. Ensure variable stiffness controller is active ----
            if not ensure_controller_active('robot1', 'robot1_variable_stiffness', timeout=30):
                print('  robot1 variable_stiffness controller activation failed')
                time.sleep(1)
                continue
            if not ensure_controller_active('robot2', 'robot2_variable_stiffness', timeout=30):
                print('  robot2 variable_stiffness controller activation failed')
                time.sleep(1)
                continue

            # ---- 5. Wait for joint state data ----
            pos1 = wait_for_joint_data('robot1', timeout=60)
            pos2 = wait_for_joint_data('robot2', timeout=60)
            if pos1 is not None and pos2 is not None:
                stage1_ok = True
                print('  Stage1 OK')
                break
            time.sleep(1)

        if not stage1_ok:
            print('Stage1 FAILED')
            launch_proc.kill()
            time.sleep(1)
            continue

            if not missing and box_detected:
                stage1_ok = True
                print('  Stage1 OK: all required models present in get_model_list')
                break

            print(f'  Stage1 waiting, missing models: {sorted(missing)}, available in get_model_list: {models}')
            time.sleep(1)

        if not stage1_ok:
            print('Stage1 FAILED'); launch_proc.kill(); time.sleep(1); continue

        # ========== STAGE 2: Basic movement ==========
        print('Stage 2: commanding tiny moves and checking joint changes')

        # Wait for controller_manager services (already done in Stage1, but ensure fresh)
        if not wait_for_service('/robot1/controller_manager/list_controllers', timeout=30) or \
           not wait_for_service('/robot2/controller_manager/list_controllers', timeout=30):
            print('  Stage2 FAILED: controller_manager services not available')
            launch_proc.kill(); time.sleep(1); continue

        # Wait for joint_state_broadcaster to be active (critical)
        def is_joint_state_broadcaster_active(robot):
            return check_controller_active(robot, 'joint_state_broadcaster', timeout=20)

        if not (is_joint_state_broadcaster_active('robot1') and is_joint_state_broadcaster_active('robot2')):
            print('  Stage2 FAILED: joint_state_broadcaster not active')
            launch_proc.kill(); time.sleep(1); continue

        def get_first_joint_position(robot, timeout=60):
            start = time.time()
            while time.time() - start < timeout:
                pos = get_joint_positions(robot)
                if pos is not None:
                    print(f'  {robot} initial position: {pos:.4f}')
                    return pos
                time.sleep(1)
            return None

        pos1_0 = get_first_joint_position('robot1', timeout=30)
        pos2_0 = get_first_joint_position('robot2', timeout=30)
        if pos1_0 is None or pos2_0 is None:
            print('  Stage2 FAILED: joint_states never published any data')
            launch_proc.kill(); time.sleep(1); continue

        print(f'  Sending +0.1 rad command to robot1 (current {pos1_0:.4f})')
        print(f'  Sending +0.1 rad command to robot2 (current {pos2_0:.4f})')
        send_joint_command('robot1', pos1_0 + 0.1)
        send_joint_command('robot2', pos2_0 + 0.1)

        time.sleep(2)
        pos1_1 = get_joint_positions('robot1')
        pos2_1 = get_joint_positions('robot2')
        if pos1_1 is None or pos2_1 is None:
            print('  Stage2 FAILED: could not read joint positions after command')
            launch_proc.kill(); time.sleep(1); continue

        diff1 = abs(pos1_1 - pos1_0)
        diff2 = abs(pos2_1 - pos2_0)
        print(f'  robot1 moved: {pos1_0:.4f} → {pos1_1:.4f} (Δ {diff1:.4f})')
        print(f'  robot2 moved: {pos2_0:.4f} → {pos2_1:.4f} (Δ {diff2:.4f})')

        if diff1 < 0.02 or diff2 < 0.02:
            print('  Stage2 FAILED: movement insufficient')
            launch_proc.kill(); time.sleep(1); continue

        print('  Stage2 OK')

        # Stage 3: synced pressing and contact wrench event
        print('Stage 3: synchronized pressing and contact wrench')
        send_joint_command('robot1', pos1_0 + 0.5)
        send_joint_command('robot2', pos2_0 + 0.5)
        t0 = time.time(); contact_ok = False
        while time.time() - t0 < 30:
            f1, t1 = get_contact_wrench('robot1')
            f2, t2 = get_contact_wrench('robot2')
            if f1 and f2:
                contact_ok = True
                print('  Stage3 OK contact wrench present')
                break
            time.sleep(1)
        if not contact_ok:
            print('  Stage3 FAILED'); launch_proc.kill(); time.sleep(1); continue

        # Stage 4: no moment about box centre
        print('Stage 4: torque magnitude check')
        _, t1 = get_contact_wrench('robot1'); _, t2 = get_contact_wrench('robot2')
        if t1 is None or t2 is None:
            print('  Stage4 FAILED: no torque info'); launch_proc.kill(); time.sleep(1); continue
        torque1 = math.sqrt(sum(x * x for x in t1)); torque2 = math.sqrt(sum(x * x for x in t2))
        if torque1 < 0.1 and torque2 < 0.1:
            print(f'  Stage4 OK: torque1 {torque1:.3f}, torque2 {torque2:.3f}')
        else:
            print(f'  Stage4 FAILED: torque1 {torque1:.3f}, torque2 {torque2:.3f}'); launch_proc.kill(); time.sleep(1); continue

        # Stage 5: force signature non-zero
        print('Stage 5: force magnitude check')
        f1, _ = get_contact_wrench('robot1'); f2, _ = get_contact_wrench('robot2')
        force1 = math.sqrt(sum(x * x for x in f1)) if f1 else 0
        force2 = math.sqrt(sum(x * x for x in f2)) if f2 else 0
        if force1 > 5.0 and force2 > 5.0:
            print(f'  Stage5 OK: force1 {force1:.2f} N, force2 {force2:.2f} N')
        else:
            print(f'  Stage5 FAILED: force1 {force1:.2f}, force2 {force2:.2f}'); launch_proc.kill(); time.sleep(1); continue

        print('ALL STAGES PASSED')
        launch_proc.kill(); time.sleep(1)
        return True

    print('FAILED: no full pass')
    return False


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--max-iterations', type=int, default=3)
    args = parser.parse_args()
    ok = run_test(max_iterations=args.max_iterations)
    sys.exit(0 if ok else 1)
