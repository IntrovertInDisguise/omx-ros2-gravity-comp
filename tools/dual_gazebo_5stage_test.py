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


def run_cmd(cmd, timeout=20, check=True):
    return subprocess.run(cmd, shell=True, timeout=timeout, check=check,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          text=True)


def run_ros2_cmd(cmd, timeout=20, check=True):
    env_source = 'source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && '
    wrapped = f"bash -lc '{env_source}{cmd}'"
    return run_cmd(wrapped, timeout=timeout, check=check)


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
    out = run_ros2_cmd(f"ros2 topic echo /{robot}/joint_states --once", timeout=timeout, check=False)
    if out.returncode != 0 or not out.stdout:
        return None
    m = re.search(r'position:\s*\[\s*([-+0-9.eE]+)', out.stdout)
    return float(m.group(1)) if m else None


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

        # Stage 1: spawn verification
        print('Stage 1: checking robot1/robot2/joint_states and box in world')
        # ensure gazebo has model list service available before sampling
        if not wait_for_service('/get_model_list', timeout=90):
            print('  Stage1 FAILED: /get_model_list service not available'); launch_proc.kill(); time.sleep(1); continue

        stage1_ok = False
        t0 = time.time()
        while time.time() - t0 < 240:
            if not gzserver_alive():
                print('  gzserver died during stage1')
                break

            robots_ready = wait_for_topic('/robot1/joint_states', timeout=2) and wait_for_topic('/robot2/joint_states', timeout=2)
            if not robots_ready:
                print('  waiting for robot joint_states topics')
                time.sleep(1)
                continue

            # require all expected entities to be present to trust Stage1
            models = get_model_list()
            if not models:
                print('  waiting for /get_model_list service response')
                time.sleep(1)
                continue

            required_models = {'robot1', 'robot2', 'opposing_push_box'}
            missing = required_models - set(models)
            if not missing:
                stage1_ok = True
                print('  Stage1 OK: all required models present in get_model_list')
                break

            # also try /gazebo/model_states and /model_states text scan to avoid remapping uncertainty
            ms = get_model_states()
            if 'opposing_push_box' in ms:
                missing.discard('opposing_push_box')

            if not missing:
                stage1_ok = True
                print('  Stage1 OK: all required models present via model_states topic')
                break

            print(f'  Stage1 waiting, missing models: {sorted(missing)}, available in get_model_list: {models}')
            time.sleep(1)

        if not stage1_ok:
            print('Stage1 FAILED'); launch_proc.kill(); time.sleep(1); continue

        # Stage 2: basic movement
        print('Stage 2: commanding tiny moves and checking joint changes')
        if not wait_for_topic('/robot1/joint_states', timeout=30) or not wait_for_topic('/robot2/joint_states', timeout=30):
            print('  Stage2 FAILED: joint state topics not present'); launch_proc.kill(); time.sleep(1); continue

        # Additional wait for controller_manager to respond consistently (extra generous)
        if not wait_for_service('/robot1/controller_manager/list_controllers', timeout=180) or not wait_for_service('/robot2/controller_manager/list_controllers', timeout=180):
            print('  Stage2 FAILED: controller_manager services not available'); launch_proc.kill(); time.sleep(1); continue

        # Poll for a meaningful joint state on both robots
        pos1_0 = get_joint_positions('robot1', timeout=30)
        pos2_0 = get_joint_positions('robot2', timeout=30)
        if pos1_0 is None or pos2_0 is None:
            print('  Stage2 FAILED: unable to read joint positions after 30s'); launch_proc.kill(); time.sleep(1); continue

        send_joint_command('robot1', pos1_0 + 0.2)
        send_joint_command('robot2', pos2_0 + 0.2)
        time.sleep(2)
        pos1_1 = get_joint_positions('robot1')
        pos2_1 = get_joint_positions('robot2')
        if pos1_1 is None or pos2_1 is None or abs(pos1_1 - pos1_0) < 0.05 or abs(pos2_1 - pos2_0) < 0.05:
            print(f'  Stage2 FAILED pos1 {pos1_0}->{pos1_1} pos2 {pos2_0}->{pos2_1}'); launch_proc.kill(); time.sleep(1); continue
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
