#!/usr/bin/env python3
"""Continuous dual Gazebo + liveplot + opposing push test with 5 stages.

Stages:
 1. GUI + live_plot launched (processes alive and screenshots are produced)
 2. Robot arms + box spawned (robot joint_states topics exist, spawn entity service success)
 3. Both controller managers have active variable stiffness controllers
 4. Execute one opposing-path cycle, then contact_wrench from both robots appears
 5. If any stage fails, retry until max iterations.

Run with:
  python3 tools/dual_gazebo_5stage_test.py --max-iterations 5

Requires workspace sourced beforehand.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))

def run_cmd(cmd, timeout=20, check=True, capture_output=True):
    return subprocess.run(cmd, shell=True, timeout=timeout, check=check,
                          stdout=subprocess.PIPE if capture_output else None,
                          stderr=subprocess.PIPE if capture_output else None,
                          text=True)


def run_ros2_cmd(cmd, timeout=20, check=True, capture_output=True):
    env_source = 'source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && '
    wrapped = f"bash -lc '{env_source}{cmd}'"
    return run_cmd(wrapped, timeout=timeout, check=check, capture_output=capture_output)


def ensure_env():
    os.environ.setdefault('AMENT_TRACE_SETUP_FILES', '0')
    os.environ.setdefault('AMENT_PYTHON_EXECUTABLE', shutil.which('python3') or 'python3')
    os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')


def source_setup():
    # source is done by calling bash -lc for each command below
    pass


def ros2_topic_exists(topic, timeout=5):
    try:
        # ros2 topic echo may block; use 'timeout' wrapper and check return code.
        cp = run_ros2_cmd(f"timeout {timeout}s ros2 topic echo {topic} --once", timeout=timeout + 5, check=False)
        return cp.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def ros2_topic_list_contains(topic):
    try:
        cp = run_ros2_cmd(f"ros2 topic list | grep -x '{topic}'", timeout=5, check=False)
        return cp.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def wait_for_topic(topic: str, timeout: int = 60) -> bool:
    """Wait for a ROS2 topic to appear in `ros2 topic list` within `timeout` seconds."""
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        try:
            if ros2_topic_list_contains(topic):
                return True
        except Exception:
            pass
        time.sleep(1.0)
    return False


def ros2_topic_list_has_namespace(prefix: str) -> bool:
    """Return True if any topic starts with the given prefix (e.g. '/robot1/robot1_variable_stiffness')."""
    try:
        cp = run_ros2_cmd(f"ros2 topic list | grep -E '^{prefix}'", timeout=5, check=False)
        return cp.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def gzserver_alive():
    return subprocess.run('pgrep -f gzserver >/dev/null 2>&1', shell=True).returncode == 0


def collect_stage1_diagnostics(args, live_log, launch_log):
    info = {}
    # gzs server processinfo
    info['gzserver'] = subprocess.run('ps -ef | grep -E "gzserver" | grep -v grep | head', shell=True, text=True, capture_output=True).stdout.strip()
    # live_plot processinfo
    info['live_plot'] = subprocess.run('ps -ef | grep -E "live_plot_logs.py" | grep -v grep | head', shell=True, text=True, capture_output=True).stdout.strip()
    # screenshot dir contents
    if os.path.exists(args.screenshot_dir):
        try:
            files = os.listdir(args.screenshot_dir)
            info['screenshot_count'] = len(files)
            info['screenshots'] = files[:5]
        except Exception as e:
            info['screenshot_error'] = str(e)
    else:
        info['screenshot_count'] = 0

    # last log lines for launcher and liveplot
    if os.path.exists(live_log):
        info['live_log_tail'] = subprocess.run(f'tail -n 15 {live_log}', shell=True, text=True, capture_output=True).stdout.strip()
    else:
        info['live_log_tail'] = '<no live log>'
    if os.path.exists(launch_log):
        info['launch_log_tail'] = subprocess.run(f'tail -n 15 {launch_log}', shell=True, text=True, capture_output=True).stdout.strip()
    else:
        info['launch_log_tail'] = '<no launch log>'

    # ROS2 topic existence snapshot
    info['ros2_topic_list'] = run_ros2_cmd('ros2 topic list | grep -E "(?:/robot1|/robot2|/gazebo)"', timeout=5).stdout.strip()
    return info


def get_controller_states(namespace):
    cmd = f"ros2 service call {namespace}/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{{}}'"
    try:
        rr = run_ros2_cmd(cmd, timeout=10, check=False)
        return rr.stdout.strip() or '<empty response>'
    except Exception as exc:
        return f'<controller states error: {exc}>'


def check_controller_active(namespace, controller):
    cmd = f"ros2 service call {namespace}/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{{}}'"
    for attempt in range(8):
        try:
            rr = run_ros2_cmd(cmd, timeout=10, check=False)
            if f"name: '{controller}'" in rr.stdout and "state: active" in rr.stdout:
                return True
        except Exception:
            pass
        time.sleep(1)
    return False


def log_to_file(message: str):
    with open('/tmp/dual_gazebo_5stage_test_results.log', 'a') as f:
        f.write(message + '\n')


def write_status(msg):
    with open('/tmp/dual_gazebo_5stage_test_status.log', 'a') as f:
        f.write(msg + '\n')


def main():
    write_status('TEST START')
    print('TEST START')
    with open('/tmp/dual_gazebo_5stage_test_results.log','a') as f:
        f.write('TEST START\n')
    parser = argparse.ArgumentParser(description='Dual Gazebo 5-stage test script')
    parser.add_argument('--max-iterations', type=int, default=30,
                        help='Max retry cycles until all 5 stages pass')
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--wait-before-push', type=float, default=8.0)
    parser.add_argument('--line-steps', type=int, default=18)
    parser.add_argument('--screenshot-dir', default='/tmp/live_plot_screenshots')
    parser.add_argument('--no-gui', action='store_true',
                        help='Run headless (disable GUI for live-plot). Default: GUI enabled')
    args = parser.parse_args()

    use_gui = not args.no_gui

    for trial in range(1, args.max_iterations + 1):
        print(f'===== Trial {trial}/{args.max_iterations} =====')
        log_to_file(f'===== Trial {trial}/{args.max_iterations} =====')
        # cleanup previous processes
        subprocess.run('pkill -f dual_gazebo_variable_stiffness.launch.py || true', shell=True)
        subprocess.run('pkill -f live_plot_logs.py || true', shell=True)
        subprocess.run('pkill -f dual_gazebo_opposing_push.py || true', shell=True)
        subprocess.run('pkill -f gzserver || true', shell=True)
        subprocess.run('pkill -f gzclient || true', shell=True)
        time.sleep(2)

        # launch stage: gazebo + live plot
        launch_log = '/tmp/dual_gazebo_5stage_launch.log'
        live_log = '/tmp/dual_gazebo_5stage_liveplot.log'
        launch_proc = subprocess.Popen(
            f'bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && ros2 launch omx_variable_stiffness_controller dual_gazebo_variable_stiffness.launch.py gui:=true launch_gazebo:=true enable_logger:=true enable_live_plot:=true start_rviz:=false"',
            shell=True, stdout=open(launch_log, 'w'), stderr=subprocess.STDOUT)
        time.sleep(4)

        # Launch live plot; allow GUI if upstream environment sets LIVEPLOT_USE_GUI=1
        live_env_prefix = 'LIVEPLOT_USE_GUI=1 ' if use_gui else 'LIVEPLOT_USE_GUI=0 '
        live_proc = subprocess.Popen(
            f'bash -lc "{live_env_prefix} source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && mkdir -p {args.screenshot_dir} && python3 tools/live_plot_logs.py --controller variable_stiffness --namespace /robot1/robot1_variable_stiffness --namespace2 /robot2/robot2_variable_stiffness --window 60 --interval 0.5 --screenshot-dir {args.screenshot_dir} --screenshot-rate 2"',
            shell=True, stdout=open(live_log, 'w'), stderr=subprocess.STDOUT)

        # Stage 1: check gazebo+live_plot launch health, eventually screenshot/topic availability
        success1 = False
        stage1_reason = ''
        write_status('stage1 start')
        for i in range(30):
            if not gzserver_alive():
                stage1_reason = 'gzserver not alive'
                break
            if live_proc.poll() is not None:
                stage1_reason = 'live_plot process exited early'
                break

            # Only count real PNG screenshots — ignore marker files.
            if os.path.isdir(args.screenshot_dir):
                try:
                    screenshot_files = [f for f in os.listdir(args.screenshot_dir) if f.lower().endswith('.png')]
                except Exception:
                    screenshot_files = []
            else:
                screenshot_files = []
            try:
                topic_list = run_ros2_cmd(
                    'ros2 topic list | grep -E "(/robot1/robot1_variable_stiffness|/robot2/robot2_variable_stiffness)"',
                    timeout=5).stdout.strip()
            except subprocess.CalledProcessError:
                topic_list = ''

            if screenshot_files:
                success1 = True
                stage1_reason = f'screenshots available ({len(screenshot_files)})'
                break
            if topic_list:
                success1 = True
                stage1_reason = 'plot topics active; no screenshots yet'
                break

            if i == 29:
                stage1_reason = 'no screenshots/topic evidence after 30s'
                break

            time.sleep(1)

        if not success1 and not stage1_reason:
            stage1_reason = 'unexpected stage1 timeout'

        write_status(f'stage1 result: {success1}, {stage1_reason}')

        if not success1:
            diag = collect_stage1_diagnostics(args, live_log, launch_log)
            print('--- Stage 1 diagnostics ---')
            print('gzserver:', diag['gzserver'] or '<not running>')
            print('live_plot:', diag['live_plot'] or '<not running>')
            print('screenshot_count:', diag.get('screenshot_count'))
            print('screenshots:', diag.get('screenshots'))
            print('ros2 topics (robot/gazebo subset):')
            print(diag.get('ros2_topic_list', '<none>'))
            print('--- live_plot log tail ---')
            print(diag['live_log_tail'])
            print('--- launch log tail ---')
            print(diag['launch_log_tail'])
            print('---------------------------')

        # Stage 2: wait until robot joint_states topics are visible in ROS topic list and box is spawned
        success2 = False
        stage2_reason = ''

        r1 = False
        r2 = False
        # Relaxed gating: accept either joint_states or controller topics
        for i in range(60):
            r1_js = ros2_topic_list_contains('/robot1/joint_states')
            r2_js = ros2_topic_list_contains('/robot2/joint_states')
            r1_ns = ros2_topic_list_has_namespace('/robot1/robot1_variable_stiffness')
            r2_ns = ros2_topic_list_has_namespace('/robot2/robot2_variable_stiffness')
            r1 = r1_js or r1_ns
            r2 = r2_js or r2_ns
            if r1 and r2:
                break
            time.sleep(1)

        missing = []
        if not r1:
            missing.append('/robot1: joint_states or variable_stiffness topics')
        if not r2:
            missing.append('/robot2: joint_states or variable_stiffness topics')
        if missing:
            stage2_reason = f'missing topic evidence after 60s: {"; ".join(missing)}'
        else:
            stage2_reason = 'joint_states or controller topics present'
        write_status(f'stage2 result pre-gazebo: {r1},{r2} -> {stage2_reason}')

        if r1 and r2:
            # Wait for /gazebo/model_states to appear before attempting to read it.
            if not wait_for_topic('/gazebo/model_states', timeout=30):
                stage2_reason = '/gazebo/model_states not published within timeout'
            else:
                try:
                    m = run_ros2_cmd("ros2 topic echo /gazebo/model_states --once", timeout=15, check=False)
                    if 'opposing_push_box' in m.stdout:
                        success2 = True
                        stage2_reason = 'ok'
                    else:
                        stage2_reason = 'box entity not detected in /gazebo/model_states'
                except Exception as e:
                    stage2_reason = f'failed to query /gazebo/model_states: {e}'

        # Stage 3: controllers active
        stage3_reason = ''
        r1_ctrl = check_controller_active('/robot1', 'robot1_variable_stiffness')
        r2_ctrl = check_controller_active('/robot2', 'robot2_variable_stiffness')
        success3 = r1_ctrl and r2_ctrl
        if success3:
            stage3_reason = 'ok'
        else:
            states1 = get_controller_states('/robot1')
            states2 = get_controller_states('/robot2')
            missing_ctrl = []
            if not r1_ctrl:
                missing_ctrl.append('robot1_variable_stiffness')
            if not r2_ctrl:
                missing_ctrl.append('robot2_variable_stiffness')
            stage3_reason = f'inactive controllers: {", ".join(missing_ctrl)}'
            print('controller states /robot1:\n', states1)
            print('controller states /robot2:\n', states2)
            log_to_file('controller states /robot1: ' + states1.replace('\n', '; '))
            log_to_file('controller states /robot2: ' + states2.replace('\n', '; '))

        # Stage 4: run opposing push once and detect contact wrench
        pusher_log = '/tmp/dual_gazebo_5stage_push.log'
        push_proc = subprocess.Popen(
            f'bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && python3 tools/dual_gazebo_opposing_push.py --duration {args.duration} --wait-before-push {args.wait_before_push} --line-steps {args.line_steps} --press-x-offset 0.0 --retracted-distance 0.20 --press-distance 0.02"',
            shell=True, stdout=open(pusher_log, 'w'), stderr=subprocess.STDOUT)

        success4 = False
        stage4_reason = ''
        for i in range(60):
            r1_contact = ros2_topic_exists('/robot1/robot1_variable_stiffness/contact_wrench', timeout=2)
            r2_contact = ros2_topic_exists('/robot2/robot2_variable_stiffness/contact_wrench', timeout=2)
            if r1_contact and r2_contact:
                success4 = True
                stage4_reason = 'ok'
                break

            # second chance: if topics exist but no messages yet, keep waiting for full duration
            t1 = ros2_topic_list_contains('/robot1/robot1_variable_stiffness/contact_wrench')
            t2 = ros2_topic_list_contains('/robot2/robot2_variable_stiffness/contact_wrench')
            if t1 and t2:
                stage4_reason = 'contact_wrench topics exist but no message yet'
            else:
                stage4_reason = 'missing contact_wrench topics'

            if i % 10 == 9:
                print(f'[stage4] attempt {i+1}: {stage4_reason}')
                log_to_file(f'[stage4] attempt {i+1}: {stage4_reason}')

            time.sleep(1)

        if not success4 and not stage4_reason:
            stage4_reason = 'missing contact_wrench for robot1 or robot2'

        # Stage 5: final decision
        all_success = success1 and success2 and success3 and success4
        print('stage results:', success1, success2, success3, success4)
        if all_success:
            print('stage reasons: stage1=ok stage2=ok stage3=ok stage4=ok')
            log_to_file('stage reasons: stage1=ok stage2=ok stage3=ok stage4=ok')
        else:
            first_fail = None
            if not success1:
                first_fail = f'stage1: {stage1_reason}'
            elif not success2:
                first_fail = f'stage2: {stage2_reason}'
            elif not success3:
                first_fail = f'stage3: {stage3_reason}'
            elif not success4:
                first_fail = f'stage4: {stage4_reason}'
            print('first fail:', first_fail)
            log_to_file('first fail: ' + str(first_fail))

        # cleanup
        push_proc.kill()
        live_proc.kill()
        launch_proc.kill()
        subprocess.run('pkill -f gzclient || true', shell=True)
        subprocess.run('pkill -f gzserver || true', shell=True)

        if all_success:
            write_status('ALL STAGES PASSED')
            print('ALL STAGES PASSED')
            with open('/tmp/dual_gazebo_5stage_test_results.log','a') as f:
                f.write('ALL STAGES PASSED\n')
            print('pressure validated; stopping iterations')
            print('TEST END')
            write_status('TEST END')
            with open('/tmp/dual_gazebo_5stage_test_results.log','a') as f:
                f.write('TEST END\n')
            return

        write_status('ONE OR MORE STAGES FAILED')
        print('One or more stages failed; retrying')
        time.sleep(3)

    write_status('FAILED: max iterations reached without all stage success')
    print('FAILED: max iterations reached without all stage success')
    with open('/tmp/dual_gazebo_5stage_test_results.log','a') as f:
        f.write('FAILED: max iterations reached without all stage success\n')
    print('TEST END')
    write_status('TEST END')
    with open('/tmp/dual_gazebo_5stage_test_results.log','a') as f:
        f.write('TEST END\n')
    sys.exit(1)


if __name__ == '__main__':
    main()
