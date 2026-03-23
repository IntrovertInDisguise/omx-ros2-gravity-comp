#!/usr/bin/env python3
import argparse
import subprocess
import sys
import time


def run_cmd(cmd, timeout=30, check=True):
    return subprocess.run(cmd, shell=True, timeout=timeout, check=check,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          text=True)


def wait_for_topic(topic, timeout):
    start = time.time()
    while time.time() - start < timeout:
        try:
            out = run_cmd(f"ros2 topic list | grep -x '{topic}'", timeout=5, check=False)
            if out.returncode == 0:
                # also wait for first message to ensure publisher active
                msg = run_cmd(f"ros2 topic echo {topic} --once --timeout 5", timeout=10, check=False)
                if msg.returncode == 0:
                    print(f"[spawn_robot] topic {topic} data available")
                    return True
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            print(f"[spawn_robot] topic check exception: {e}")
        time.sleep(1.0)
    return False


def wait_for_service(service, timeout):
    start = time.time()
    while time.time() - start < timeout:
        try:
            out = run_cmd(f"ros2 service list | grep -x '{service}'", timeout=5, check=False)
            if out.returncode == 0:
                print(f"[spawn_robot] service {service} available")
                return True
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            print(f"[spawn_robot] service check exception: {e}")
        time.sleep(1.0)
    return False


def do_spawn(robot_name, topic, entity_namespace, x, y, z):
    spawn_cmd = (
        'ros2 run gazebo_ros spawn_entity.py '
        f"-topic {topic} -entity {robot_name} -x {x} -y {y} -z {z} -robot_namespace {entity_namespace}"
    )
    print(f"[spawn_robot] executing: {spawn_cmd}")
    try:
        run_cmd(spawn_cmd, timeout=120, check=True)
        print(f"[spawn_robot] spawned {robot_name} successfully")
        return True
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip() if exc.stderr else ''
        print(f"[spawn_robot] spawn failed: {stderr}")
        if 'Entity [' in stderr and 'already exists' in stderr:
            print(f"[spawn_robot] entity {robot_name} already exists; continuing")
            return True
        return False
    except Exception as exc:
        print(f"[spawn_robot] spawn exception: {exc}")
        return False


def main():
    parser = argparse.ArgumentParser(description='Wait for condition and spawn robot in Gazebo')
    parser.add_argument('--robot', required=True, help='Robot name/entity name')
    parser.add_argument('--topic', default=None, help='ROS2 topic to wait for before spawning')
    parser.add_argument('--wait_controller', default=None, help='robot namespace for controller manager service wait')
    parser.add_argument('--timeout', type=int, default=120, help='Timeout seconds for conditions')
    parser.add_argument('--urdf_topic', default=None, help='If set, use topic path for spawn_entity (robot description topic)')
    parser.add_argument('--x', default='0.0', help='spawn x')
    parser.add_argument('--y', default='0.0', help='spawn y')
    parser.add_argument('--z', default='0.0', help='spawn z')
    parser.add_argument('--ns', default=None, help='robot namespace for spawn_entity')

    args = parser.parse_args()

    target_topic = args.urdf_topic or f"/{args.robot}/robot_description"
    robot_ns = args.ns or args.robot

    phoenix = False
    if args.topic:
        if args.topic == '/gazebo/model_states':
            print(f"[spawn_robot] waiting for gazebo services up to {args.timeout}s")
            gazebo_ready = wait_for_service('/gazebo/get_world_properties', args.timeout) or wait_for_service('/get_model_list', args.timeout)
            phoenix = gazebo_ready and wait_for_service('/spawn_entity', args.timeout)
        else:
            print(f"[spawn_robot] waiting for topic: {args.topic} up to {args.timeout}s")
            phoenix = wait_for_topic(args.topic, args.timeout)
    elif args.wait_controller:
        service_name = f"/{args.wait_controller}/controller_manager/list_controllers"
        print(f"[spawn_robot] waiting for service: {service_name} up to {args.timeout}s")
        phoenix = wait_for_service(service_name, args.timeout)
    else:
        print("[spawn_robot] error: one of --topic or --wait_controller is required")
        sys.exit(1)

    if not phoenix:
        print('[spawn_robot] error: condition not met, aborting')
        sys.exit(1)

    success = do_spawn(args.robot, target_topic, robot_ns, args.x, args.y, args.z)
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
