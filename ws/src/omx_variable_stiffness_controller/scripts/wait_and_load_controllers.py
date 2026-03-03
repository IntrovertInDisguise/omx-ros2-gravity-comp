#!/usr/bin/env python3
"""
Wait for controller_manager services, load controllers, then switch them to active.

Usage:
  python3 wait_and_load_controllers.py --ns /omx joint_state_broadcaster variable_stiffness_controller

This script retries service calls and prints results for debugging launch ordering issues.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers, ListControllerTypes


class LoaderNode(Node):
    def __init__(self, namespace, controllers, timeout=10.0, retries=10):
        super().__init__('wait_and_load_controllers')
        self.raw_namespace = namespace.rstrip('/')
        # Ensure we target the controller_manager namespace (e.g. /omx/controller_manager)
        if self.raw_namespace.endswith('controller_manager'):
            self.manager_ns = self.raw_namespace
        else:
            self.manager_ns = f'{self.raw_namespace}/controller_manager'
        self.controllers = controllers
        self.timeout = timeout
        self.retries = retries
        self.load_srv_name = f'{self.manager_ns}/load_controller'
        self.switch_srv_name = f'{self.manager_ns}/switch_controller'

    def wait_for_service(self, srv_name, srv_type):
        tries = 0
        client = self.create_client(srv_type, srv_name)
        while rclpy.ok() and tries < self.retries:
            self.get_logger().info(f'Checking service {srv_name} (attempt {tries+1}/{self.retries})')
            if client.service_is_ready():
                return True
            time.sleep(1.0)
            tries += 1
        return False

    def call_load(self, client, controller_name):
        req = LoadController.Request()
        req.name = controller_name
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.timeout)
        if fut.done():
            return fut.result()
        return None

    def call_list(self, srv_name, srv_type):
        client = self.create_client(srv_type, srv_name)
        req = srv_type.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.timeout)
        if fut.done():
            return fut.result()
        return None

    def call_switch(self, client, start_ctrls):
        req = SwitchController.Request()
        req.start_controllers = start_ctrls
        req.stop_controllers = []
        # 1 = BEST_EFFORT, 2 = STRICT
        req.strictness = 1
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.timeout)
        if fut.done():
            return fut.result()
        return None

    def run(self):
        # Load each controller
        if not self.wait_for_service(self.load_srv_name, LoadController):
            self.get_logger().error(f'Service {self.load_srv_name} not available after retries')
            return 2
        load_client = self.create_client(LoadController, self.load_srv_name)

        # Diagnostic: list existing controllers and controller types
        try:
            types = self.call_list(f'{self.manager_ns}/list_controller_types', ListControllerTypes)
            if types is not None:
                self.get_logger().info(f'Controller types: {getattr(types, "types", types)}')
            ctrls = self.call_list(f'{self.manager_ns}/list_controllers', ListControllers)
            if ctrls is not None:
                names = [c.name for c in getattr(ctrls, 'controllers', [])]
                self.get_logger().info(f'Currently loaded controllers: {names}')
        except Exception as e:
            self.get_logger().warn(f'Unable to query controller manager lists: {e}')

        for c in self.controllers:
            self.get_logger().info(f'Loading controller: {c}')
            resp = self.call_load(load_client, c)
            if resp is None:
                self.get_logger().warn(f'No response loading {c}')
            else:
                self.get_logger().info(f'Load response for {c}: {resp.ok}')

        # Switch/start controllers
        if not self.wait_for_service(self.switch_srv_name, SwitchController):
            self.get_logger().error(f'Service {self.switch_srv_name} not available after retries')
            return 3
        switch_client = self.create_client(SwitchController, self.switch_srv_name)
        self.get_logger().info(f'Switching controllers active: {self.controllers}')
        resp = self.call_switch(switch_client, self.controllers)
        if resp is None:
            self.get_logger().warn('No response from switch_controller')
        else:
            self.get_logger().info(f'Switch response: {resp.ok}')
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Wait & load controllers helper')
    parser.add_argument('--ns', type=str, default='/omx', help='controller_manager namespace')
    parser.add_argument('--timeout', type=float, default=10.0, help='service call timeout (s)')
    parser.add_argument('--retries', type=int, default=30, help='service availability retries')
    parser.add_argument('controllers', nargs='+', help='controllers to load+activate')
    args = parser.parse_args(argv)

    rclpy.init()
    node = LoaderNode(args.ns, args.controllers, timeout=args.timeout, retries=args.retries)
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(rc)


if __name__ == '__main__':
    main()
