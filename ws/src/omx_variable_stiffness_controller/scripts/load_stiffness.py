#!/usr/bin/env python3
"""
Stiffness Profile Loader for Variable Stiffness Controller.

This node reads stiffness/damping profiles from a CSV file and either:
1. Sets them as parameters on the controller, or
2. Publishes them to the controller's stiffness_profile_update topic.

CSV format expected:
s,Kx,Ky,Kz,Dx,Dy,Dz
0.0,200.0,200.0,200.0,20.0,20.0,20.0
...
1.0,200.0,200.0,200.0,20.0,20.0,20.0

"""

import csv
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float64MultiArray


class StiffnessLoader(Node):
    def __init__(self):
        super().__init__('stiffness_loader')

        # Declare parameters
        self.declare_parameter('csv_path', '')
        self.declare_parameter('controller_name', 'variable_stiffness_controller')
        self.declare_parameter('use_parameter_service', True)
        self.declare_parameter('publish_rate', 0.0)  # 0 = one-shot, >0 = periodic Hz

        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        self.use_param_service = self.get_parameter('use_parameter_service').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        if not self.csv_path:
            self.get_logger().warn('No csv_path specified. Stiffness loader will not load any profiles.')
            return

        # Load profiles from CSV
        self.stiffness_x = []
        self.stiffness_y = []
        self.stiffness_z = []
        self.damping_x = []
        self.damping_y = []
        self.damping_z = []

        if not self.load_csv():
            return

        if self.use_param_service:
            # Use parameter service to set profiles
            self.set_parameters_via_service()
        else:
            # Use topic to publish profiles
            self.profile_pub = self.create_publisher(
                Float64MultiArray,
                f'{self.controller_name}/stiffness_profile_update',
                10
            )

            if self.publish_rate > 0:
                period = 1.0 / self.publish_rate
                self.timer = self.create_timer(period, self.publish_profiles)
            else:
                # One-shot publish after a short delay
                self.timer = self.create_timer(1.0, self.publish_profiles_once)

    def load_csv(self) -> bool:
        """Load stiffness/damping profiles from CSV file."""
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.stiffness_x.append(float(row['Kx']))
                    self.stiffness_y.append(float(row['Ky']))
                    self.stiffness_z.append(float(row['Kz']))
                    self.damping_x.append(float(row['Dx']))
                    self.damping_y.append(float(row['Dy']))
                    self.damping_z.append(float(row['Dz']))

            self.get_logger().info(
                f'Loaded {len(self.stiffness_x)} profile points from {self.csv_path}'
            )
            return True

        except FileNotFoundError:
            self.get_logger().error(f'CSV file not found: {self.csv_path}')
            return False
        except KeyError as e:
            self.get_logger().error(f'Missing column in CSV: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error loading CSV: {e}')
            return False

    def set_parameters_via_service(self):
        """Set stiffness profiles via the controller's parameter service."""
        # Get namespace from node
        ns = self.get_namespace()
        if ns == '/':
            ns = ''

        service_name = f'{ns}/{self.controller_name}/set_parameters'
        self.get_logger().info(f'Waiting for service: {service_name}')

        client = self.create_client(SetParameters, service_name)

        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service {service_name} not available')
            return

        # Build parameter messages
        params = []

        # Stiffness profiles
        for name, values in [
            ('stiffness_profile_x', self.stiffness_x),
            ('stiffness_profile_y', self.stiffness_y),
            ('stiffness_profile_z', self.stiffness_z),
            ('damping_profile_x', self.damping_x),
            ('damping_profile_y', self.damping_y),
            ('damping_profile_z', self.damping_z),
        ]:
            param = Parameter()
            param.name = name
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            param.value.double_array_value = values
            params.append(param)

        request = SetParameters.Request()
        request.parameters = params

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            results = future.result().results
            success = all(r.successful for r in results)
            if success:
                self.get_logger().info('Successfully set stiffness/damping profile parameters')
            else:
                for i, r in enumerate(results):
                    if not r.successful:
                        self.get_logger().error(f'Failed to set parameter {i}: {r.reason}')
        else:
            self.get_logger().error('Service call failed')

    def publish_profiles(self):
        """Publish stiffness profiles to topic."""
        msg = Float64MultiArray()

        # Format: [n, kx_0, kx_1, ..., ky_0, ky_1, ..., kz_0, kz_1, ...]
        n = len(self.stiffness_x)
        msg.data = [float(n)] + self.stiffness_x + self.stiffness_y + self.stiffness_z

        self.profile_pub.publish(msg)
        self.get_logger().debug(f'Published stiffness profile with {n} points')

    def publish_profiles_once(self):
        """Publish profiles once and stop the timer."""
        self.publish_profiles()
        self.timer.cancel()
        self.get_logger().info('Published stiffness profiles (one-shot)')


def main(args=None):
    rclpy.init(args=args)
    node = StiffnessLoader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
