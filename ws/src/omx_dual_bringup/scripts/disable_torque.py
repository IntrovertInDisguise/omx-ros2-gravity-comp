#!/usr/bin/env python3
"""
Disable torque on all Open Manipulator X servos before launching ros2_control.

Usage:
    python3 disable_torque.py [--port /dev/ttyUSB0] [--baud 1000000]

Servo IDs (default OMX config): 11, 12, 13, 14, 15
Torque Enable address: 64 (XM/XL series)
"""

import argparse
import sys

# Dynamixel SDK
from dynamixel_sdk import PortHandler, PacketHandler

ADDR_TORQUE_ENABLE = 64
TORQUE_DISABLE = 0
PROTOCOL_VERSION = 2.0
DXL_IDS = [11, 12, 13, 14, 15]


def main():
    parser = argparse.ArgumentParser(description='Disable torque on OMX servos')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--baud', type=int, default=1000000, help='Baud rate')
    parser.add_argument('--ids', nargs='+', type=int, default=DXL_IDS,
                        help='Servo IDs (default: 11 12 13 14 15)')
    args = parser.parse_args()

    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"ERROR: Cannot open port {args.port}")
        sys.exit(1)

    if not port_handler.setBaudRate(args.baud):
        print(f"ERROR: Cannot set baud rate {args.baud}")
        sys.exit(1)

    print(f"Port: {args.port}  Baud: {args.baud}")

    all_ok = True
    for dxl_id in args.ids:
        result, error = packet_handler.write1ByteTxRx(
            port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if result != 0:  # COMM_SUCCESS = 0
            print(f"  ID {dxl_id}: FAIL (comm result={result})")
            all_ok = False
        elif error != 0:
            print(f"  ID {dxl_id}: FAIL (dxl error={error})")
            all_ok = False
        else:
            print(f"  ID {dxl_id}: torque disabled")

    port_handler.closePort()

    if all_ok:
        print("All servos torque disabled. Safe to launch.")
    else:
        print("WARNING: Some servos did not respond. Check power/connections/IDs.")
        sys.exit(1)


if __name__ == '__main__':
    main()
