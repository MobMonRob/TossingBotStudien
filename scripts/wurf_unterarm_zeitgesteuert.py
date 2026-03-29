#!/usr/bin/env python3
import rclpy
import argparse
import sys
sys.path.insert(0, '/home/s/tossingbot_ws/install/ur5_rg2_ign/lib/ur5_rg2_ign')
from ur5e_base_controller import UR5eBaseController

WAYPOINTS = [
    {'positions': [0.77, -43.34,  92.68, 36.16, 90.4, 0], 'wait_time': 0, 'duration': 0.5},
    {'positions': [0,    -70.62,  83.67,  0,    99.70, 0], 'wait_time': 0, 'duration': 0.5},
    {'positions': [-3,   -68,    115,    85,    96,    0], 'wait_time': 0, 'duration': 0.5},
    {'positions': [-5,   -60,     66,    25,    80,    0], 'wait_time': 0.22, 'duration': 0.20},
]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip',   default='192.168.12.136')
    parser.add_argument('--port', type=int, default=502)
    args = parser.parse_args()

    rclpy.init()
    node = UR5eBaseController('wurf_unterarm_zeitgesteuert',
                               gripper_ip=args.ip, gripper_port=args.port)
    node.execute_waypoint_sequence(WAYPOINTS)
    node.close_connections()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
