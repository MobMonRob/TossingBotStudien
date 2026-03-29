#!/usr/bin/env python3
import rclpy
import argparse
import sys
sys.path.insert(0, '/home/s/tossingbot_ws/install/ur5_rg2_ign/lib/ur5_rg2_ign')
from ur5e_base_controller import UR5eBaseController

WAYPOINTS = [
    {'positions': [0.77,   -43.34,  92.68,  36.16, 90.4,   0], 'wait_time': 2.0, 'duration': 1.5},
    {'positions': [0,      -70.62,  83.67,  0,     99.70,  0], 'wait_time': 1.0, 'duration': 1.5},
    {'positions': [182.48, -50.67, 103.69, 13.88, 100.40,  0], 'wait_time': 1.0, 'duration': 1.5},
    {'positions': [26.16,  -50.06,  42.96, 13.88, 100.40,  0], 'wait_time': 1.2, 'duration': 0.5},
]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip',   default='192.168.12.136')
    parser.add_argument('--port', type=int, default=502)
    args = parser.parse_args()

    rclpy.init()
    node = UR5eBaseController('wurf_seite_zeitgesteuert',
                               gripper_ip=args.ip, gripper_port=args.port)
    node.execute_waypoint_sequence(WAYPOINTS)
    node.close_connections()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
