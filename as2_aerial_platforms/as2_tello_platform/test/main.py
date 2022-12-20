"""Tello tests"""
#!/bin/python3

import sys
import os
from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface


def arm_test(uav: DroneInterface):
    """Arm test"""
    uav.offboard()
    print("Offboard")
    sleep(1)

    uav.arm()
    print("Armed")
    sleep(1)


def takeoff_land_test(uav: DroneInterface):
    """Takeoff and land"""
    uav.takeoff()
    sleep(5)

    uav.land()


def command_pose_test(uav: DroneInterface):
    """Pose test"""
    uav.takeoff()
    sleep(2)

    uav.send_motion_reference_pose(position=[0, 0, 0.5])
    sleep(5)

    uav.send_motion_reference_pose(position=[0, 0, 1.0])
    sleep(5)

    uav.send_motion_reference_pose(position=[0, 0, 0.3])
    sleep(5)
    uav.land()


if __name__ == '__main__':
    rclpy.init()

    drone_id = "tello"
    print("Connecting to: ", drone_id)
    drone = DroneInterface(drone_id, True)

    takeoff_land_test(drone)

    drone.shutdown()

    print("Bye")

    rclpy.shutdown()
