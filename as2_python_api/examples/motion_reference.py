#!/bin/python3

import os
from time import sleep
import rclpy
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop

sleep_time = 2.0

rclpy.init()
uav_name = os.environ['AEROSTACK2_SIMULATION_DRONE_ID']
drone_interface = DroneInterfaceTeleop(uav_name, verbose=True)

drone_interface.offboard()
drone_interface.arm()

##### TAKE OFF #####
print("Take Off")
drone_interface.takeoff(1.0, speed=1.0)
print("Take Off done")
sleep(sleep_time)

##### HOVER #####
print("Hovering")
drone_interface.motion_ref_handler.hover()
print("Hovering done")
sleep(sleep_time)

##### POSITION #####
print("Position")
drone_interface.motion_ref_handler.position.send_position_command_with_yaw_angle(
    [1.0, 0.0, 1.0], 1.0, 'earth', 'earth', 0.0)
print("Position done")
sleep(sleep_time)

##### SPEED #####
print("Speed")
drone_interface.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
    [1.0, 0.0, 0.0], 'earth', 0.0)
print("Speed done")
sleep(sleep_time)

##### SPEED IN A PLANE #####
print("Speed in a plane")
drone_interface.motion_ref_handler.speed_in_a_plane.send_speed_in_a_plane_command_with_yaw_speed(
    [1.0, 0.0, 0.0], 2.0, 'earth', 'earth', 0.0)
print("Speed in a plane done")
sleep(sleep_time)

##### LAND #####
print("Landing")
drone_interface.land(speed=0.5)
print("Land done")

drone_interface.shutdown()
rclpy.shutdown()
print("Clean exit")
exit(0)
