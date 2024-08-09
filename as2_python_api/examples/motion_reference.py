#!/bin/python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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
