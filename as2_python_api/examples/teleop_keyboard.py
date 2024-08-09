#!/usr/bin/env python

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


import rclpy
import time
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface

RATE = 0.1
LIN_VEL = 0.1
ANG_VEL = 0.1
msg = """
    Teleoperate Target!
---------------------------
Take Off:
    tk
Land:
    la
Moving around:
        w                i
   a    s    d      j    k    l
        x                ,
w/x : increase/decrease linear Z velocity (~ 0.1)
a/d : increase/decrease angular velocity (~ 0.1)
i/, : increase/decrease linear X velocity (~ 0.1)
j/l : increase/decrease linear Y velocity (~ 0.1)
space key, s, k : force stop
CTRL-D to quit
"""


def print_status(vx, vy, vz, az):
    print("\n\t   STATUS")
    print("---------------------------")
    print("%s\t%s\t%s\t%s\n" % (str(vx), str(vy), str(vz), str(az)))


if __name__ == "__main__":
    rclpy.init()

    drone = DroneInterface("drone_sim_0", verbose=True)
    drone.offboard()
    drone.arm()

    vx, vy, vz, az = 0.0, 0.0, 0.0, 0.0

    while rclpy.ok():
        try:
            cmd = input(msg).lower()
            if cmd == "tk":
                # drone.takeoff()
                continue
            elif cmd == "la":
                # drone.land()
                continue
            elif cmd == "w":
                vz += LIN_VEL
            elif cmd == "a":
                az += ANG_VEL
            elif cmd == "d":
                az -= ANG_VEL
            elif cmd == "x":
                vz -= LIN_VEL
            elif cmd == "i":
                vx += LIN_VEL
            elif cmd == "j":
                vy -= LIN_VEL
            elif cmd == "l":
                vy += LIN_VEL
            elif cmd == ",":
                vx -= LIN_VEL
            elif cmd == "s" or "k" or " ":
                vx, vy, vz, az = 0.0, 0.0, 0.0, 0.0
            elif cmd == "00":
                vx, vy, vz, az = 0.0, 0.0, 0.0, 0.0
            else:
                print("[Error] Invalid command")
        except EOFError:
            break
        else:
            print_status(vx, vy, vz, az)
            drone.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
                [vx, vy, vz], twist_frame_id='base_link', yaw_speed=az)

        time.sleep(RATE)

    drone.get_logger().info("BYE")
    time.sleep(1)
