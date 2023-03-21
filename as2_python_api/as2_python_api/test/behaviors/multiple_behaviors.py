
# Copyright 2022 Universidad Politécnica de Madrid
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
#    * Neither the name of the the copyright holder nor the names of its
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


__authors__ = "Pedro Arias Pérez, Miguel Fernández Cortizas, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import time
from enum import Enum

import rclpy

from as2_python_api.drone_interface import DroneInterface


class BehaviorStatus(Enum):
    IDLE = 0
    RUNNING = 1
    PAUSED = 2


rclpy.init()

drone_interface = DroneInterface("", verbose=False)
drone_interface_2 = DroneInterface("", verbose=False)

time.sleep(1)

print("STATUS:", BehaviorStatus(drone_interface.takeoff.status).name)
print("STATUS:", BehaviorStatus(drone_interface_2.takeoff.status).name)

print("START")
drone_interface.takeoff.start(1.0, 0.5, False)

time.sleep(2)
print("STATUS:", BehaviorStatus(drone_interface.takeoff.status).name)
print("STATUS:", BehaviorStatus(drone_interface_2.takeoff.status).name)

print("PAUSE")
drone_interface_2.takeoff.pause()

time.sleep(2)
print("STATUS:", BehaviorStatus(drone_interface.takeoff.status).name)
print("STATUS:", BehaviorStatus(drone_interface_2.takeoff.status).name)

print("RESUME")
drone_interface.takeoff.resume(False)
time.sleep(2)
print("STATUS:", BehaviorStatus(drone_interface.takeoff.status).name)
print("STATUS:", BehaviorStatus(drone_interface_2.takeoff.status).name)

print("WAIT TO RESULT")
drone_interface_2.takeoff.wait_to_result()

print(drone_interface.takeoff.result)
print(drone_interface_2.takeoff.result)


print("Exit")
drone_interface.shutdown()
drone_interface_2.shutdown()
