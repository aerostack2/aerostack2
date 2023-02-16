"""
test_gps.py
"""

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

__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import rclpy
from as2_python_api.drone_interface_gps import DroneInterfaceGPS as DroneInterface
from geographic_msgs.msg import GeoPath, GeoPoseStamped


rclpy.init()

uav1 = DroneInterface('drone_sim_0', verbose=True)

print("ready")

print("Origin:", uav1.gps.origin)

uav1.gps.set_origin([10.0, 20.0, 30.0])

print("Origin:", uav1.gps.origin)

uav1.gps.set_origin([40.0, 50.0, 60.0])

print("Origin:", uav1.gps.origin)

# uav1.go_to.go_to_gps(10, 20, 30, 3.0)

path = GeoPath()
p1 = GeoPoseStamped()
p1.pose.position.latitude = 10.0
p1.pose.position.longitude = 20.0
p1.pose.position.altitude = 30.0
path.poses.append(p1)
path.poses.append(p1)
uav1.follow_path(path, 2.0)

rclpy.shutdown()
