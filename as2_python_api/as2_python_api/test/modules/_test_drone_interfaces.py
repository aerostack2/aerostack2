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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from as2_python_api.drone_interface import DroneInterface
from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop
import rclpy


rclpy.init()

drone_interface = DroneInterfaceBase('drone_sim_0', verbose=False)
print(drone_interface.modules)
assert len(drone_interface.modules) == 0, 'Unexpected number of modules found'

print('Exit')
drone_interface.shutdown()
print(drone_interface.modules)
assert len(drone_interface.modules) == 0, 'Unexpected number of modules found'


drone_interface = DroneInterface('drone_sim_1', verbose=False)
print(drone_interface.modules)
assert len(drone_interface.modules) == 4, 'Unexpected number of modules found'

print('Exit')
drone_interface.shutdown()
print(drone_interface.modules)
assert len(drone_interface.modules) == 0, 'Unexpected number of modules found'


drone_interface = DroneInterfaceGPS('drone_sim_2', verbose=False)
print(drone_interface.modules)
assert len(drone_interface.modules) == 5, 'Unexpected number of modules found'

print('Exit')
drone_interface.shutdown()
print(drone_interface.modules)
assert len(drone_interface.modules) == 0, 'Unexpected number of modules found'


drone_interface = DroneInterfaceTeleop('drone_sim_3', verbose=False)
print(drone_interface.modules)
assert len(drone_interface.modules) == 5, 'Unexpected number of modules found'

print('Exit')
drone_interface.shutdown()
print(drone_interface.modules)
assert len(drone_interface.modules) == 0, 'Unexpected number of modules found'

rclpy.shutdown()
