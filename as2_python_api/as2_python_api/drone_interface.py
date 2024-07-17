"""Drone interface to easily command drones with Aerostack2."""

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

__authors__ = 'Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.modules.follow_path_module import FollowPathModule
from as2_python_api.modules.go_to_module import GoToModule
from as2_python_api.modules.land_module import LandModule
from as2_python_api.modules.takeoff_module import TakeoffModule


class DroneInterface(DroneInterfaceBase):
    """Drone interface node."""

    def __init__(self, drone_id: str = 'drone0', verbose: bool = False,
                 use_sim_time: bool = False, spin_rate: float = 20.0) -> None:
        """
        Construct method.

        :param drone_id: drone namespace, defaults to "drone0"
        :type drone_id: str, optional
        :param verbose: output mode, defaults to False
        :type verbose: bool, optional
        :param use_sim_time: use simulation time, defaults to False
        :type use_sim_time: bool, optional
        :param spin_rate: spin rate (Hz), defaults to 20
        :type spin_rate: float, optional
        """
        super().__init__(drone_id=drone_id, verbose=verbose,
                         use_sim_time=use_sim_time, spin_rate=spin_rate)

        self.takeoff = TakeoffModule(drone=self)
        self.go_to = GoToModule(drone=self)
        self.follow_path = FollowPathModule(drone=self)
        self.land = LandModule(drone=self)
