"""Profiling test."""

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
import rclpy


def loading_mod():
    module_takeoff = 'as2_python_api.modules.takeoff_module'
    module_land = 'as2_python_api.modules.land_module'
    module_goto = 'as2_python_api.modules.go_to_module'
    module_follow_path = 'as2_python_api.modules.follow_path_module'

    drone_interface = DroneInterfaceBase('drone_sim_0', verbose=True)

    drone_interface.load_module(module_takeoff)
    drone_interface.load_module(module_land)
    drone_interface.load_module(module_goto)
    drone_interface.load_module(module_follow_path)
    print(drone_interface.modules)

    drone_interface.shutdown()


def preloaded_mod():
    drone_interface = DroneInterface('drone_sim_0', verbose=True)
    print(drone_interface.modules)

    drone_interface.shutdown()


def main():
    import cProfile
    import pstats

    with cProfile.Profile() as pr:
        # loading_mod()
        preloaded_mod()

    stats = pstats.Stats(pr)
    stats.sort_stats(pstats.SortKey.TIME)
    # stats.print_stats()
    stats.dump_stats(filename='needs_profiling.prof')


if __name__ == '__main__':
    rclpy.init()
    main()
