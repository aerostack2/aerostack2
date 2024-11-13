"""Mission runner."""

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
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
from importlib.util import module_from_spec, spec_from_file_location
import sys

from as2_python_api.drone_interface import DroneInterface
import rclpy


def main():
    args = sys.argv[1:]
    if '--ros-args' in args:
        args = args[:args.index('--ros-args')]

    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', type=str, default='drone0', help='Namespace')
    parser.add_argument('--use_sim_time', type=bool, default='false', help='Use sim time')
    parser.add_argument('--log-level', type=str, default='info', help='Use verbose')
    parser.add_argument('--spin-rate', type=int, default=20, help='Spin rate')

    argument_parser = parser.parse_args(args)
    verbose = argument_parser.log_level == 'debug'

    rclpy.init()

    uav = DroneInterface(drone_id=argument_parser.ns, use_sim_time=argument_parser.use_sim_time,
                         verbose=verbose, spin_rate=argument_parser.spin_rate)
    uav.declare_parameter('mission_path', rclpy.Parameter.Type.STRING)
    uav.declare_parameter('mission_name', rclpy.Parameter.Type.STRING)

    module_path = uav.get_parameter('mission_path').get_parameter_value().string_value
    module_name = uav.get_parameter('mission_name').get_parameter_value().string_value

    uav.get_logger().info(f'Loading mission from {module_path}/{module_name}.py')
    spec = spec_from_file_location(module_name, module_path + f'/{module_name}.py')
    module = module_from_spec(spec)  # get module from spec
    spec.loader.exec_module(module)  # load module

    drone_run = getattr(module, 'drone_run')
    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
