"""Keyboard Teleoperation."""

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

__authors__ = 'Javier Melero Deza, Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

import argparse
import sys
import threading
from typing import List

from as2_keyboard_teleoperation.config_values import ControlModes
from as2_keyboard_teleoperation.config_values import ControlValues
from as2_keyboard_teleoperation.drone_manager import DroneManager
from as2_keyboard_teleoperation.localization_window import LocalizationWindow
from as2_keyboard_teleoperation.main_window import MainWindow
from as2_keyboard_teleoperation.settings_window import SettingsWindow
from as2_python_api.behavior_manager.behavior_manager import SwarmBehaviorManager
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface

import PySimpleGUI as sg
import rclpy


class DroneInterfaceExtended(DroneInterface):
    """Drone interface extended with given modules."""

    def __init__(self, drone_id, verbose=False, use_sim_time=False, modules: List[str] = None,
                 spin_rate=100.0):
        super().__init__(drone_id, verbose=verbose, use_sim_time=use_sim_time,
                         spin_rate=spin_rate)
        self._logger = self.get_logger()
        self._logger.info(f'DroneInterfaceExtended created for {drone_id}')
        if modules is not None:
            try:
                for module in modules:
                    self.load_module(module)
            except ModuleNotFoundError as e:
                self._logger.error(f'ModuleNotFoundError: {e}')

        self._logger.info(f'{self.modules}')


def parse_config_values(args):
    """Parse configuration values."""
    parser = argparse.ArgumentParser(description='Keyboard Teleoperation')

    parser.add_argument('--speed_value', type=float, help='Speed value')
    parser.add_argument('--altitude_speed_value', type=float, help='Altitude speed value')
    parser.add_argument('--turn_speed_value', type=float, help='Turn speed value')
    parser.add_argument('--position_value', type=float, help='Position value')
    parser.add_argument('--altitude_value', type=float, help='Altitude value')
    parser.add_argument('--turn_angle_value', type=float, help='Turn angle value')
    parser.add_argument('--speed_frame_id', type=str, help='Speed frame id',
                        choices=['earth', 'base_link'])
    parser.add_argument('--pose_frame_id', type=str, help='Pose frame id')
    parser.add_argument('--initial_mode', type=str, help='Initial mode')
    parser.add_argument('--use_sim_time', type=str, help='Use sim time')
    parser.add_argument('--verbose', type=str, help='Verbose')
    parser.add_argument('--namespace', type=str, help='Drone id list')
    parser.add_argument('--drone_frequency', type=float,
                        help='Drone frequency', default=100.0)
    parser.add_argument('--modules', type=str, help='Modules list', default='')

    args = parser.parse_args()

    control_values = {
        'speed_value': args.speed_value,
        'altitude_speed_value': args.altitude_speed_value,
        'turn_speed_value': args.turn_speed_value,
        'position_value': args.position_value,
        'altitude_value': args.altitude_value,
        'turn_angle_value': args.turn_angle_value,
    }

    teleop_config = {
        'speed_frame_id': args.speed_frame_id,
        'pose_frame_id': args.pose_frame_id,
        'initial_mode': args.initial_mode
    }

    node_config = {
        'namespace': args.namespace,
        'use_sim_time': args.use_sim_time.lower() == 'true',
        'verbose': args.verbose.lower() == 'true',
        'drone_frequency': args.drone_frequency,
        'modules': args.modules.split(',') if args.modules != '' else None
    }

    return control_values, teleop_config, node_config


def main():
    """entrypoint."""
    config_values, teleop_config, node_config = parse_config_values(sys.argv[1:])
    uav_list = []
    drone_id = node_config['namespace']
    simulated = node_config['use_sim_time']
    verbose = node_config['verbose']
    rclpy.init()
    if ',' in drone_id:
        drone_id_list = drone_id.split(',')
        for uav_id in drone_id_list:
            uav_list.append(DroneInterfaceExtended(
                uav_id, verbose=verbose, use_sim_time=simulated, modules=node_config['modules'],
                spin_rate=node_config['drone_frequency']))
            uav_list[-1].get_logger().info(
                f'Drone {uav_id} initialized with use_sim_time={simulated} \
                and verbose={verbose}')
    else:
        uav_list.append(DroneInterfaceExtended(
            drone_id, verbose=verbose, use_sim_time=simulated, modules=node_config['modules'],
            spin_rate=node_config['drone_frequency']))
        uav_list[-1].get_logger().info(
            f'Drone {drone_id} initialized with use_sim_time={simulated} \
                and verbose={verbose}')

    k_t = KeyboardTeleoperation(uav_list, False, config_values, teleop_config)
    while k_t.execute_main_window(k_t.main_window):
        pass

    rclpy.shutdown()


class KeyboardTeleoperation:
    """
    Main class for the teleoperation.

    It initializes both the frontend and backend. It also
    execute the main loop where the Main window is executed. When the handler of the Main window
    returns an output, it calls the drone manager to perform an action.
    """

    def __init__(self, uav_list: List[DroneInterface], thread=False,
                 config_values=None,
                 teleop_config=None,
                 node_config=None):

        self.uav_list = uav_list

        drone_id_list = []

        for uav in self.uav_list:
            drone_id_list.append([uav.drone_id, True])

        if config_values is not None:
            ControlValues.initialize(**config_values)

        value_list = [
            ControlValues.SPEED_VALUE,
            ControlValues.VERTICAL_VALUE,
            ControlValues.TURN_SPEED_VALUE,
            ControlValues.POSITION_VALUE,
            ControlValues.ALTITUDE_VALUE,
            ControlValues.TURN_ANGLE_VALUE]

        self.localization_opened = False

        sg.theme('DarkBlack1')

        self.drone_manager = DroneManager(
            uav_list=self.uav_list, drone_id_list=drone_id_list,
            pose_frame_id=teleop_config['pose_frame_id'],
            twist_frame_id=teleop_config['speed_frame_id'])

        self.settings_window = SettingsWindow(
            font=(
                'Terminus Font',
                14),
            menu_font=(
                'Ubuntu Mono',
                18,
                'bold'),
            value_list=value_list,
            title='Settings',
            enable_close_attempted_event=True)

        self.localization_window = LocalizationWindow(
            font=(
                'Terminus Font',
                11),
            menu_font=(
                'Ubuntu Mono',
                13,
                'bold'),
            uav_list=self.uav_list,
            title='Localization',
            use_default_focus=False,
            enable_close_attempted_event=True,
            resizable=True)

        self.main_window = MainWindow(
            settings_window=self.settings_window,
            localization_window=self.localization_window,
            font=(
                'Terminus Font',
                14),
            menu_font=(
                'Ubuntu Mono',
                18,
                'bold'),
            drone_id_list=drone_id_list,
            value_list=value_list,
            title='Keyboard Teleoperation',
            finalize=True,
            return_keyboard_events=True)

        initial_control_mode = ControlModes.SPEED_CONTROL.value

        if teleop_config['initial_mode'] == 'pose':
            initial_control_mode = ControlModes.POSE_CONTROL.value

        self.main_window.make_main_window(initial_control_mode)
        self.execute_main_window(self.main_window, initial_control_mode)

        if thread:
            self._t = threading.Thread(
                target=self.tick_main_window, daemon=True)
            self._t.start()

    def tick_main_window(self):
        """
        Ticks the main window.

        Function to be executed in a separated thread in order to make the execution of the
        teleoperation non-blocking."
        """
        while self.execute_main_window(self.main_window):  # type: ignore
            pass

    def execute_main_window(self, window: MainWindow, custom_event=None):
        """
        Execute main window event handler.

        Main loop where the main window is executed. Every iteration it receives an output from
        the main window so an action is taken in consequence."

        :param window: Main window to be looped
        :type window: MainWindow
        :return: If the interface is still opened
        :rtype: bool
        """
        event, values = window.read(timeout=50)  # type: ignore
        if custom_event is not None:
            event = custom_event
        control_mode, key, value_list, behavior_control, opened = self.main_window.event_handler(
            event, values, self.get_behavior_status())

        # self.uav_list[0].get_logger().info(
        #     str(values['-ACTIVE_BEHAVIORS-']))
        if control_mode is not None:

            self.drone_manager.manage_common_behaviors(key)

            if control_mode == ControlModes.SPEED_CONTROL.value:
                self.drone_manager.manage_speed_behaviors(key, value_list)

            elif control_mode == ControlModes.POSE_CONTROL.value:
                self.drone_manager.manage_pose_behaviors(key, value_list)

        if behavior_control == '-PAUSE_BEHAVIORS-':

            SwarmBehaviorManager.pause_behaviors(
                self.set_value_list(value_list))

        if behavior_control == '-RESUME_BEHAVIORS-':

            SwarmBehaviorManager.resume_behaviors(
                self.set_value_list(value_list))

        if behavior_control == '-PAUSE_ALL_BEHAVIORS-':

            SwarmBehaviorManager.pause_all_behaviors(self.uav_list)

        if behavior_control == '-RESUME_ALL_BEHAVIORS-':

            SwarmBehaviorManager.resume_all_behaviors(self.uav_list)

        return opened

    def get_behavior_status(self):
        """
        Get behavior status for each interface.

        :return: dictionary with namespace and behavior status
        :rtype: dict(namespace, list(int))
        """
        return SwarmBehaviorManager.get_behaviors_status(self.uav_list)

    def set_value_list(self, value_list):
        """
        Convert drone_ids to drone_interfaces.

        :param value_list: list of drone_ids
        :type value_list: dictionary with drone_ids
        :return: converted dictionary of drone_interfaces
        :rtype: dictionary with drone_interfaces
        """
        for uav in self.uav_list:
            if uav.drone_id in value_list:
                value_list[uav] = value_list.pop(uav.drone_id)
        return value_list


if __name__ == '__main__':
    main()
