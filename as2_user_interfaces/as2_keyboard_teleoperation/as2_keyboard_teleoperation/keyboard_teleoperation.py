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

__authors__ = "Javier Melero Deza, Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import threading
import sys
import rclpy

import PySimpleGUI as sg
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface
from as2_keyboard_teleoperation.main_window import MainWindow
from as2_keyboard_teleoperation.localization_window import LocalizationWindow
from as2_keyboard_teleoperation.behavior_manager import BehaviorManager
from as2_keyboard_teleoperation.settings_window import SettingsWindow
from as2_keyboard_teleoperation.drone_manager import DroneManager
from as2_keyboard_teleoperation.config_values import ControlValues
from as2_keyboard_teleoperation.config_values import ControlModes
from typing import List


def main():
    """entrypoint."""
    drone_id, is_verbose, use_sim_time = sys.argv[1:]
    is_verbose = is_verbose.lower() == 'true'
    use_sim_time = use_sim_time.lower() == 'true'
    uav_list = list()
    rclpy.init()
    if ',' in drone_id:
        drone_id_list = drone_id.split(",")
        for uav_id in drone_id_list:
            uav_list.append(DroneInterface(
                uav_id, verbose=is_verbose, use_sim_time=use_sim_time))
    else:
        uav_list.append(DroneInterface(
            drone_id, verbose=is_verbose, use_sim_time=use_sim_time))

    k_t = KeyboardTeleoperation(uav_list, False)
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

    def __init__(self, list_drone_interface: List[DroneInterface], thread=False):
        self.uav_list = list_drone_interface
        drone_id_list = list()

        for uav in self.uav_list:
            drone_id_list.append([uav.get_namespace(), True])

        value_list = [ControlValues.SPEED_VALUE.value, ControlValues.VERTICAL_VALUE.value,
                      ControlValues.TURN_SPEED_VALUE.value, ControlValues.POSITION_VALUE.value,
                      ControlValues.ALTITUDE_VALUE.value, ControlValues.TURN_ANGLE_VALUE.value,
                      ControlValues.PITCH_ANGLE_VALUE.value,
                      ControlValues.ROLL_ANGLE_VALUE.value,
                      ControlValues.ATTITUDE_DURATION.value]

        self.localization_opened = False

        sg.theme("DarkBlack1")

        self.drone_manager = DroneManager(
            uav_list=self.uav_list, drone_id_list=drone_id_list,
            pose_frame_id='earth', twist_frame_id='earth')

        self.behavior_manager = BehaviorManager(
            uav_list=self.uav_list, drone_id_list=drone_id_list)

        self.settings_window = SettingsWindow(font=("Terminus Font", 14), menu_font=(
            "Ubuntu Mono", 18, 'bold'), value_list=value_list, title="Settings",
            enable_close_attempted_event=True)

        self.localization_window = LocalizationWindow(font=("Terminus Font", 11), menu_font=(
            "Ubuntu Mono", 13, 'bold'), uav_list=self.uav_list, title="Localization",
            use_default_focus=False, enable_close_attempted_event=True, resizable=True)

        self.main_window = MainWindow(settings_window=self.settings_window,
                                      localization_window=self.localization_window,
                                      font=("Terminus Font", 14),
                                      menu_font=("Ubuntu Mono", 18, 'bold'),
                                      drone_id_list=drone_id_list,
                                      value_list=value_list, title="Keyboard Teleoperation",
                                      finalize=True, return_keyboard_events=True)

        if thread:
            self._t = threading.Thread(
                target=self.tick_main_window, daemon=True)
            self._t.start()
        else:
            self.main_window.make_main_window()

    def tick_main_window(self):
        """
        Ticks the main window.

        Function to be executed in a separated thread in order to make the execution of the
        teleoperation non-blocking."
        """
        while self.execute_main_window(self.main_window):  # type: ignore
            pass

    def execute_main_window(self, window: MainWindow):
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
        control_mode, key, value_list, behavior_control, opened = self.main_window.event_handler(
            event, values)

        # self.uav_list[0].get_logger().info(value_list)
        if control_mode is not None:

            self.drone_manager.manage_common_behaviors(key)

            if control_mode == ControlModes.SPEED_CONTROL.value:
                self.drone_manager.manage_speed_behaviors(key, value_list)

            elif control_mode == ControlModes.POSE_CONTROL.value:
                self.drone_manager.manage_pose_behaviors(key, value_list)

        if behavior_control is not None:

            self.behavior_manager.manage_behavior_control(
                behavior_list=value_list, order=behavior_control)

        return opened


if __name__ == '__main__':
    main()
