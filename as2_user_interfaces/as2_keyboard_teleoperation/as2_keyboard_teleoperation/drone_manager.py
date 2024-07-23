"""Drone control manager."""

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

import threading
from typing import List

from as2_keyboard_teleoperation.config_values import KeyMappings
from as2_keyboard_teleoperation.config_values import Options
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface


class DroneManager:
    """Manage drone control."""

    def __init__(self, uav_list: List[DroneInterface],
                 drone_id_list, pose_frame_id, twist_frame_id):
        self.uav_list = uav_list
        self.drone_id_list = drone_id_list
        self.pose_frame_id = pose_frame_id
        self.twist_frame_id = twist_frame_id
        self.value_common_behavior = {KeyMappings.TAKE_OFF_KEY.value: self.take_off,
                                      KeyMappings.LAND_KEY.value: self.land,
                                      KeyMappings.HOVER_KEY.value: self.hover,
                                      KeyMappings.EMERGENCY_KEY.value: self.emergency_stop}
        self.reference_cleared = False

    def manage_common_behaviors(self, key):
        """
        Manage behaviors common to every control mode.

        :param key: Control order
        :type key: string
        """
        if key in self.value_common_behavior:
            self.execute_common_behaviors(self.value_common_behavior[key])

    def manage_speed_behaviors(self, key, value_list):
        """
        Manage speed control behaviors.

        :param key: Control order
        :type key: string
        :param value_list: Speed value
        :type value_list: float
        """
        if key == KeyMappings.FORWARD_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    lineal = [value_list[0], 0.0, 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index], lineal, 0.0,))

        elif key == KeyMappings.BACKWARD_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    lineal = [-value_list[0], 0.0, 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index], lineal, 0.0,))

        elif key == KeyMappings.RIGHT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, -value_list[0], 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index], lineal, 0.0,))

        elif key == KeyMappings.LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, value_list[0], 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index], lineal, 0.0,))

        elif key == KeyMappings.UP_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, 0.0, value_list[1]]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index], lineal, 0.0,))

        elif key == KeyMappings.DOWN_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, 0.0, -value_list[1]]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index], lineal, 0.0,))

        elif key == KeyMappings.ROTATE_RIGHT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, 0.0, 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index],
                                             lineal, -value_list[2],))

        elif key == KeyMappings.ROTATE_LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, 0.0, 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index],
                                             lineal, value_list[2],))

        if key is None:
            if not self.reference_cleared:
                for index, drone_id in enumerate(self.drone_id_list):
                    if drone_id[1]:

                        lineal = [0.0, 0.0, 0.0]
                        self.execute_function(
                            self.move_at_speed, (self.uav_list[index],
                                                 lineal, 0.0,))
                self.reference_cleared = True
        else:
            self.reference_cleared = False

    def manage_pose_behaviors(self, key, value_list):
        """
        Manage pose control behaviors.

        :param key: Control order
        :type key: string
        :param value_list: Pose value
        :type value_list: float
        """
        if key == KeyMappings.FORWARD_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    position = [self.uav_list[index].position[0] + value_list[3],
                                self.uav_list[index].position[1],
                                self.uav_list[index].position[2]]

                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position,
                                          self.uav_list[index].orientation[2],))

        elif key == KeyMappings.BACKWARD_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0] - value_list[3],
                                self.uav_list[index].position[1],
                                self.uav_list[index].position[2]]

                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position,
                                          self.uav_list[index].orientation[2],))

        elif key == KeyMappings.RIGHT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1] -
                                value_list[3],
                                self.uav_list[index].position[2]]
                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position,
                                          self.uav_list[index].orientation[2],))

        elif key == KeyMappings.LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1] +
                                value_list[3],
                                self.uav_list[index].position[2]]
                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position,
                                          self.uav_list[index].orientation[2],))

        elif key == KeyMappings.UP_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1],
                                self.uav_list[index].position[2] + value_list[4]]
                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position,
                                          self.uav_list[index].orientation[2],))

        elif key == KeyMappings.DOWN_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1],
                                self.uav_list[index].position[2] - value_list[4]]
                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position,
                                          self.uav_list[index].orientation[2],))

        elif key == KeyMappings.ROTATE_RIGHT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1],
                                self.uav_list[index].position[2]]
                    euler = self.uav_list[index].orientation
                    yaw = euler[2] - value_list[5]

                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position, yaw,))

        elif key == KeyMappings.ROTATE_LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1],
                                self.uav_list[index].position[2]]
                    euler = self.uav_list[index].orientation
                    yaw = euler[2] + value_list[5]

                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position, yaw,))

    def execute_common_behaviors(self, method):
        """
        Execute common behaviors.

        :param method: behavior to be executed
        :type method: function
        """
        for index, drone_id in enumerate(self.drone_id_list):
            if drone_id[1]:
                self.execute_function(method, (self.uav_list[index],))

    def execute_function(self, target, args):
        """
        Execute function.

        :param method: function to be executed
        :type method: function
        """
        try:
            threading.Thread(target=target, args=args, daemon=True).start()
        except Exception as ex:
            print('Error starting work thread: ', ex)

    # FUNCTIONS TO CALL THE DRONE INTERFACES FUNCTIONS

    # def shutdown(self):
        # self.t.join()

    def take_off(self, uav: DroneInterface):
        """Take off."""
        if (Options.ARM_ON_TAKE_OFF.value):
            uav.arm()
            uav.offboard()

        uav.takeoff(1.0, 1.0)

    def land(self, uav: DroneInterface):
        """Land."""
        uav.land(0.5)

    def hover(self, uav: DroneInterface):
        """Hover."""
        # uav.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
        #     [0.0, 0.0, 0.0], self.twist_frame_id, 0.0)
        uav.motion_ref_handler.hover()

    def move_at_speed(self, uav: DroneInterface, lineal, yaw_speed):
        """Move at speed."""
        frame_id = 'earth'
        if 'base_link' in self.twist_frame_id:
            frame_id = uav.get_namespace().replace('/', '') + '/base_link'
        uav.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
            lineal, frame_id, yaw_speed)

    def go_to_pose(self, uav: DroneInterface, position, orientation):
        """Go to pose."""
        frame_id = 'earth'
        if 'base_link' in self.pose_frame_id:
            frame_id = uav.get_namespace().replace('/', '') + '/base_link'
        uav.motion_ref_handler.position.send_position_command_with_yaw_angle(
            position, None, frame_id,
            'earth', orientation)

    def emergency_stop(self, uav: DroneInterface):
        """Emergency stop."""
        uav.send_emergency_killswitch_to_aircraft()
