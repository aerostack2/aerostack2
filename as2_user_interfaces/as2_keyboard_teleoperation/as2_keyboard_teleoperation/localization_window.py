"""Localization window."""

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

from typing import List

from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface

import PySimpleGUI as sg


class LocalizationWindow(sg.Window):
    """
    Display the localization of the drones.

    :param sg: Window class of PySimpleGUI
    :type sg: PySimpleGUI Window
    """

    def __init__(self, font, menu_font, uav_list: List[DroneInterface], *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.font = font
        self.menu_font = menu_font
        self.uav_list = uav_list

    def make_localization_window(self, location):
        """
        Create the localization window layout.

        :param location: location relative to the main window
        :type location: (int, int)
        """
        self.Location = location

        drone_frame_list_layout = []

        for index, uav in enumerate(self.uav_list):

            drone_localization_layout = [[
                sg.Text('Position', font=self.menu_font,
                        key='-POSITION_' + str(index) + '-')],
                [sg.Text('x:', font=self.font),
                 sg.Text(f'{round(uav.position[0], 2):.2f}',
                         font=self.font, key='-LOCALIZATION_X_' + str(index) + '-'),
                 sg.Text(',', font=self.font),
                 sg.Text('y:', font=self.font),
                 sg.Text(f'{round(uav.position[1], 2):.2f}', font=self.font,
                         key='-LOCALIZATION_Y_' + str(index) + '-'),
                 sg.Text(',', font=self.font),
                 sg.Text('z:', font=self.font),
                 sg.Text(f'{round(uav.position[2], 2):.2f}',
                 font=self.font, key='-LOCALIZATION_Z_' + str(index) + '-')],
                [sg.Text('Orientation', font=self.menu_font,
                         key='-ORIENTATION_' + str(index) + '-')],
                [sg.Text('r:', font=self.font),
                 sg.Text(f'{round(uav.orientation[0], 2):.2f}',
                         font=self.font, key='-LOCALIZATION_R_' + str(index) + '-'),
                 sg.Text(',', font=self.font),
                 sg.Text('p:', font=self.font),
                 sg.Text(f'{round(uav.orientation[1], 2):.2f}', font=self.font,
                         key='-LOCALIZATION_P_' + str(index) + '-'),
                 sg.Text(',', font=self.font),
                 sg.Text('y:', font=self.font),
                 sg.Text(f'{round(uav.orientation[2], 2):.2f}',
                         font=self.font, key='-LOCALIZATION_YW_' + str(index) + '-')]]

            localization_frame = sg.Frame(
                uav.drone_id, layout=drone_localization_layout)
            if index % 2 == 0:
                drone_frame_list_layout.append([localization_frame])
            else:
                drone_frame_list_layout[-1].append(localization_frame)

        localization_column = sg.Column(layout=drone_frame_list_layout,
                                        scrollable=True, vertical_scroll_only=True,
                                        size=(570, 500), sbar_trough_color='white',
                                        sbar_arrow_color='grey')

        localization_layout = [[localization_column],
                               [sg.Button('Exit', font=self.font, pad=((500, 0), (20, 0)))]]

        self.layout(localization_layout)

    def execute_localization_window(self):
        """
        Update the localization window visual output.

        :return: Opened or not
        :rtype: bool
        """
        localization_event, _ = self.read(timeout=1)  # type: ignore

        for index, uav in enumerate(self.uav_list):

            self['-LOCALIZATION_X_' + str(index) + '-'].update(
                value=f'{round(uav.position[0], 2):.2f}')
            self['-LOCALIZATION_Y_' + str(index) + '-'].update(
                value=f'{round(uav.position[1], 2):.2f}')
            self['-LOCALIZATION_Z_' + str(index) + '-'].update(
                value=f'{round(uav.position[2], 2):.2f}')
            self['-LOCALIZATION_R_' + str(index) + '-'].update(
                value=f'{round(uav.orientation[0], 2):.2f}')
            self['-LOCALIZATION_P_' + str(index) + '-'].update(
                value=f'{round(uav.orientation[1], 2):.2f}')
            self['-LOCALIZATION_YW_' + str(index) + '-'].update(
                value=f'{round(uav.orientation[2], 2):.2f}')

        # self.refresh()
        if localization_event == sg.WIN_CLOSE_ATTEMPTED_EVENT or localization_event == 'Exit':
            self.hide()
            return False

        return True
