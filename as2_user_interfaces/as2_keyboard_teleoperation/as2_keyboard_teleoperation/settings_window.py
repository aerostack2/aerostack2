"""Settings window."""

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

import PySimpleGUI as sg


class SettingsWindow(sg.Window):
    """
    Display value control settings so the user can change them.

    :param sg: Window class of PySimpleGUI
    :type sg: PySimpleGUI Window
    """

    def __init__(self, font, menu_font, value_list, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.value_list = value_list
        self.font = font
        self.menu_font = menu_font

    def make_settings_window(self, location):
        """
        Create the settings window layout.

        :param location: location relative to the main window
        :type location: (int, int)
        """
        self.Location = location
        col_value_settings_layout = [[sg.Text("Speed Control Values", font=self.menu_font)],
                                     [sg.Text("Speed value:", font=self.font),
                                     sg.InputText(str(
                                         f"{self.value_list[0]:.2f}"), font=self.font,
                                         key="-VALUE0-", size=(5, 3), background_color="white"),
                                         sg.Text("m/s", font=self.font)],
                                     [sg.Text("Vertical value:", font=self.font),
                                     sg.InputText(str(
                                         f"{self.value_list[1]:.2f}"),
                                         font=self.font,
                                         key="-VALUE1-", size=(5, 3), background_color="white"),
                                         sg.Text("m/s", font=self.font)],
                                     [sg.Text("Turn speed value:", font=self.font),
                                     sg.InputText(str(
                                         f"{self.value_list[2]:.2f}"), font=self.font,
                                         key="-VALUE2-", size=(5, 3), background_color="white"),
                                         sg.Text("rad/s", font=self.font)],
                                     [sg.Text(
                                         "", font=self.font)],
                                     [sg.Text(
                                         "Position control values:", font=self.menu_font)],
                                     [sg.Text("Position value:", font=self.font),
                                     sg.InputText(str(
                                         f"{self.value_list[3]:.2f}"), font=self.font,
                                         key="-VALUE3-", size=(5, 3), background_color="white"),
                                         sg.Text("m", font=self.font)],
                                     [sg.Text("Altitude value:", font=self.font),
                                     sg.InputText(str(
                                         f"{self.value_list[4]:.2f}"), font=self.font,
                                         key="-VALUE4-", size=(5, 3), background_color="white"),
                                         sg.Text("m", font=self.font)],
                                     [sg.Text("Turn angle value:", font=self.font),
                                     sg.InputText(str(
                                         f"{self.value_list[5]:.2f}"), font=self.font,
                                         key="-VALUE5-", size=(5, 3), background_color="white"),
                                         sg.Text("rad", font=self.font)],
                                     [sg.Text(
                                         "", font=self.font)],
                                     [sg.Button("Save", font=self.font),
                                     sg.Button("Exit",
                                               font=self.font, pad=((150, 0), (0, 0)))]]

        self.layout([[sg.Column(col_value_settings_layout)]])

    def event_handler(self, window: sg.Window, settings_event, settings_value):
        """
        Handle the events generated from the user's input.

        :param window: main window
        :type window: sg.Window
        :param settings_event: Events from user's input
        :type settings_event: event
        :param settings_value: Control values previously saved or default
        :type settings_value: list(float)
        :return: list of values and whether the window is opened or not
        :rtype: list(float), bool
        """
        numeric_values = list(settings_value.values())[0:len(self.value_list)]
        for idx, value in enumerate(list(numeric_values)):
            try:
                self.value_list[idx] = float(value)
            except ValueError:
                print("Invalid Input, setting to 0.0")
                self.value_list[idx] = 0.0
                self["-VALUE" +
                     str(idx) + "-"].update(f"{0.00:.2f}")

        if settings_event == sg.WIN_CLOSE_ATTEMPTED_EVENT or settings_event == "Exit":
            self.hide()
            return self.value_list, False

        if settings_event == "Save":
            jdx = 0
            for idx, value in enumerate(self.value_list):
                window["-INPUTTEXT" +
                       str(jdx+1) + "-"].update(value=f"{value:.2f}")
                window["-INPUTTEXT" +
                       str(jdx+2) + "-"].update(value=f"{value:.2f}")
                if ((idx != 1) and (idx != 2) and (idx != 4) and
                        (idx != 5) and (idx != 6) and (idx != 7)):

                    window["-INPUTTEXT" +
                           str(jdx+3) + "-"].update(value=f"{value:.2f}")
                    window["-INPUTTEXT" +
                           str(jdx+4) + "-"].update(value=f"{value:.2f}")
                    jdx = jdx + 4
                else:
                    jdx = jdx + 2

                self["-VALUE" + str(idx) +
                     "-"].update(value=f"{value:.2f}")

            return self.value_list, True

        return self.value_list, True
