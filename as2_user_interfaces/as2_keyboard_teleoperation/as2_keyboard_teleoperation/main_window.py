"""Main Window."""

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
from as2_keyboard_teleoperation.settings_window import SettingsWindow
from as2_keyboard_teleoperation.localization_window import LocalizationWindow
from as2_keyboard_teleoperation.config_values import KeyMappings
from as2_keyboard_teleoperation.config_values import ControlValues
from as2_keyboard_teleoperation.config_values import ControlModes


class MainWindow(sg.Window):
    """
    Create the main window.

    Send events taken as an input from the user.
    This class also handles the settings window and the localization window.
    :param sg: Window class of PySimpleGUI
    :type sg: PySimpleGUI Window
    """

    def __init__(self, settings_window: SettingsWindow,
                 localization_window: LocalizationWindow, font, menu_font,
                 drone_id_list, value_list, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.settings_window = settings_window
        self.localization_window = localization_window

        self.font = font
        self.menu_font = menu_font
        self.return_keyboard_events = True
        self.use_default_focus = True
        self.resizable = True
        self.drone_id_list = drone_id_list
        self.control_mode = ControlModes.SPEED_CONTROL.value
        self.value_list = value_list
        self.localization_opened = False
        self.active_behaviors = []
        self.paused_behaviors = []

    def make_main_window(self):
        """Create the main window layout."""
        col1_layout = [
            [sg.Text(KeyMappings.TAKE_OFF_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.LAND_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.HOVER_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.EMERGENCY_KEY.value, font=self.font)],
            [sg.Text("r", font=self.font)]]

        col2_layout = [
            [sg.Text("Take off", font=self.font)],
            [sg.Text("Land", font=self.font)],
            [sg.Text("Hover", font=self.font)],
            [sg.Text("Emergency Stop", font=self.font)],
            [sg.Text("Reset orientation", font=self.font)]]

        col3_layout = [
            [sg.Text("↑", font=self.font)],
            [sg.Text("↓", font=self.font)],
            [sg.Text("→", font=self.font)],
            [sg.Text("←", font=self.font)],
            [sg.Text("", font=self.font)]]

        col4_layout = [
            [sg.Text("Increase forward speed", font=self.font), sg.Text(
                f"{ControlValues.SPEED_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT1-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase backward speed", font=self.font), sg.Text(
                f"{ControlValues.SPEED_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT2-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the right", font=self.font), sg.Text(
                f"{ControlValues.SPEED_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT3-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the left", font=self.font), sg.Text(
                f"{ControlValues.SPEED_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT4-"), sg.Text("m/s", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col6_layout = [
            [sg.Text("Increase altitude", font=self.font),
             sg.Text(f"{ControlValues.ALTITUDE_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT13-"),
             sg.Text("m", font=self.font)],
            [sg.Text("Decrease altitude", font=self.font),
             sg.Text(f"{ControlValues.ALTITUDE_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT14-"),
             sg.Text("m", font=self.font)],
            [sg.Text("Turn counter-clockwise", font=self.font),
             sg.Text(f"{ControlValues.TURN_ANGLE_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT15-"),
             sg.Text("rad", font=self.font)],
            [sg.Text("Turn clockwise", font=self.font),
             sg.Text(f"{ControlValues.TURN_ANGLE_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT16-"),
             sg.Text("rad", font=self.font)]
        ]

        col6b_layout = [
            [sg.Text("Increase vertical speed", font=self.font),
             sg.Text(f"{ControlValues.VERTICAL_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT5-"),
             sg.Text("m/s", font=self.font)],
            [sg.Text("Decrease vertical speed", font=self.font),
             sg.Text(f"{ControlValues.VERTICAL_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT6-"),
             sg.Text("m/s", font=self.font)],
            [sg.Text("Turn speed counter-clockwise", font=self.font),
             sg.Text(f"{ControlValues.TURN_SPEED_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT7-"),
             sg.Text("rad/s", font=self.font)],
            [sg.Text("Turn speed clockwise", font=self.font),
             sg.Text(f"{ControlValues.TURN_SPEED_VALUE.value:.2f}",
                     font=self.font, key="-INPUTTEXT8-"),
             sg.Text("rad/s", font=self.font)]
        ]

        col7_layout = [
            [sg.Text("Increase forward position", font=self.font), sg.Text(
                f"{ControlValues.POSITION_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT9-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase backward position", font=self.font), sg.Text(
                f"{ControlValues.POSITION_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT10-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the right", font=self.font), sg.Text(
                f"{ControlValues.POSITION_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT11-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the left", font=self.font), sg.Text(
                f"{ControlValues.POSITION_VALUE.value:.2f}",
                font=self.font, key="-INPUTTEXT12-"), sg.Text("m", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col8_layout = [
            [sg.Text(KeyMappings.UP_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.DOWN_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.ROTATE_LEFT_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.ROTATE_RIGHT_KEY.value, font=self.font)]
        ]

        col_button_layout = [
            [sg.Button("Speed mode", font=self.font,
                       key=ControlModes.SPEED_CONTROL.value, focus=True)],
            [sg.Button("Pose mode", font=self.font,
                       key=ControlModes.POSE_CONTROL.value)],
        ]

        main_buttons_layout = [
            [sg.Text("BASIC MOTIONS", pad=((10, 280), (10, 0)), font=self.menu_font),
             sg.Text("SPEED CONTROL", pad=((0, 0), (10, 0)),
                     font=self.menu_font, key="-SP_CONTROL-"),
             sg.Text("POSE CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font,
             visible=False, key="-POS_CONTROL-")],
            [sg.Column(col1_layout, element_justification='left'),
             sg.Column(col2_layout, element_justification='left',
                       pad=((0, 190), (0, 0))),
             sg.Column(col3_layout, element_justification='left',
                       justification="left"),
             sg.Column(col4_layout, element_justification='left',
                       justification="left", key="-COL4-"),
             sg.Column(col7_layout, element_justification='left', visible=False, key="-COL7-")],
            [sg.Text("TELEOPERATION MODE SELECTION", pad=((10, 100), (10, 0)),
                     font=self.menu_font),
             sg.Text("POSE CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font, key="-P_CONTROL-",
             visible=False)],
            [sg.Column(col_button_layout, element_justification='left', pad=((0, 290), (0, 0))),
             sg.Column(col8_layout, element_justification='left', key="-COL8-"),
             sg.Column(col6_layout, element_justification='left',
                       key="-COL6-", visible=False),
             sg.Column(col6b_layout, element_justification='left', key="-COL6B-")]]

        col_selection_layout = list()
        # Here active behavior list is added
        all_selector = True

        for drone_id in self.drone_id_list:
            col_selection_layout.append(
                [sg.CB(drone_id[0], key=drone_id[0], enable_events=True,
                       font=self.font, background_color="grey", default=drone_id[1])])
            if not drone_id[1]:
                all_selector = False

        selection_frame = sg.Frame("Drone selection control",
                                   layout=[[sg.Column(col_selection_layout, expand_y=True,
                                                      expand_x=True, scrollable=True,
                                                      vertical_scroll_only=True,
                                                      background_color="grey",
                                                      sbar_trough_color="white",
                                                      sbar_arrow_color="grey")],
                                           [sg.CB("All", key="All", enable_events=True,
                                                  font=self.font, background_color="grey",
                                                  expand_x=True,
                                                  default=all_selector)]],
                                   vertical_alignment="top", size=(200, 300), expand_y=True)

        behavior_frame = sg.Frame("Behavior control",
                                  key="-BEHAVIOR CONTROL-",
                                  layout=[[sg.Text("Active Behaviors", font=self.menu_font,
                                                   background_color="grey", expand_x=True,
                                                   justification="center"),
                                           sg.Text("Paused Behaviors", font=self.menu_font,
                                                   background_color="grey",
                                                   expand_x=True, justification="center")],
                                          [sg.Listbox([],
                                                      background_color="grey",
                                                      select_mode=sg.LISTBOX_SELECT_MODE_MULTIPLE,
                                                      highlight_text_color="white",
                                                      text_color="white",
                                                      highlight_background_color="blue",
                                                      expand_y=True, size=(30,),
                                                      sbar_trough_color="white",
                                                      sbar_arrow_color="grey",
                                                      key="-ACTIVE_BEHAVIORS-"),
                                           sg.Listbox([],
                                                      background_color="grey",
                                                      select_mode=sg.LISTBOX_SELECT_MODE_MULTIPLE,
                                                      highlight_text_color="white",
                                                      text_color="white",
                                                      highlight_background_color="blue",
                                                      expand_y=True,
                                                      size=(30,), sbar_trough_color="white",
                                                      sbar_arrow_color="grey",
                                                      key="-PAUSED_BEHAVIORS-")],
                                          [sg.Button(" Pause ", font=self.font,
                                                     key="-PAUSE_BEHAVIORS-", expand_x=True),
                                           sg.Button("Pause All", font=self.font,
                                                     key="-PAUSE_ALL_BEHAVIORS-", expand_x=True),
                                           sg.Button("Resume", font=self.font,
                                                     key="-RESUME_BEHAVIORS-", expand_x=True),
                                           sg.Button("Resume All", font=self.font,
                                                     key="-RESUME_ALL_BEHAVIORS-",
                                                     expand_x=True)]],
                                  vertical_alignment="top", size=(470, 300),
                                  expand_y=True, visible=False)

        self.layout([
            [sg.Button("Settings", font=self.menu_font),
             sg.Text("|", font=self.menu_font),
             sg.Button("Localization", font=self.menu_font),
             sg.Text("|", font=self.menu_font),
             sg.Button("Behavior control",
                    font=self.menu_font, key="-BEHAVIOR-"),
             sg.Text("Teleoperation mode: Speed mode", justification="left",
                    font=self.menu_font, key="-HEADER_SPEED-",
                    visible=True, pad=((78, 0), (0, 0))),
             sg.Text("Teleoperation mode: Pose mode", justification="left",
                    font=self.menu_font, key="-HEADER_POSE-",
                    visible=False, pad=((78, 0), (0, 0)))],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Column(layout=main_buttons_layout),
             selection_frame, behavior_frame],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Text("Last key pressed:", font=self.menu_font),
             sg.Text("", font=self.menu_font, key="-key_pressed-")]])

    def event_handler(self, event, value, behaviors_status):
        """
        Handle the events taken from the user's input.

        Make the calls to the
        event handlers of the localization and settings windows.
        :param event: Event generated from the user's input keyboard
        :type event: String
        :param value: Value connected to user's input numeric values
        :type value: float
        :return: A list containing the control mode, key pressed (event), a list of values
        and wheter if it is still opened or not
        :rtype: string, string, list(float), bool
        """
        if event == sg.WIN_CLOSED:
            if self.localization_opened:
                self.localization_window.close()
            return None, None, None, None, False

        selection_values = list(value.values())[:len(self.drone_id_list)+1]

        if event == "Localization":  # Non-Blocking
            if not self.localization_opened:
                self.localization_window.make_localization_window(
                    location=self.current_location())

                if self.localization_window._Hidden:
                    self.localization_window.move(self.current_location()[0],
                                                  self.current_location()[1])
                    self.localization_window.un_hide()

                self.localization_opened = True

        elif event == "Settings":  # Blocking
            self.settings_window.make_settings_window(
                location=self.current_location())

            if self.settings_window._Hidden:
                self.settings_window.move(self.current_location()[0],
                                          self.current_location()[1])
                self.settings_window.un_hide()

            while True:
                settings_event, settings_value = self.settings_window.read()  # type: ignore

                self.value_list, opened = self.settings_window.event_handler(
                    self, settings_event, settings_value)

                if not opened:
                    break

        elif event == "All":

            if selection_values[-1]:
                for index, _ in enumerate(selection_values[:-1]):
                    self[self.drone_id_list[index][0]].update(True)
            else:
                for index, _ in enumerate(selection_values[:-1]):
                    self[self.drone_id_list[index][0]].update(False)

            for drone_id in self.drone_id_list:
                drone_id[1] = True

        elif event in [x for list in self.drone_id_list for x in list]:

            if all(selection_values[:-1]):
                self["All"].update(True)
            else:
                self["All"].update(False)

            for index, selection in enumerate(selection_values[:-1]):
                self.drone_id_list[index][1] = bool(selection)

        elif event in ControlModes.list() or event == "-BEHAVIOR-":
            self.update_main_window_mode(event)

        elif event.split(":")[0] in KeyMappings.list():
            key = event.split(":")
            self["-key_pressed-"].update(value=key[0])
            if key[0] == KeyMappings.FORWARD_KEY.value:
                self["-key_pressed-"].update(value="↑")
            elif key[0] == KeyMappings.BACKWARD_KEY.value:
                self["-key_pressed-"].update(value="↓")
            elif key[0] == KeyMappings.LEFT_KEY.value:
                self["-key_pressed-"].update(value="←")
            elif key[0] == KeyMappings.RIGHT_KEY.value:
                self["-key_pressed-"].update(value="→")

            return self.control_mode, key[0], self.value_list, None, True

        if self.localization_opened:
            self.localization_opened = self.localization_window.execute_localization_window()

        self.update_behavior(behaviors_status, value)

        if event == "-PAUSE_BEHAVIORS-":

            return None, None, self.parse_behavior_list(value["-ACTIVE_BEHAVIORS-"]), event, True

        if event == "-RESUME_BEHAVIORS-":

            return None, None, self.parse_behavior_list(value["-PAUSED_BEHAVIORS-"]), event, True

        if event == "-PAUSE_ALL_BEHAVIORS-":

            return None, None, None, event, True

        if event == "-RESUME_ALL_BEHAVIORS-":

            return None, None, None, event, True

        return None, None, None, None, True

    def update_main_window_mode(self, event):
        """
        Update the main window control mode.

        :param event: User input
        :type event: string
        """
        if event == ControlModes.SPEED_CONTROL.value:
            self.control_mode = event
            self.update_window_to_speed()

        elif event == ControlModes.POSE_CONTROL.value:
            self.control_mode = event
            self.update_window_to_pose()

        elif event == "-BEHAVIOR-":
            self["-BEHAVIOR CONTROL-"].update(
                visible=(not self["-BEHAVIOR CONTROL-"].visible))

    def update_window_to_pose(self):
        """Update window to pose mode."""
        self[ControlModes.POSE_CONTROL.value].set_focus(True)
        self["-HEADER_SPEED-"].update(visible=False)
        self["-HEADER_POSE-"].update(visible=True)
        self["-SP_CONTROL-"].update(visible=False)
        self["-POS_CONTROL-"].update(visible=True)
        self["-COL4-"].update(visible=False)
        self["-P_CONTROL-"].update(visible=False)
        self["-COL7-"].update(visible=True)
        self["-COL6B-"].update(visible=False)
        self["-COL6-"].update(visible=True)

    def update_window_to_speed(self):
        """Update window to speed mode."""
        self[ControlModes.SPEED_CONTROL.value].set_focus(True)
        self["-HEADER_SPEED-"].update(visible=True)
        self["-HEADER_POSE-"].update(visible=False)
        self["-SP_CONTROL-"].update(visible=True)
        self["-POS_CONTROL-"].update(visible=False)
        self["-P_CONTROL-"].update(visible=False)
        self["-COL4-"].update(visible=True)
        self["-COL7-"].update(visible=False)
        self["-COL6B-"].update(visible=True)
        self["-COL6-"].update(visible=False)

    def update_behavior(self, behaviors_status: dict, value):
        """Update Behavior values."""
        active_behaviors = []
        paused_behaviors = []
        for namespace in behaviors_status:
            for behavior in behaviors_status[namespace]:
                if behaviors_status[namespace][behavior] == 1:
                    active_behaviors.append(namespace + ":" + behavior)
                elif behaviors_status[namespace][behavior] == 2:
                    paused_behaviors.append(namespace + ":" + behavior)

        self["-ACTIVE_BEHAVIORS-"].update(values=active_behaviors,
                                          set_to_index=[active_behaviors.index(behavior)
                                                        for behavior
                                                        in value["-ACTIVE_BEHAVIORS-"]
                                                        if behavior in active_behaviors])

        self["-PAUSED_BEHAVIORS-"].update(values=paused_behaviors,
                                          set_to_index=[paused_behaviors.index(behavior)
                                                        for behavior
                                                        in value["-PAUSED_BEHAVIORS-"]
                                                        if behavior in paused_behaviors])

    def parse_behavior_list(self, value):
        behavior_dict = dict()
        for drone_behavior in value:
            behavior_dict.setdefault(drone_behavior.split(
                ":")[0], []).append(drone_behavior.split(":")[1])

        return behavior_dict
