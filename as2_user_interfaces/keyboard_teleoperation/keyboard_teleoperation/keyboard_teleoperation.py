"""Keyboard Teleoperation"""

import sys
import threading
import rclpy

import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface
import motion_reference_handlers.utils as mh_utils


def main():
    """entrypoint"""

    drone_id, is_verbose, use_sim_time = sys.argv[1:]
    is_verbose = is_verbose.lower() == 'true'
    use_sim_time = use_sim_time.lower() == 'true'

    rclpy.init()

    uav = DroneInterface(drone_id, verbose=is_verbose,
                         use_sim_time=use_sim_time)
    kt = keyboardTeleoperation(uav, False)
    while kt.execute_main_window(kt.window):
        pass

    rclpy.shutdown()


class keyboardTeleoperation:
    """Keyborad Teleoperation"""

    def __init__(self, drone_interface: DroneInterface, thread=False):
        self.uav = drone_interface
        self.drone_id = self.uav.drone_id

        self.control_mode = "-SPEED-"
        self.value_list = [1.0, 1.0, 0.10, 1.0, 1.0, 0.10, 0.20, 0.20, 0.50]
        self.localization_opened = False

        self.pose_frame_id = 'earth'
        self.twist_frame_id = 'earth'

        self.font = ("Terminus Font", 14)
        self.font_menu = ("Ubuntu Mono", 18, 'bold')

        if thread:
            self.t = threading.Thread(target=self.tick_main_window, daemon=True)
            self.t.start()
        else:
            self.window = self.make_main_window()

    def tick_main_window(self):
        self.window = self.make_main_window()
        while self.execute_main_window(self.window):
            pass

    def execute_main_window(self, window: sg.Window):

        event, values = window.read(timeout=50)  # type: ignore

        if event == sg.WIN_CLOSED:
            if (self.localization_opened):
                self.localization_window.close()
            return False

        self.manage_main_window_event(window, event, values)

        return True

    def manage_main_window_event(self, window: sg.Window, event, value):

        if event == "Localization":
            if (not self.localization_opened):
                self.localization_window = self.make_localization_window()

            self.localization_opened = True

        elif event == "Settings":
            settings_window = self.make_settings_window()

            while (True):
                settings_event, settings_value = settings_window.read()  # type: ignore

                if settings_event == sg.WIN_CLOSED or settings_event == "Exit":
                    settings_window.close()
                    break

                self.manage_settings_event(window, settings_window, settings_event, settings_value)   

        elif event == "-SPEED-":
            self.control_mode = event
            window["-SPEED-"].set_focus(True)
            window["-HEADER_SPEED-"].update(visible=True)
            window["-HEADER_POSE-"].update(visible=False)
            window["-HEADER_ATTITUDE-"].update(visible=False)
            window["-SP_CONTROL-"].update(visible=True)
            window["-POS_CONTROL-"].update(visible=False)
            window["-AT_CONTROL-"].update(visible=False)
            window["-P_CONTROL-"].update(visible=False)
            window["-COL5-"].update(visible=False)
            window["-COL4-"].update(visible=True)
            window["-COL7-"].update(visible=False)
            window["-COL6B-"].update(visible=True)
            window["-COL6-"].update(visible=False)

        elif event == "-POSE-":
            self.control_mode = event
            window["-POSE-"].set_focus(True)
            window["-HEADER_SPEED-"].update(visible=False)
            window["-HEADER_POSE-"].update(visible=True)
            window["-HEADER_ATTITUDE-"].update(visible=False)
            window["-SP_CONTROL-"].update(visible=False)
            window["-POS_CONTROL-"].update(visible=True)
            window["-AT_CONTROL-"].update(visible=False)
            window["-COL5-"].update(visible=False)
            window["-COL4-"].update(visible=False)
            window["-P_CONTROL-"].update(visible=False)
            window["-COL7-"].update(visible=True)
            window["-COL6B-"].update(visible=False)
            window["-COL6-"].update(visible=True)

        elif event == "-ATTITUDE-":
            self.control_mode = event
            window["-ATTITUDE-"].set_focus(True)
            window["-HEADER_SPEED-"].update(visible=False)
            window["-HEADER_POSE-"].update(visible=False)
            window["-HEADER_ATTITUDE-"].update(visible=True)
            window["-SP_CONTROL-"].update(visible=False)
            window["-POS_CONTROL-"].update(visible=False)
            window["-AT_CONTROL-"].update(visible=True)
            window["-P_CONTROL-"].update(visible=True)
            window["-COL5-"].update(visible=True)
            window["-COL4-"].update(visible=False)
            window["-COL7-"].update(visible=False)
            window["-COL6B-"].update(visible=False)
            window["-COL6-"].update(visible=True)
        else:
            input = event.split(":")
            self.manage_common_behaviors(window, input)

            if (self.control_mode == "-SPEED-"):
                self.manage_speed_behaviors(window, input)

            elif (self.control_mode == "-POSE-"):
                self.manage_pose_behaviors(window, input)

            if (self.localization_opened):
                self.execute_localization_window(self.localization_window)

    def manage_settings_event(self, window: sg.Window, settings_window: sg.Window, settings_event, settings_value):

        for idx, value in enumerate(list(settings_value.values())):
            try:
                self.value_list[idx] = float(value)
            except ValueError:
                print("Invalid Input, setting to 0.0")
                self.value_list[idx] = 0.0
                settings_window["-VALUE" +
                                str(idx) + "-"].update(value="{:0.2f}".format(0.00))

        if settings_event == "Save":
            jdx = 0
            for idx, value in enumerate(self.value_list):
                window["-INPUTTEXT" +
                        str(jdx+1) + "-"].update(value="{:0.2f}".format(value))
                window["-INPUTTEXT" +
                        str(jdx+2) + "-"].update(value="{:0.2f}".format(value))
                if ((idx != 1) and (idx != 2) and (idx != 4) and (idx != 5) and (idx != 6) and (idx != 7)):

                    window["-INPUTTEXT" +
                            str(jdx+3) + "-"].update(value="{:0.2f}".format(value))
                    window["-INPUTTEXT" +
                            str(jdx+4) + "-"].update(value="{:0.2f}".format(value))
                    jdx = jdx + 4
                else:
                    jdx = jdx + 2

                settings_window["-VALUE" +
                                str(idx) + "-"].update(value="{:0.2f}".format(value))

    def execute_localization_window(self, localization_window: sg.Window):
        localization_event, _ = localization_window.read(timeout=1)  # type: ignore

        localization_window["-LOCALIZATION_X-"].update(
            value="{:0.2f}".format(round(self.uav.position[0], 2)))
        localization_window["-LOCALIZATION_Y-"].update(
            value="{:0.2f}".format(round(self.uav.position[1], 2)))
        localization_window["-LOCALIZATION_Z-"].update(
            value="{:0.2f}".format(round(self.uav.position[2], 2)))

        localization_window["-LOCALIZATION_R-"].update(
            value="{:0.2f}".format(round(self.uav.orientation[0], 2)))
        localization_window["-LOCALIZATION_P-"].update(
            value="{:0.2f}".format(round(self.uav.orientation[1], 2)))
        localization_window["-LOCALIZATION_YW-"].update(
            value="{:0.2f}".format(round(self.uav.orientation[2], 2)))

        # localization_window.refresh()
        if localization_event == sg.WIN_CLOSED or localization_event == "Exit":
            self.localization_opened = False
            localization_window.close()

    # FUNCTIONS THAT BUILD THE GRAPHICAL INTERFACE

    def make_main_window(self):
        sg.theme("DarkBlack1")

        col1_layout = [
            [sg.Text("t", font=self.font)],
            [sg.Text("l", font=self.font)],
            [sg.Text("space", font=self.font)],
            [sg.Text("del", font=self.font)],
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
            [sg.Text("←", font=self.font)],
            [sg.Text("→", font=self.font)],
            [sg.Text("", font=self.font)]]
        col4_layout = [
            [sg.Text("Increase forward speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT1-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase backward speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT2-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the right", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT3-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the left", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT4-"), sg.Text("m/s", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col5_layout = [
            [sg.Text("Pitch", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT17-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT21-"), sg.Text("s", font=self.font)],
            [sg.Text("Pitch -", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT18-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT22-"), sg.Text("s", font=self.font)],
            [sg.Text("Roll", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT19-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT23-"), sg.Text("s", font=self.font)],
            [sg.Text("Roll -", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT20-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT24-"), sg.Text("s", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col6_layout = [
            [sg.Text("Increase altitude", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT13-"), sg.Text("m", font=self.font)],
            [sg.Text("Decrease altitude", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT14-"), sg.Text("m", font=self.font)],
            [sg.Text("Turn counter-clockwise", font=self.font), sg.Text("0.10",
                                                                   font=self.font, key="-INPUTTEXT15-"), sg.Text("rad", font=self.font)],
            [sg.Text("Turn clockwise", font=self.font), sg.Text(
                "0.10", font=self.font, key="-INPUTTEXT16-"), sg.Text("rad", font=self.font)]
        ]

        col6B_layout = [
            [sg.Text("Increase vertical speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT5-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Decrease vertical speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT6-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Turn speed counter-clockwise", font=self.font), sg.Text(
                "0.10", font=self.font, key="-INPUTTEXT7-"), sg.Text("rad/s", font=self.font)],
            [sg.Text("Turn speed clockwise", font=self.font), sg.Text(
                "0.10", font=self.font, key="-INPUTTEXT8-"), sg.Text("rad/s", font=self.font)]
        ]

        col7_layout = [
            [sg.Text("Increase forward position", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT9-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase backward position", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT10-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the right", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT11-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the left", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT12-"), sg.Text("m", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col8_layout = [
            [sg.Text("w", font=self.font)],
            [sg.Text("s", font=self.font)],
            [sg.Text("a", font=self.font)],
            [sg.Text("d", font=self.font)]
        ]

        col_button_layout = [
            [sg.Button("Speed mode", font=self.font, key="-SPEED-", focus=True)],
            [sg.Button("Pose mode", font=self.font, key="-POSE-")],
            [sg.Button("Attitude mode", font=self.font, key="-ATTITUDE-")]
        ]

        self.layout = [
                        [sg.Button("Settings", font=self.font_menu), sg.Text("|", font=self.font_menu), sg.Button("Localization", font=self.font_menu), sg.Text("|", font=self.font_menu), sg.Text("Teleoperation mode: Speed mode", justification="left", font=self.font_menu, key="-HEADER_SPEED-", visible=True, pad=((78, 0), (0, 0))),
                        sg.Text("Teleoperation mode: Pose mode", justification="left",
                                font=self.font_menu, key="-HEADER_POSE-", visible=False, pad=((78, 0), (0, 0))),
                        sg.Text("Teleoperation mode: Attitude mode", justification="left", font=self.font_menu, key="-HEADER_ATTITUDE-", visible=False, pad=((78, 0), (0, 0)))],
                       [sg.HSeparator(pad=(0, 10))],
                       [sg.Text("BASIC MOTIONS", pad=((10, 280), (10, 0)), font=self.font_menu), sg.Text("SPEED CONTROL", pad=((0, 0), (10, 0)), font=self.font_menu, key="-SP_CONTROL-"), sg.Text("ATTITUDE CONTROL",
                                                                                                                                                                                         pad=((0, 0), (10, 0)), font=self.font_menu, visible=False, key="-AT_CONTROL-"), sg.Text("POSE CONTROL", pad=((0, 0), (10, 0)), font=self.font_menu, visible=False, key="-POS_CONTROL-")],
                       [sg.Column(col1_layout, element_justification='left'), sg.Column(col2_layout, element_justification='left', pad=((0, 190), (0, 0))),
                        sg.Column(col3_layout, element_justification='left', justification="left"), sg.Column(col4_layout, element_justification='left', justification="left", key="-COL4-"), sg.Column(col5_layout, element_justification='left', visible=False, key="-COL5-"), sg.Column(col7_layout, element_justification='left', visible=False, key="-COL7-")],
                       [sg.Text("TELEOPERATION MODE SELECTION", pad=((10, 100), (10, 0)), font=self.font_menu), sg.Text(
                           "POSE CONTROL", pad=((0, 0), (10, 0)), font=self.font_menu, key="-P_CONTROL-", visible=False)],
                       [sg.Column(col_button_layout, element_justification='left', pad=((0, 270), (0, 0))), sg.Column(
                           col8_layout, element_justification='left', key="-COL8-"), sg.Column(col6_layout, element_justification='left', key="-COL6-", visible=False), sg.Column(col6B_layout, element_justification='left', key="-COL6B-")],
                       [sg.HSeparator(pad=(0, 10))],
                       [sg.Text("Last key pressed:", font=self.font_menu), sg.Text("", font=self.font_menu, key="-key_pressed-")]]

        return sg.Window("Keyboard Teleoperation: " + self.uav.drone_id, self.layout, return_keyboard_events=True, use_default_focus=True, resizable=True)

    def make_settings_window(self):
        col_value_settings_layout=[[sg.Text("Speed Control Values", font=self.font_menu)],
                [sg.Text("Speed value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[0])), font=self.font, key="-VALUE0-", size=(5, 3), background_color="white"), sg.Text("m/s", font=self.font)],
                [sg.Text("Vertical value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[1])), font=self.font, key="-VALUE1-", size=(5, 3), background_color="white"), sg.Text("m/s", font=self.font)],
                [sg.Text("Turn speed value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[2])), font=self.font, key="-VALUE2-", size=(5, 3), background_color="white"), sg.Text("rad/s", font=self.font)],
                [sg.Text(
                    "", font=self.font)],
                [sg.Text(
                    "Position control values:", font=self.font_menu)],
                [sg.Text("Position value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[3])), font=self.font, key="-VALUE3-", size=(5, 3), background_color="white"), sg.Text("m", font=self.font)],
                [sg.Text("Altitude value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[4])), font=self.font, key="-VALUE4-", size=(5, 3), background_color="white"), sg.Text("m", font=self.font)],
                [sg.Text("Turn angle value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[5])), font=self.font, key="-VALUE5-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
                [sg.Text(
                    "", font=self.font)],
                [sg.Text(
                    "Attitude control values:", font=self.font_menu)],
                [sg.Text("Pitch angle value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[6])), font=self.font, key="-VALUE6-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
                [sg.Text("Roll angle value:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[7])), font=self.font, key="-VALUE7-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
                [sg.Text("Attitude duration:", font=self.font), sg.InputText(str(
                    "{:0.2f}".format(self.value_list[8])), font=self.font, key="-VALUE8-", size=(5, 3), background_color="white"), sg.Text("s", font=self.font)],
                [sg.Text(
                    "", font=self.font)],
                [sg.Button("Save", font=self.font), sg.Button("Exit", font=self.font, pad=((150, 0), (0, 0)))]]

        return sg.Window("Settings: "+ self.uav.drone_id, layout=[[sg.Column(col_value_settings_layout)]], location=self.window.CurrentLocation(), use_default_focus=False)

    def make_localization_window(self):

        return sg.Window("Localization: "+ self.uav.drone_id, location=self.window.CurrentLocation(), use_default_focus=False, size=(330, 200),
                layout=[[sg.Text("Position", font=self.font_menu)],
                        [sg.Text("x:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav.position[0], 2)), font=self.font, key="-LOCALIZATION_X-"), sg.Text(",", font=self.font),
                        sg.Text("y:", font=self.font), sg.Text("{:0.2f}".format(round(
                            self.uav.position[1], 2)), font=self.font, key="-LOCALIZATION_Y-"), sg.Text(",", font=self.font),
                        sg.Text("z:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav.position[2], 2)), font=self.font, key="-LOCALIZATION_Z-")],
                        [sg.Text(
                            "Orientation", font=self.font_menu)],
                        [sg.Text("r:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav.orientation[0], 2)), font=self.font, key="-LOCALIZATION_R-"), sg.Text(",", font=self.font),
                        sg.Text("p:", font=self.font), sg.Text("{:0.2f}".format(round(
                            self.uav.orientation[1], 2)), font=self.font, key="-LOCALIZATION_P-"), sg.Text(",", font=self.font),
                        sg.Text("y:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav.orientation[2], 2)), font=self.font, key="-LOCALIZATION_YW-")],
                        [sg.Button("Exit", font=self.font, pad=((240, 0), (20, 0)))]])

    # FUNCTIONS THAT MANAGE THE CALLS TO DRONE INTERFACES FUNCTIONS GIVEN A KB INPUT

    def manage_common_behaviors(self, window: sg.Window, input):

        if (input[0] == "t"):
            window["-key_pressed-"].update(value=input[0])

            try:
                threading.Thread(target=self.take_off, daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "l"):
            window["-key_pressed-"].update(value=input[0])

            try:
                threading.Thread(target=self.land, daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "space"):
            window["-key_pressed-"].update(value=input[0])

            try:
                threading.Thread(target=self.hover, daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        if input[0] == "Delete":
            window["-key_pressed-"].update(value=input[0])

            try:
                threading.Thread(
                    target=self.emergency_stop, daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

    def manage_speed_behaviors(self, window: sg.Window, input):

        if (input[0] == "Up"):
            window["-key_pressed-"].update(value="↑")
            lineal = [self.value_list[0], 0.0, 0.0]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, 0.0,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "Down"):
            window["-key_pressed-"].update(value="↓")
            lineal = [-self.value_list[0], 0.0, 0.0]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, 0.0,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "Left"):
            window["-key_pressed-"].update(value="←")
            lineal = [0.0, self.value_list[0], 0.0]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, 0.0,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "Right"):
            window["-key_pressed-"].update(value="→")
            lineal = [0.0, -self.value_list[0], 0.0]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, 0.0,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        if (input[0] == "w"):
            window["-key_pressed-"].update(value=input[0])
            lineal = [0.0, 0.0, self.value_list[1]]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, 0.0,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "s"):
            window["-key_pressed-"].update(value=input[0])
            lineal = [0.0, 0.0, -self.value_list[1]]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, 0.0,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "a"):
            window["-key_pressed-"].update(value=input[0])
            lineal = [0.0, 0.0, 0.0]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, self.value_list[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "d"):
            window["-key_pressed-"].update(value=input[0])
            lineal = [0.0, 0.0, 0.0]

            try:
                threading.Thread(target=self.move_at_speed, args=(
                    lineal, -self.value_list[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

    def manage_pose_behaviors(self, window: sg.Window, input):

        if (input[0] == "Up"):
            window["-key_pressed-"].update(value="↑")
            position = [self.uav.position[0] + self.value_list[3],
                        self.uav.position[1], self.uav.position[2]]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, self.uav.orientation[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "Down"):
            window["-key_pressed-"].update(value="↓")
            position = [self.uav.position[0] - self.value_list[3],
                        self.uav.position[1], self.uav.position[2]]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, self.uav.orientation[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "Left"):
            window["-key_pressed-"].update(value="←")
            position = [self.uav.position[0], self.uav.position[1] +
                        self.value_list[3], self.uav.position[2]]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, self.uav.orientation[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "Right"):
            window["-key_pressed-"].update(value="→")
            position = [self.uav.position[0], self.uav.position[1] -
                        self.value_list[3], self.uav.position[2]]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, self.uav.orientation[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "w"):
            window["-key_pressed-"].update(value=input[0])
            position = [self.uav.position[0], self.uav.position[1],
                        self.uav.position[2] + self.value_list[4]]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, self.uav.orientation[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "s"):
            window["-key_pressed-"].update(value=input[0])
            position = [self.uav.position[0], self.uav.position[1],
                        self.uav.position[2] - self.value_list[4]]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, self.uav.orientation[2],), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "a"):
            window["-key_pressed-"].update(value=input[0])
            position = [self.uav.position[0],
                        self.uav.position[1], self.uav.position[2]]
            euler = self.uav.orientation
            yaw = euler[2] + self.value_list[5]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, yaw,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

        elif (input[0] == "d"):
            window["-key_pressed-"].update(value=input[0])
            position = [self.uav.position[0],
                        self.uav.position[1], self.uav.position[2]]
            euler = self.uav.orientation
            yaw = euler[2] - self.value_list[5]

            try:
                threading.Thread(target=self.go_to_pose, args=(
                    position, yaw,), daemon=True).start()
            except Exception as e:
                print('Error starting work thread.')

    # FUNCTIONS TO CALL THE DRONE INTERFACES FUNCTIONS

    def shutdown(self):
        self.t.join()

    def take_off(self):
        self.uav.arm()
        self.uav.offboard()
        self.uav.takeoff(1.0, 1.0)

    def land(self):
        self.uav.land(0.5)

    def hover(self):
        self.uav.send_hover()

    def move_at_speed(self, lineal, yaw_speed):
        self.uav.speed_motion_handler.send_speed_command_with_yaw_speed(
            lineal, self.twist_frame_id, yaw_speed)

    def go_to_pose(self, position, orientation):
        self.uav.position_motion_handler.send_position_command_with_yaw_angle(
            position, None, self.pose_frame_id, self.twist_frame_id, orientation)

    def emergency_stop(self):
        self.uav.send_emergency_killswitch_to_aircraft()


if __name__ == '__main__':
    main()
