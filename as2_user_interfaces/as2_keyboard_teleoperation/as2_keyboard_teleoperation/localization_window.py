"""Localization window."""
import PySimpleGUI as sg
from python_interface.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface


class LocalizationWindow(sg.Window):
    """
    Display the localization of the drones.

    :param sg: Window class of PySimpleGUI
    :type sg: PySimpleGUI Window
    """

    def __init__(self, font, menu_font, uav_list: list[DroneInterface], *args, **kwargs):
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

        drone_frame_list_layout = list()

        for index, uav in enumerate(self.uav_list):

            drone_localization_layout = [[
                sg.Text("Position", font=self.menu_font,
                        key="-POSITION_" + str(index) + "-")],
                [sg.Text("x:", font=self.font),
                 sg.Text(f"{round(uav.position[0], 2):.2f}",
                         font=self.font, key="-LOCALIZATION_X_" + str(index) + "-"),
                 sg.Text(",", font=self.font),
                 sg.Text("y:", font=self.font),
                 sg.Text(f"{round(uav.position[1], 2):.2f}", font=self.font,
                         key="-LOCALIZATION_Y_" + str(index) + "-"),
                 sg.Text(",", font=self.font),
                 sg.Text("z:", font=self.font),
                 sg.Text(f"{round(uav.position[2], 2):.2f}",
                 font=self.font, key="-LOCALIZATION_Z_" + str(index) + "-")],
                [sg.Text("Orientation", font=self.menu_font,
                         key="-ORIENTATION_" + str(index) + "-")],
                [sg.Text("r:", font=self.font),
                 sg.Text(f"{round(uav.orientation[0], 2):.2f}",
                         font=self.font, key="-LOCALIZATION_R_" + str(index) + "-"),
                 sg.Text(",", font=self.font),
                 sg.Text("p:", font=self.font),
                 sg.Text(f"{round(uav.orientation[1], 2):.2f}", font=self.font,
                         key="-LOCALIZATION_P_" + str(index) + "-"),
                 sg.Text(",", font=self.font),
                 sg.Text("y:", font=self.font),
                 sg.Text(f"{round(uav.orientation[2], 2):.2f}",
                         font=self.font, key="-LOCALIZATION_YW_" + str(index) + "-")]]

            localization_frame = sg.Frame(
                uav.get_namespace(), layout=drone_localization_layout)
            if index % 2 == 0:
                drone_frame_list_layout.append([localization_frame])
            else:
                drone_frame_list_layout[-1].append(localization_frame)

        localization_column = sg.Column(layout=drone_frame_list_layout,
                                        scrollable=True, vertical_scroll_only=True,
                                        size=(530, 500), sbar_trough_color="white",
                                        sbar_arrow_color="grey")

        localization_layout = [[localization_column],
                               [sg.Button("Exit", font=self.font, pad=((500, 0), (20, 0)))]]

        self.layout(localization_layout)

    def execute_localization_window(self):
        """
        Update the localization window visual output.

        :return: Opened or not
        :rtype: bool
        """
        localization_event, _ = self.read(timeout=1)  # type: ignore

        for index, uav in enumerate(self.uav_list):

            self["-LOCALIZATION_X_" + str(index) + "-"].update(
                value=f"{round(uav.position[0], 2):.2f}")
            self["-LOCALIZATION_Y_" + str(index) + "-"].update(
                value=f"{round(uav.position[1], 2):.2f}")
            self["-LOCALIZATION_Z_" + str(index) + "-"].update(
                value=f"{round(uav.position[2], 2):.2f}")
            self["-LOCALIZATION_R_" + str(index) + "-"].update(
                value=f"{round(uav.orientation[0], 2):.2f}")
            self["-LOCALIZATION_P_" + str(index) + "-"].update(
                value=f"{round(uav.orientation[1], 2):.2f}")
            self["-LOCALIZATION_YW_" + str(index) + "-"].update(
                value=f"{round(uav.orientation[2], 2):.2f}")

        # self.refresh()
        if localization_event == sg.WIN_CLOSE_ATTEMPTED_EVENT or localization_event == "Exit":
            self.hide()
            return False

        return True
