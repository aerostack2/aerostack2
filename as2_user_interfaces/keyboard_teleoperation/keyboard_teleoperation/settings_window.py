import PySimpleGUI as sg


class SettingsWindow(sg.Window):
    def __init__(self, font, menu_font, value_list, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.value_list = value_list
        self.font = font
        self.menu_font = menu_font

    def make_settings_window(self, location):
        self.Location = location
        col_value_settings_layout = [[sg.Text("Speed Control Values", font=self.menu_font)],
                                     [sg.Text("Speed value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[0])), font=self.font,
                                         key="-VALUE0-", size=(5, 3), background_color="white"), sg.Text("m/s", font=self.font)],
                                     [sg.Text("Vertical value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[1])), font=self.font,
                                         key="-VALUE1-", size=(5, 3), background_color="white"), sg.Text("m/s", font=self.font)],
                                     [sg.Text("Turn speed value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[2])), font=self.font,
                                         key="-VALUE2-", size=(5, 3), background_color="white"), sg.Text("rad/s", font=self.font)],
                                     [sg.Text(
                                         "", font=self.font)],
                                     [sg.Text(
                                         "Position control values:", font=self.menu_font)],
                                     [sg.Text("Position value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[3])), font=self.font,
                                         key="-VALUE3-", size=(5, 3), background_color="white"), sg.Text("m", font=self.font)],
                                     [sg.Text("Altitude value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[4])), font=self.font,
                                         key="-VALUE4-", size=(5, 3), background_color="white"), sg.Text("m", font=self.font)],
                                     [sg.Text("Turn angle value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[5])), font=self.font,
                                         key="-VALUE5-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
                                     [sg.Text(
                                         "", font=self.font)],
                                     [sg.Text(
                                         "Attitude control values:", font=self.menu_font)],
                                     [sg.Text("Pitch angle value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[6])), font=self.font,
                                         key="-VALUE6-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
                                     [sg.Text("Roll angle value:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[7])), font=self.font,
                                         key="-VALUE7-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
                                     [sg.Text("Attitude duration:", font=self.font), sg.InputText(str(
                                         "{:0.2f}".format(self.value_list[8])), font=self.font,
                                         key="-VALUE8-", size=(5, 3), background_color="white"), sg.Text("s", font=self.font)],
                                     [sg.Text(
                                         "", font=self.font)],
                                     [sg.Button("Save", font=self.font), sg.Button("Exit", font=self.font, pad=((150, 0), (0, 0)))]]

        self.layout([[sg.Column(col_value_settings_layout)]])

    def event_handler(self, window: sg.Window, settings_event, settings_value):
        numeric_values = list(settings_value.values())[0:len(self.value_list)]
        for idx, value in enumerate(list(numeric_values)):
            try:
                self.value_list[idx] = float(value)
            except ValueError:
                print("Invalid Input, setting to 0.0")
                self.value_list[idx] = 0.0
                self["-VALUE" +
                     str(idx) + "-"].update(value="{:0.2f}".format(0.00))

        if settings_event == sg.WIN_CLOSE_ATTEMPTED_EVENT or settings_event == "Exit":
            self.hide()
            return self.value_list, False

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

                self["-VALUE" + str(idx) +
                     "-"].update(value="{:0.2f}".format(value))

            return self.value_list, True

        return self.value_list, True
