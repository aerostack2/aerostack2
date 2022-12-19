"""Behavior manager."""

from python_interface.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface
from as2_keyboard_teleoperation.config_values import AvailableBehaviors


class BehaviorManager:
    """Handle behavior control."""

    def __init__(self, uav_list: list[DroneInterface], drone_id_list):
        self.uav_list = uav_list
        self.drone_id_list = drone_id_list

    def manage_behavior_control(self, behavior_list, order):
        """
        Make de calls to behavior methods.

        :param behavior_list: list of behaviors to be controlled
        :type behavior_list: list(string)
        :param order: order to be taken uppon behavior list
        :type order: string
        """
        if AvailableBehaviors.BEHAVIOR_TAKE_OFF.value in behavior_list:
            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    if order == "-PAUSE_BEHAVIORS-":
                        self.uav_list[index].takeoff.pause()
                    elif order == "-RESUME_BEHAVIORS-":
                        self.uav_list[index].takeoff.resume()

        if AvailableBehaviors.BEHAVIOR_LAND.value in behavior_list:
            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    if order == "-PAUSE_BEHAVIORS-":
                        self.uav_list[index].land.pause()
                    elif order == "-RESUME_BEHAVIORS-":
                        self.uav_list[index].land.resume()

        if AvailableBehaviors.BEHAVIOR_FOLLOW_PATH.value in behavior_list:
            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    if order == "-PAUSE_BEHAVIORS-":
                        self.uav_list[index].follow_path.pause()
                    elif order == "-RESUME_BEHAVIORS-":
                        self.uav_list[index].follow_path.resume()

        if AvailableBehaviors.BEHAVIOR_GO_TO.value in behavior_list:
            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    if order == "-PAUSE_BEHAVIORS-":
                        self.uav_list[index].goto.pause()
                    elif order == "-RESUME_BEHAVIORS-":
                        self.uav_list[index].goto.resume()
