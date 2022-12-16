import threading
from python_interface.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface
from config_values import KeyMappings


class DroneManager:
    def __init__(self, uav_list: list[DroneInterface],
                 drone_id_list, pose_frame_id, twist_frame_id):
        self.uav_list = uav_list
        self.drone_id_list = drone_id_list
        self.pose_frame_id = pose_frame_id
        self.twist_frame_id = twist_frame_id
        self.value_common_behavior = {KeyMappings.TAKE_OFF_KEY.value: self.take_off,
                                      KeyMappings.LAND_KEY.value: self.land,
                                      KeyMappings.HOVER_KEY.value: self.hover,
                                      KeyMappings.EMERGENCY_KEY.value: self.emergency_stop}

    def manage_common_behaviors(self, key):

        if key in self.value_common_behavior:
            self.execute_common_behaviors(self.value_common_behavior[key])

    def manage_speed_behaviors(self, key, value_list):

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

                    lineal = [0.0, value_list[0], 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index], lineal, 0.0,))

        elif key == KeyMappings.LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, -value_list[0], 0.0]
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
                                             lineal, value_list[2],))

        elif key == KeyMappings.ROTATE_LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:

                    lineal = [0.0, 0.0, 0.0]
                    self.execute_function(
                        self.move_at_speed, (self.uav_list[index],
                                             lineal, -value_list[2],))

    def manage_pose_behaviors(self, key, value_list):

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
                                self.uav_list[index].position[1] +
                                value_list[3],
                                self.uav_list[index].position[2]]
                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position,
                                          self.uav_list[index].orientation[2],))

        elif key == KeyMappings.LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1] -
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
                    yaw = euler[2] + value_list[5]

                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position, yaw,))

        elif key == KeyMappings.ROTATE_LEFT_KEY.value:

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1]:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1],
                                self.uav_list[index].position[2]]
                    euler = self.uav_list[index].orientation
                    yaw = euler[2] - value_list[5]

                    self.execute_function(
                        self.go_to_pose, (self.uav_list[index], position, yaw,))

    def execute_common_behaviors(self, method):
        for index, drone_id in enumerate(self.drone_id_list):
            if drone_id[1]:
                self.execute_function(method, (self.uav_list[index],))

    def execute_function(self, target, args):
        try:
            threading.Thread(target=target, args=args, daemon=True).start()
        except Exception as ex:
            print('Error starting work thread: ', ex)

    # FUNCTIONS TO CALL THE DRONE INTERFACES FUNCTIONS

    # def shutdown(self):
        # self.t.join()

    def take_off(self, uav: DroneInterface):
        uav.arm()
        uav.offboard()
        uav.takeoff(1.0, 1.0)

    def land(self, uav: DroneInterface):
        uav.land(0.5)

    def hover(self, uav: DroneInterface):
        uav.motion_ref_handler.hover()

    def move_at_speed(self, uav: DroneInterface, lineal, yaw_speed):
        uav.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
            lineal, self.twist_frame_id, yaw_speed)

    def go_to_pose(self, uav: DroneInterface, position, orientation):
        uav.motion_ref_handler.position.send_position_command_with_yaw_angle(
            position, None, self.pose_frame_id,
            self.twist_frame_id, orientation)

    def emergency_stop(self, uav: DroneInterface):
        uav.send_emergency_killswitch_to_aircraft()
