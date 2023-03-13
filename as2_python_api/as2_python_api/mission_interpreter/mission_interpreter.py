"""
mission_interpreter.py
"""

import time
from threading import Thread
from collections import deque
import rclpy

from as2_python_api.drone_interface import DroneInterfaceBase
from as2_python_api.mission_interpreter.mission import Mission


class MissionInterpreter:
    """Mission Interpreter and Executer
    """

    def __init__(self, mission: Mission) -> None:
        self._mission = mission

        self._drone = None
        self._mission_stack = None
        self.performing = False

        self.exec_thread = None
        self.current_behavior = None
        self.stopped = False

    @property
    def drone(self) -> DroneInterfaceBase:
        """
        Build a DroneInterface based on the mission requirements
        """
        if not self._drone:
            needed_modules = {item.behavior for item in self._mission.plan}
            drone = DroneInterfaceBase(
                drone_id=self._mission.target,
                verbose=self._mission.verbose
            )

            for module_name in needed_modules:
                drone.load_module(
                    f'as2_python_api.modules.{module_name}_module')
            self._drone = drone

        return self._drone

    @property
    def mission_stack(self) -> deque:
        """Mission stack
        """
        if self._mission_stack is None:
            self._mission_stack = self._mission.stack
        return self._mission_stack

    def start_mission(self) -> None:
        self.exec_thread = Thread(
            target=self.perform_mission, args=[True])
        self.exec_thread.start()

    def stop_mission(self) -> None:
        if self.exec_thread:
            print("trying to stop")
            self.stopped = True
            self.current_behavior.stop()

    def next_item(self) -> None:
        if self.exec_thread:
            self.current_behavior.stop()

    def perform_mission(self, debug=False) -> None:
        """
        Perform a mission
        """

        if self.performing:
            print("Already performing a mission")
            return
        self.performing = True

        # TODO: drone setup?
        if not debug:
            self.drone.arm()
            self.drone.offboard()

        while self.mission_stack and not self.stopped:
            behavior, args = self.mission_stack.popleft()  # get first in
            self.current_behavior = getattr(self.drone, behavior)
            self.current_behavior(*args)

        self.drone.shutdown()
        self.performing = False

    def poor_perform_mission(self):
        """POOR PRACTICE: do not use exec
        """
        import re
        for mission_item in self._mission.plan:
            plan = f"self.drone.{mission_item.behavior}("
            for name, value in mission_item.args.items():
                plan += f"{name}={value}, "
            plan = re.sub(', $', ')', plan)
            print(plan)
            exec(plan)
        self.drone.shutdown()


def test():
    """a doctest in a docstring

    >>> test()
    test called with height=1.0, speed=2.0 and wait=True
    test called with height=98.0, speed=99.0 and wait=True
    """

    dummy_mission = """
    {
        "target": "drone_0",
        "verbose": "False",
        "plan": [
            {
                "behavior": "test",
                "args": {
                    "arg1": 1.0,
                    "arg2": 2.0,
                    "wait": "True"
                }
            },
            {
                "behavior": "test",
                "args": {
                    "arg2": 98.0,
                    "arg1": 99.0,
                    "wait": "False"
                }
            }
        ]
    }"""
    mission = Mission.parse_raw(dummy_mission)

    rclpy.init()
    interpreter = MissionInterpreter(mission)
    interpreter.start_mission()
    time.sleep(3)
    interpreter.next_item()
    rclpy.shutdown()


if __name__ == "__main__":
    # import doctest
    # doctest.testmod()

    test()
