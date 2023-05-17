"""
mission_interpreter.py
"""

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


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import time
from threading import Thread
from collections import deque

from as2_python_api.drone_interface import DroneInterfaceBase
from as2_python_api.mission_interpreter.mission import Mission

# TODO: improve mission_stack
# Class MissionStack:
#       attributtes: current, done_deque, todo_deque
#       methods: append, insert, repeat_last


class MissionInterpreter:
    """Mission Interpreter and Executer
    """

    def __init__(self, mission: Mission, use_sim_time: bool = False) -> None:
        self._mission = mission
        self._use_sim_time = use_sim_time

        self._drone = None
        self._mission_stack = None
        self.performing = False

        self.exec_thread = None
        self.current_behavior = None
        self.stopped = False

        self.last_mission_item = None

        self.status = None

    def __del__(self) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        """Shutdown properly"""
        self.drone.shutdown()
        self.stopped = True
        if self.exec_thread:
            self.exec_thread.join()

    @property
    def drone(self) -> DroneInterfaceBase:
        """
        Build a DroneInterface based on the mission requirements
        """
        if not self._drone:
            needed_modules = {item.behavior for item in self._mission.plan}
            drone = DroneInterfaceBase(
                drone_id=self._mission.target,
                verbose=self._mission.verbose,
                use_sim_time=self._use_sim_time
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

    # TODO: mission managment should return boolean value
    def start_mission(self) -> None:
        """Start mission in different thread"""
        self.exec_thread = Thread(
            target=self.perform_mission, args=[True])  # TODO
        self.exec_thread.start()

    def stop_mission(self) -> None:
        """Stop mission"""
        if self.exec_thread:
            print("trying to stop")
            self.stopped = True
            self.current_behavior.stop()

    def next_item(self) -> None:
        """Advance to next item in mission"""
        if self.exec_thread:
            self.current_behavior.stop()

    def pause_mission(self) -> None:
        """Pause mission"""
        raise NotImplementedError  # TODO

    def resume_mission(self) -> None:
        """Resume mission"""
        raise NotImplementedError  # TODO

    def modify_current(self) -> None:
        """Modify current item in mission"""
        raise NotImplementedError
    
    def append_mission(self, mission: Mission) -> None:
        """Insert mission at the end of the stack"""
        self._mission_stack.extend(mission.stack)
    
    def insert_mission(self, mission: Mission) -> None:
        """Insert mission in front of the stack"""
        self._mission_stack.appendleft(self.last_mission_item)
        stack = mission.stack
        stack.reverse()
        self._mission_stack.extendleft(stack)
        self.next_item()

    def perform_mission(self, debug=False) -> None:
        """
        Perform a mission
        """

        self.status = -1

        if self.performing:
            print("Already performing a mission")
            return
        self.performing = True

        # TODO: drone setup?
        if not debug:
            self.drone.arm()
            self.drone.offboard()

        while self.mission_stack and not self.stopped:
            self.last_mission_item = self.mission_stack.popleft()  # get first in
            behavior, args = self.last_mission_item
            self.current_behavior = getattr(self.drone, behavior)
            self.status += 1
            self.current_behavior(*args)

        self.exec_thread = False
        self.performing = False

        if not self.stopped:
            self.drone.shutdown()

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

    def reset(self, mission: Mission) -> None:
        """Reset Mission Interpreter with other mission"""
        self.stop_mission()
        if self.exec_thread:
            self.exec_thread.join()

        self._mission = mission

        self._drone = None
        self._mission_stack = None
        self.performing = False

        self.exec_thread = None
        self.current_behavior = None
        self.stopped = False


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

    import rclpy
    rclpy.init()
    interpreter = MissionInterpreter(mission)
    interpreter.start_mission()
    time.sleep(3)
    interpreter.next_item()
    interpreter.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    # import doctest
    # doctest.testmod()

    test()
