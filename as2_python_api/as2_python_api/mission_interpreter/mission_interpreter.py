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
import logging
from threading import Thread

from as2_python_api.drone_interface import DroneInterfaceBase
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_python_api.mission_interpreter.mission import Mission, InterpreterStatus
from as2_python_api.mission_interpreter.mission_stack import MissionStack

logging.basicConfig(level=logging.INFO,
                    format="[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s",
                    datefmt='%s')


class MissionInterpreter:
    """Mission Interpreter and Executer
    """

    # TODO: mission default None -> default values to drone and mission_stack properties
    def __init__(self, mission: Mission = None, use_sim_time: bool = False) -> None:
        self._logger = logging.getLogger("MissionInterpreter")

        self._mission: Mission = mission
        self._use_sim_time: bool = use_sim_time

        self._drone: DroneInterfaceBase = None
        self._mission_stack: MissionStack = None
        self.performing: bool = False

        self.exec_thread: Thread = None
        self.current_behavior: BehaviorHandler = None
        self.stopped: bool = False
        self.paused: bool = False

        self._logger.debug("Mission interpreter ready")

    def __del__(self) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        """Shutdown properly"""
        self.drone.shutdown()
        self.stopped = True
        if self.exec_thread:
            self.exec_thread.join()
        self._logger.info("Shutdown")

    @property
    def drone(self) -> DroneInterfaceBase:
        """
        Build a DroneInterface based on the mission requirements
        """
        if self._mission is None:
            return None
        if not self._drone:
            needed_modules = {item.behavior for item in self._mission.plan}
            drone = DroneInterfaceBase(
                drone_id=self._mission.target,
                verbose=self._mission.verbose,
                use_sim_time=self._use_sim_time
            )

            for module_name in needed_modules:
                print(f"module {module_name} loaded")
                drone.load_module(
                    f'{module_name}_module')
            self._drone = drone

        return self._drone

    @property
    def mission_stack(self) -> MissionStack:
        """Mission stack
        """
        if self._mission is None:
            return None
        if self._mission_stack is None:
            self._mission_stack = self._mission.stack
        return self._mission_stack

    @property
    def status(self) -> InterpreterStatus:
        """Mission status"""
        state = "IDLE"
        if self._mission is None:
            return InterpreterStatus()
        if self.performing:
            state = "RUNNING"
        # TODO: use current behavior internal status
        # if self.current_behavior.status == "PAUSED":
        if self.paused:
            state = "PAUSED"
        if self.stopped:
            state = "IDLE"

        return InterpreterStatus(state=state, pending_items=len(self.mission_stack.pending),
                                 done_items=len(self.mission_stack.done),
                                 current_item=self.mission_stack.current,
                                 feedback_current=self.feedback_dict)

    @property
    def feedback(self):
        """Current behavior feedback"""
        return None if self.current_behavior is None else self.current_behavior.feedback

    @property
    def feedback_dict(self):
        """Current behavior feedback dictionary"""
        feedback = None if self.current_behavior is None else self.current_behavior.feedback
        if feedback is None:
            return None

        fb_dict = {}
        if isinstance(feedback, dict):
            fb_dict = feedback
        else:
            for k, _ in feedback.get_fields_and_field_types().items():
                fb_dict[k] = getattr(feedback, k)
        return fb_dict

    def start_mission(self) -> bool:
        """Start mission in different thread"""
        if self.exec_thread:
            self._logger.warning(
                "Mission being performed, start not allowed")
            return False
        self.exec_thread = Thread(target=self.perform_mission)
        self.exec_thread.start()
        return True

    def stop_mission(self) -> bool:
        """Stop mission"""
        if not self.exec_thread:
            self._logger.debug("No mission being executed, already stopped")
            return True
        self.stopped = True
        return self.current_behavior.stop()

    def next_item(self) -> bool:
        """Advance to next item in mission"""
        if not self.exec_thread:
            self._logger.warning(
                "No mission being executed, next item not allowed")
            return False
        return self.current_behavior.stop()

    def pause_mission(self) -> bool:
        """Pause mission"""
        if not self.exec_thread:
            self._logger.warning(
                "No mission being executed, pause not allowed")
            return False
        self.paused = True
        return self.current_behavior.pause()

    def resume_mission(self) -> bool:
        """Resume mission"""
        if not self.exec_thread:
            self._logger.warning(
                "No mission being executed, resume not allowed")
            return False
        self.paused = False
        return self.current_behavior.resume(wait_result=False)

    def modify_current(self) -> bool:
        """Modify current item in mission"""
        raise NotImplementedError

    def append_mission(self, mission: Mission) -> None:
        """Insert mission at the end of the stack"""
        self._mission_stack.extend(mission.stack)

    # TODO: refactor to current mission_stack
    def insert_mission(self, mission: Mission) -> None:
        """Insert mission in front of the stack"""
        # self._mission_stack.appendleft(self.last_mission_item)
        stack = mission.stack
        stack.reverse()
        self._mission_stack.extendleft(stack)
        self.next_item()

    def perform_mission(self) -> None:
        """
        Perform a mission
        """

        if self.performing:
            self._logger.warning("Already performing a mission")
            return
        self.performing = True

        while self.mission_stack.pending and not self.stopped:
            mission_item = self.mission_stack.next()
            behavior = mission_item.behavior
            method = mission_item.method
            args = mission_item.args

            self.current_behavior = getattr(self.drone, behavior)
            current_method = getattr(self.current_behavior, method)
            try:
                current_method(**args)
            except BehaviorHandler.GoalRejected:
                self._logger.error(f"Goal rejected by behavior {behavior}")
                break
            except BehaviorHandler.BehaviorNotAvailable:
                self._logger.error(f"Behavior {behavior} not available")
                break

        self.mission_stack.next()  # current done or stopped

        self.exec_thread = False
        self.performing = False

        if not self.stopped:
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
        self.paused = False


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
