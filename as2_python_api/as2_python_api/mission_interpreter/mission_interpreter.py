"""Mission Interpreter and Executer."""

# Copyright 2025 Universidad Politécnica de Madrid
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

__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import logging
from threading import Thread
import time

from as2_msgs.msg import BehaviorStatus
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_python_api.drone_interface import DroneInterfaceBase
from as2_python_api.mission_interpreter.mission import InterpreterStatus, Mission, MissionItem
from as2_python_api.mission_interpreter.mission_stack import MissionStack
from rclpy.executors import Executor, SingleThreadedExecutor

logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s',
    datefmt='%s',
)


class MissionInterpreter:
    """Mission Interpreter and Executer."""

    def __init__(
        self,
        use_sim_time: bool = False,
        verbose: bool = False,
        executor: Executor = SingleThreadedExecutor,
    ) -> None:
        self._verbose = verbose
        self._logger = logging.getLogger('MissionInterpreter')
        self._logger.setLevel(logging.DEBUG if verbose else logging.INFO)

        self._missions: dict[int, Mission] = {}
        self._current_mid: int = None
        self._mission_stack: MissionStack = None

        self._use_sim_time: bool = use_sim_time
        self._executor: Executor = executor
        self._drone: DroneInterfaceBase = None

        self.exec_thread: Thread = None
        self.current_behavior: BehaviorHandler = None
        self.stopped: bool = False

        self._logger.debug('Mission interpreter ready')

    def __del__(self) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        """Shutdown properly."""
        self.drone.shutdown()
        self.stopped = True
        if self.exec_thread:
            self.exec_thread.join()
        self._logger.info('Shutdown')

    @property
    def mission(self) -> Mission:
        """Mission."""
        return self._missions.get(self._current_mid, None)

    @mission.setter
    def mission(self, mid: int) -> None:
        """Set mission to be executed by ID."""
        mission = self._missions.get(mid, None)

        if mission is None:
            self._logger.error(f'Mission {mid} not found, please load it first')
            return

        if self._current_mid != mid:
            self._current_mid = mid
            self._mission_stack = None

    @property
    def drone(self) -> DroneInterfaceBase:
        """Build a DroneInterface based on the mission requirements."""
        if self.mission is None:
            return None
        if not self._drone or self._drone.drone_id != self.mission.target:
            drone = DroneInterfaceBase(
                drone_id=self.mission.target,
                verbose=self._verbose,
                use_sim_time=self._use_sim_time,
                executor=self._executor,
            )
            self._drone = drone
            self.load_modules(self.mission)
        return self._drone

    def load_modules(self, mission: Mission):
        needed_modules = {item.behavior for item in mission.plan}
        for module_name in needed_modules.difference(set(self.drone.modules)):
            print(f'module {module_name} loaded')
            self.drone.load_module(f'{module_name}_module')

    @property
    def mission_stack(self) -> MissionStack:
        """Mission stack."""
        if self.mission is None:
            return None
        if self._mission_stack is None:
            self._mission_stack = self.mission.stack
        return self._mission_stack

    @property
    def status(self) -> InterpreterStatus:
        """Mission status."""
        state = BehaviorStatus.IDLE
        if self.mission is None:
            return InterpreterStatus()
        if self.current_behavior is not None:
            state = self.current_behavior.status

        return InterpreterStatus(
            state=state,
            pending_items=len(self.mission_stack.pending),
            done_items=len(self.mission_stack.done),
            current_item=self.mission_stack.current,
            feedback_current=self.feedback_dict,
        )

    @property
    def feedback(self):
        """Get current behavior feedback."""
        return None if self.current_behavior is None else self.current_behavior.feedback

    @property
    def feedback_dict(self):
        """Get current behavior feedback dictionary."""
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

    def load_mission(self, mid: int, mission: Mission) -> None:
        """Reset Mission Interpreter with other mission."""
        self._missions[mid] = mission
        if self._current_mid is None:
            self.mission = mid
        self.drone  # Load drone
        self.load_modules(mission)

        self._logger.info(f'Mission {mid} loaded')
        self._logger.info(self._missions)

    def start_mission(self, mid: int) -> bool:
        """Start mission in different thread."""
        if self.exec_thread:
            self._logger.warning('Mission being performed, start not allowed')
            return False
        self.mission = mid
        self.exec_thread = Thread(target=self.__perform_mission)
        self.exec_thread.start()
        return True

    def stop_mission(self, mid: int) -> bool:
        """Stop mission."""
        if not self.exec_thread:
            self._logger.debug('No mission being executed, already stopped')
            return True
        if mid != self._current_mid:
            self._logger.error('Stop requested for another mission, not the one executing')
            return False
        self.stopped = True
        return self.current_behavior.stop()

    def next_item(self, mid: int) -> bool:
        """Advance to next item in mission."""
        if not self.exec_thread:
            self._logger.warning('No mission being executed, next item not allowed')
            return False
        if mid != self._current_mid:
            self._logger.error('Next item requested for another mission, not the one executing')
            return False
        return self.current_behavior.stop()

    def pause_mission(self, mid: int) -> bool:
        """Pause mission."""
        if not self.exec_thread:
            self._logger.warning('No mission being executed, pause not allowed')
            return False
        if mid != self._current_mid:
            self._logger.error('Pause requested for another mission, not the one executing')
            return False
        return self.current_behavior.pause()

    def resume_mission(self, mid: int) -> bool:
        """Resume mission."""
        if not self.exec_thread:
            self._logger.warning('No mission being executed, resume not allowed')
            return False
        if mid != self._current_mid:
            self._logger.error('Resume requested for another mission, not the one executing')
            return False
        return self.current_behavior.resume(wait_result=False)

    def modify(self, idx: int, mid: int, item: MissionItem) -> bool:
        """
        Modify mission item at index with another MissionItem.

        :param idx: index of the item to modify
        :type idx: int
        :param mid: mission ID
        :type mid: int
        :param item: MissionItem to modify from
        :type item: MissionItem
        :return: True if modified, False otherwise
        :rtype: bool
        """
        if mid == self._current_mid and self.mission_stack.current_idx == idx:
            return self.current_behavior.modify(**item.args)
        else:
            mission: Mission = self._missions.get(mid, None)
            if mission is None:
                print('Mission does not exist')
                self._logger.error(f'Mission {mid} does not exist')
                return False
            valid: bool = mission.modify(idx, item)
            if not valid:
                self._logger.error(f'Failed to modify mission {mid} at index {idx}')
                return False

            return valid

    def append_mission(self, mid: int, mission: Mission) -> None:
        """Insert mission at the end of the stack."""
        if mid != self._current_mid:
            self._logger.error('Append requested for another mission, not the one executing')
            return
        self._mission_stack.add(mission.stack)

    def insert_mission(self, mid: int, mission: Mission) -> None:
        """Insert mission in front of the stack."""
        if mid != self._current_mid:
            self._logger.error('Insert requested for another mission, not the one executing')
            return
        self._mission_stack.insert(mission.stack)

    def __perform_mission(self) -> None:
        """Perform a mission."""
        while self.mission_stack.pending and not self.stopped:
            mission_item = self.mission_stack.next_item()
            behavior = mission_item.behavior
            method = mission_item.method
            args = mission_item.args

            self.current_behavior = getattr(self.drone, behavior)
            current_method = getattr(self.current_behavior, method)
            try:
                # TODO(pariaspe): check current_method result
                res = current_method(**args)
                self._logger.debug(f'Behavior {behavior} method {method} result: {res}')
            except BehaviorHandler.GoalRejected:
                self._logger.error(f'Goal rejected by behavior {behavior}')
                break
            except BehaviorHandler.BehaviorNotAvailable:
                self._logger.error(f'Behavior {behavior} not available')
                break

        self.mission_stack.next_item()  # current done or stopped

        self.exec_thread = False
        self.current_behavior = None

    def reset(self, mid: int, mission: Mission) -> None:
        """Reset Mission Interpreter with other mission."""
        self.stop_mission(self._current_mid)
        if self.exec_thread:
            self.exec_thread.join()

        self._mission_stack = None

        self.exec_thread = None
        self.current_behavior = None
        self.stopped = False

        self._current_mid = None
        self.load_mission(mid, mission)

    # def abort_mission(self):
    #     """Abort current mission, and start safety mission."""
    #     if self._abort_mission is None:
    #         self._logger.critical(
    #             'Abort command received but not abort mission available. ' +
    #             'Change to manual control!')
    #         return

    #     self.reset(self._abort_mission)
    #     try:
    #         self.start_mission()
    #     except AttributeError:
    #         self._logger.error('Trying to start mission but no mission is loaded.')


def test():
    """
    A doctest in a docstring.

    >>> test()
    test called with height=1.0, speed=2.0 and wait=True
    test called with height=98.0, speed=99.0 and wait=True
    """
    dummy_mission = """
    {
        "target": "drone_0",
        "plan": [
            {
                "behavior": "dummy",
                "args": {
                    "arg1": 1.0,
                    "arg2": 2.0,
                    "wait": "True"
                }
            },
            {
                "behavior": "dummy",
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
    interpreter = MissionInterpreter(verbose=True)
    interpreter.load_mission(0, mission)
    interpreter.start_mission(0)
    time.sleep(3)
    interpreter.next_item()
    interpreter.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    # import doctest
    # doctest.testmod()

    test()
