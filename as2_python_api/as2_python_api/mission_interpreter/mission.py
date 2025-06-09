"""Mission message definitions."""

from __future__ import annotations

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

from enum import IntEnum
import inspect
from typing import Any, List

try:
    from pydantic.v1 import BaseModel
except ModuleNotFoundError:
    from pydantic import BaseModel

from as2_msgs.msg import BehaviorStatus
from as2_python_api.mission_interpreter.mission_stack import MissionStack
from as2_python_api.tools.utils import get_module_call_signature


class InterpreterState(IntEnum):
    """Interpreter state."""

    IDLE = BehaviorStatus.IDLE
    RUNNING = BehaviorStatus.RUNNING
    PAUSED = BehaviorStatus.PAUSED


class MissionItem(BaseModel):
    """Mission Item data model. It represents a behavior call."""

    behavior: str
    method: str = '__call__'
    args: dict

    def __str__(self):
        return f'{self.behavior}: {self.method}: {self.args}'

    @property
    def args_extended(self) -> List:
        """Check if module exist and return full list of arguments, default."""
        signature = get_module_call_signature(self.behavior)

        args = []
        for param in signature.parameters:
            try:
                # if param found in mission, append it
                args.append(self.args[param])
            except KeyError as exc:
                # if param not found in mission
                if param == 'self':
                    pass
                elif signature.parameters[param].default != inspect.Parameter.empty:
                    # append default
                    args.append(signature.parameters[param].default)
                else:
                    raise exc
        return args

    def modify(self, other: MissionItem) -> bool:
        """
        Modify current item with another MissionItem.

        :param other: MissionItem to modify from
        :type other: MissionItem
        :return: True if modified, False otherwise
        :rtype: bool
        """
        # Check behavior is the same
        if self.behavior != other.behavior or self.method != other.method:
            print(
                f"{self.behavior} , {self.method} doesn't match {other.behavior}, {other.method}"
            )
            return False

        # Compare arguments and types
        seen: set[str] = set()
        for k in other.args:
            if k in seen or k not in self.args:
                print(f'Key {k} not found in args or duplicated in replacement item.')
                return False
            if not isinstance(other.args[k], type(self.args[k])):
                print(f'Key {k} type mismatch: {type(other.args[k])} != {type(self.args[k])}')
                return False
            seen.add(k)

        if len(seen) != len(self.args):
            print(f'Number of keys mismatch: {len(seen)} != {len(self.args)}')
            return False

        # Update args with the new ones
        self.args.update(other.args)

        return True


class Mission(BaseModel):
    """Mission data model."""

    target: str
    plan: List[MissionItem] = []

    @property
    def stack(self) -> MissionStack:
        """
        Return mission stack.

        :raises exc: if behavior arg doesn't exist
        :rtype: MissionStack
        """
        return MissionStack(mission_stack=self.plan)

    def modify(self, idx: int, item: MissionItem) -> bool:
        """
        Modify mission item at index with another MissionItem.

        :param idx: index of the item to modify
        :type idx: int
        :param item: MissionItem to modify from
        :type item: MissionItem
        :return: True if modified, False otherwise
        :rtype: bool
        """
        if idx < 0 or idx >= len(self.plan):
            return False

        return self.plan[idx].modify(item)

    def __str__(self):
        return self.json()


class InterpreterStatus(BaseModel):
    """Mission status."""

    state: InterpreterState = BehaviorStatus.IDLE
    pending_items: int = 0
    done_items: int = 0
    current_item: MissionItem = None
    feedback_current: Any = None

    @property
    def total_items(self) -> int:
        """Total amount of items in mission, done + current + pending."""
        count_current = 1
        if self.current_item is None:
            count_current = 0
        return self.done_items + count_current + self.pending_items

    def __str__(self):
        count_current = 1
        if self.current_item is None:
            count_current = 0
        s = (
            f'[{self.state}] [{self.done_items + count_current}/{self.total_items}] '
            + f'{self.current_item}'
        )
        return s

    def __eq__(self, other):
        """Override the default implementation, check all attributes except feedback."""
        if isinstance(other, InterpreterStatus):
            return (
                self.state == other.state
                and self.pending_items == other.pending_items
                and self.done_items == other.done_items
                and self.current_item == other.current_item
            )
        return False


if __name__ == '__main__':
    import unittest

    class TestMission(unittest.TestCase):
        """Mission testing."""

        def test_mission_model(self):
            """Two test dummy mission."""
            dummy_mission = """
            {
                "target": "drone_0",
                "plan": [
                    {
                        "behavior": "dummy",
                        "args": {
                            "arg1": 1.0,
                            "arg2": 2.0,
                            "wait": "False"
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
            item0 = MissionItem(behavior='dummy', args={'arg1': 1.0, 'arg2': 2.0, 'wait': 'False'})
            item1 = MissionItem(
                behavior='dummy', args={'arg1': 99.0, 'arg2': 98.0, 'wait': 'False'}
            )
            other_mission = Mission(target='drone_0', plan=[item0, item1])
            self.assertEqual(Mission.parse_raw(dummy_mission), other_mission)

    class TestInterpreterStatus(unittest.TestCase):
        """Interpreter Status testing."""

        # TODO: WIP test
        def _test_status(self):
            """Test dummy status."""
            status = InterpreterStatus(
                state='RUNNING',
                current_item='go_to',
                feedback_current={'actual_speed': 2.983, 'actual_distance_to_goal': 4.563},
                done_items=1,
                pending_items=1,
            )
            print(status)
            print(status.json())

    unittest.main()
