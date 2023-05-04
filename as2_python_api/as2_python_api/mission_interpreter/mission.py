"""
mission.py
"""

from __future__ import annotations

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

import inspect
from collections import deque

from pydantic import BaseModel

from as2_python_api.tools.utils import get_module_call_signature


class MissionItem(BaseModel):
    """Mission Item data model
    """
    behavior: str
    args: dict

    def __str__(self):
        return f"{self.behavior}: {self.args}"


class Mission(BaseModel):
    """Mission data model
    """
    target: str
    verbose: bool = False
    plan: list[MissionItem] = []

    @property
    def stack(self) -> deque:
        """
        Return mission deque stack

        :raises exc: if behavior arg doesn't exist
        :rtype: deque
        """
        mission_queue = deque()

        for mission_item in self.plan:
            signature = get_module_call_signature(mission_item.behavior)

            args = []
            for param in signature.parameters:
                try:
                    # if param found in mission, append it
                    args.append(mission_item.args[param])
                except KeyError as exc:
                    # if param not found in mission
                    if param == 'self':
                        pass
                    elif signature.parameters[param].default != inspect.Parameter.empty:
                        # append default
                        args.append(signature.parameters[param].default)
                    else:
                        raise exc

            mission_queue.append((mission_item.behavior, args))
        return mission_queue

    def __str__(self):
        mission = f"{self.target} verbose={self.verbose}\n"
        for item in self.plan:
            mission += f"\t{item}\n"
        return mission

    # TODO
    def append_to_plan(self) -> None:
        """Append mission item to plan"""
        raise NotImplementedError


def test():
    """Test Mission"""
    dummy_mission = """
    {
        "target": "drone_0",
        "verbose": "True",
        "plan": [
            {
                "behavior": "test",
                "args": {
                    "arg1": 1.0,
                    "arg2": 2.0,
                    "wait": "False"
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
    stack = mission.stack
    item = stack.popleft()
    assert item == ('test', [1.0, 2.0, 'False'])

    item = stack.popleft()
    assert item == ('test', [99.0, 98.0, 'False'])


if __name__ == "__main__":
    test()
