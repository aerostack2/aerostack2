"""
mission.py
"""

from __future__ import annotations
import importlib
import inspect
from collections import deque

from pydantic import BaseModel


class MissionItem(BaseModel):
    """Mission Item data model
    """
    behavior: str
    args: dict


class Mission(BaseModel):
    """Mission data model
    """
    target: str
    verbose: bool = False
    plan: list[MissionItem] = []

    @staticmethod
    def get_module_call_signature(module_name: str) -> inspect.Signature:
        """get call method signature from given module name

        :rtype: inspect.Signature
        """
        module = importlib.import_module(
            f'as2_python_api.modules.{module_name}_module')
        target = [t for t in dir(
            module) if "Module" in t and t != 'ModuleBase']
        class_ = getattr(module, str(*target))

        signature = inspect.signature(class_.__call__)
        return signature

    @property
    def stack(self) -> deque:
        """
        Return mission deque stack

        :raises exc: if behavior arg doesn't exist
        :rtype: deque
        """
        mission_queue = deque()

        for mission_item in self.plan:
            signature = self.get_module_call_signature(mission_item.behavior)

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
