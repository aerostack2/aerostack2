"""
test_interpreter.py
"""

import rclpy

from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter


def test_dummy():
    """a doctest in a docstring

    >>> test_dummy()
    test called with height=1.0, speed=2.0 and wait=True
    test called with height=98.0, speed=99.0 and wait=True
    """

    mission = Mission.parse_file("dummy_mission.json")

    rclpy.init()
    interpreter = MissionInterpreter(mission)
    interpreter.perform_mission()


def test():
    """a doctest in a docstring
    """

    mission = Mission.parse_file("my_mission.json")

    rclpy.init()
    interpreter = MissionInterpreter(mission)
    assert sorted(interpreter.drone.modules.keys()) == [
        "go_to", "land", "takeoff"]
    rclpy.shutdown()


if __name__ == "__main__":
    test()

    # import doctest
    # doctest.testmod()
