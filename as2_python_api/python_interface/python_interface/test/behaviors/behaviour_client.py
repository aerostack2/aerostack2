
import time
from enum import Enum

import rclpy

from python_interface.drone_interface import DroneInterface
from python_interface.behaviour_actions.behavior_handler import BehaviorHandler

from as2_msgs.action import TakeOff


class BehaviorStatus(Enum):
    IDLE = 0
    RUNNING = 1
    PAUSED = 2


rclpy.init()

drone_interface = DroneInterface("drone_sim_0", verbose=False)

test = BehaviorHandler(drone_interface, TakeOff, "/TakeOffBehaviour")

time.sleep(1)

print("STATUS:", BehaviorStatus(test.status).name)

print("START")
goal = TakeOff.Goal()
goal.takeoff_height = 1.0
goal.takeoff_speed = 0.5
test.start(goal, False)

time.sleep(2)
print("STATUS:", BehaviorStatus(test.status).name)
print("PAUSE")
test.pause()

time.sleep(2)
print("STATUS:", BehaviorStatus(test.status).name)
print("RESUME")
test.resume()
time.sleep(2)
print("STATUS:", BehaviorStatus(test.status).name)
print(test.result)

print("SECOND START")
test.start(goal, False)
time.sleep(2)

print("STOP")
test.stop()
time.sleep(2)
print("STATUS:", BehaviorStatus(test.status).name)
print(test.result)

print("Exit")
test.destroy()
