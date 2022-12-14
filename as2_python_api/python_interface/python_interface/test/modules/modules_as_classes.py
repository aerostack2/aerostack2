import rclpy

from python_interface.drone_interface import DroneInterface
from python_interface.modules.takeoff_module import TakeoffModule as Takeoff

rclpy.init()


drone_interface = DroneInterface("drone_sim_0", verbose=True)

print(drone_interface.modules)

tk = Takeoff(drone_interface)
tk()
del tk

print(drone_interface.modules)

# drone_interface.shutdown()

# print("Bye!")
