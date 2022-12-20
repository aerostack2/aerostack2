from time import sleep

import rclpy

from as2_python_api.drone_interface import DroneInterface

rclpy.init()

drone_interface = DroneInterface("drone_sim_0", verbose=True)

drone_interface.offboard()
drone_interface.arm()

drone_interface.follow_path([[0, 0, 3]], 1)
print("Takeoff completed\n")
sleep(1)


drone_interface.follow_path([[5, 0, 3],
                            [5, 5, 3],
                            [0, 5, 3],
                            [0, 0, 3]], 5)

print("Path finished")

drone_interface.land(0.2)
drone_interface.shutdown()

print("Bye!")
