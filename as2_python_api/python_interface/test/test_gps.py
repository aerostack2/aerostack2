from time import sleep

import rclpy

from python_interface.drone_interface import DroneInterface

rclpy.init()

drone_interface = DroneInterface("drone_sim_0", verbose=True)

drone_interface.offboard()
drone_interface.arm()

drone_interface.takeoff(3, 2)
print("Takeoff completed\n")
sleep(1)

drone_interface.go_to(x=0, y=5, z=3, speed=1.0, ignore_yaw=True)
# drone_interface.go_to(point=[5, 0, 3], speed=1.0, ignore_yaw=True)
drone_interface.go_to_gps(lat=28.1439, lon=-16.503, alt=46, speed=1.0, ignore_yaw=True)
# drone_interface.go_to_gps(waypoint=[28.1439, -16.503, 46], speed=1.0, ignore_yaw=True)

print("Path finished")

drone_interface.land(0.2)
drone_interface.shutdown()

print("Bye!")