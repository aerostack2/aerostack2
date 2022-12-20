from as2_python_api.drone_interface import DroneInterface
import rclpy
import time


list_uav = list()
rclpy.init()

uav1 = DroneInterface('drone_sim_0', verbose=True, use_sim_time=False)
uav2 = DroneInterface('drone_sim_1', verbose=True, use_sim_time=False)

uav1.disarm()

while (rclpy.ok()):
    time.sleep(0.1)
    print(uav1.drone_id, uav1.info)
    print(uav2.drone_id, uav2.info)

rclpy.shutdown()
