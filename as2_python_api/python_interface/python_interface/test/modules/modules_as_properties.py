import rclpy

from python_interface.drone_interface_base import DroneInterfaceBase as DroneInterface

module_takeoff = 'python_interface.modules.takeoff_module'
module_land = 'python_interface.modules.land_module'
module_gps = 'python_interface.modules.gps_module'

rclpy.init()


# load_module(DroneInterface, 'takeoff', module_takeoff)
drone_interface = DroneInterface("drone_sim_0", verbose=True)

print(drone_interface.modules)

drone_interface.load_module(module_takeoff)
drone_interface.load_module(module_land)
# drone_interface.load_module(module_gps)

drone_interface.arm()
drone_interface.offboard()
drone_interface.takeoff()

drone_interface.land()

print(drone_interface.modules)

drone_interface.shutdown()

print(drone_interface.modules)

print("Bye")
