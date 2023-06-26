
import rclpy

from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.drone_interface import DroneInterface


def loading_mod():
    module_takeoff = 'as2_python_api.modules.takeoff_module'
    module_land = 'as2_python_api.modules.land_module'
    module_goto = 'as2_python_api.modules.go_to_module'
    module_follow_path = 'as2_python_api.modules.follow_path_module'

    drone_interface = DroneInterfaceBase("drone_sim_0", verbose=True)

    drone_interface.load_module(module_takeoff)
    drone_interface.load_module(module_land)
    drone_interface.load_module(module_goto)
    drone_interface.load_module(module_follow_path)
    print(drone_interface.modules)

    drone_interface.shutdown()


def preloaded_mod():
    drone_interface = DroneInterface("drone_sim_0", verbose=True)
    print(drone_interface.modules)

    drone_interface.shutdown()


def main():
    import cProfile
    import pstats

    with cProfile.Profile() as pr:
        # loading_mod()
        preloaded_mod()

    stats = pstats.Stats(pr)
    stats.sort_stats(pstats.SortKey.TIME)
    # stats.print_stats()
    stats.dump_stats(filename='needs_profiling.prof')


if __name__ == "__main__":
    rclpy.init()
    main()
