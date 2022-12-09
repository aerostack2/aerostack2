from python_interface.behaviour_actions.takeoff_behaviour import SendTakeoff


class TakeoffModule:
    __alias__ = "takeoff"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

    def __call__(self, height: float = 1.0, speed: float = 0.5) -> None:
        """Takeoff to given height (m) and given speed (m/s).

        :type height: float
        :type speed: float
        """
        SendTakeoff(self.__drone, float(height), float(speed))

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def modify(self, height, speed):
        raise NotImplementedError
