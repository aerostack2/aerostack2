from python_interface.behaviour_actions.land_behaviour import SendLand


class LandModule:
    __alias__ = "land"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

    def __call__(self, speed: float = 0.5) -> None:
        """Land with given speed (m/s).

        :type speed: float
        """
        SendLand(self.__drone, float(speed))

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def modify(self, speed):
        raise NotImplementedError
