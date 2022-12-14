"""
motion_reference_handler_module.py
"""
import typing

from std_srvs.srv import SetBool

from as2_motion_reference_handlers.hover_motion import HoverMotion
from as2_motion_reference_handlers.position_motion import PositionMotion
from as2_motion_reference_handlers.speed_motion import SpeedMotion
from as2_motion_reference_handlers.speed_in_a_plane import SpeedInAPlaneMotion

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class MotionReferenceHandlerModule:
    """Motion Reference Handlers module"""
    __alias__ = "motion_ref_handler"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        # FIXME: temporaly, manually stoping behaviors
        self.trajectory_gen_cli = self.__drone.create_client(
            SetBool, "traj_gen/run_node")
        if not self.trajectory_gen_cli.wait_for_service(timeout_sec=3):
            self.__drone.get_logger().warn("Trajectory generator service not found")
            self.trajectory_gen_cli = None

        self.__hover_motion_handler = HoverMotion(self.__drone)
        self.position = PositionMotion(self.__drone)
        self.speed = SpeedMotion(self.__drone)
        self.speed_in_a_plane = SpeedInAPlaneMotion(self.__drone)

    def hover(self) -> None:
        """Stop and hover current position.
        """
        if self.trajectory_gen_cli is not None:
            self.__drone.get_logger().info("Calling trajectory generator")
            req = SetBool.Request()
            req.data = False
            resp = self.trajectory_gen_cli.call(req)
            if not resp.success:
                self.__drone.get_logger().warn("Cannot stop trajectory generator")
        self.__hover_motion_handler.send_hover()
        self.__drone.get_logger().info("Hover sent")

    def destroy(self) -> None:
        """Destroy module, clean exit
        """
        self.__drone.destroy_client(self.trajectory_gen_cli)

        self.__hover_motion_handler = None
        self.position = None
        self.speed = None
        self.speed_in_a_plane = None
