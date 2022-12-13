"""
Implementation of a motion reference handler for speed motion.
"""

from rclpy.node import Node
from as2_msgs.msg import ControlMode
from as2_motion_reference_handlers.basic_motion_references import BasicMotionReferenceHandler


class HoverMotion(BasicMotionReferenceHandler):
    """ Send hover motion command """

    def __init__(self, node: Node):
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

    def send_hover(self):
        """ Send hover command """
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME
        return self.check_mode()
