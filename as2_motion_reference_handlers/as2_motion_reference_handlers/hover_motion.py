#!/usr/bin/env python3

"""Implementation of a motion reference handler for speed motion."""

# Copyright 2023 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = ' Rafael Perez Seguí, Miguel Fernandez Cortizas, Pedro Arias Perez '
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'


from as2_motion_reference_handlers.basic_motion_references import BasicMotionReferenceHandler
from as2_msgs.msg import ControlMode
from rclpy.node import Node


class HoverMotion(BasicMotionReferenceHandler):
    """Send hover motion command."""

    def __init__(self, node: Node):
        """Initialize hover motion handler."""
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

    def send_hover(self):
        """Send hover command."""
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME
        return self.check_mode()
