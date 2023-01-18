"""
fake_origin_server.py
"""

# Copyright 2022 Universidad Politécnica de Madrid
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
#    * Neither the name of the the copyright holder nor the names of its
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

__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import rclpy
from rclpy.node import Node

from as2_msgs.srv import SetOrigin, GetOrigin


class FakeOriginServer(Node):
    """Fake Origin Server Node
    """

    def __init__(self, namespace=""):
        super().__init__('fake_origin', namespace=namespace)

        self.__origin = None

        self._set_origin_srv = self.create_service(
            SetOrigin, 'set_origin', self.set_origin_cbk)

        self._get_origin_srv = self.create_service(
            GetOrigin, 'get_origin', self.get_origin_cbk)

        self.get_logger().info("Fake Origin Server started")

    def set_origin_cbk(self, request: SetOrigin.Request, response: SetOrigin.Response):
        """set origin callbak, saves the origin
        """
        self.get_logger().info(f"SET origin {request.origin}")
        if self.__origin:
            response.success = False
            return response
        self.__origin = request.origin
        response.success = True
        return response

    def get_origin_cbk(self, request: GetOrigin.Request, response: GetOrigin.Response):
        """get origin callback, returns the origin if set
        """
        self.get_logger().info(f"GET origin {self.__origin}")
        if not self.__origin:
            response.success = False
            return response
        response.origin = self.__origin
        response.success = True
        return response


def main(args=None):
    """main entrypoint
    """
    rclpy.init(args=args)

    fake_origin_server = FakeOriginServer("drone_sim_0")

    rclpy.spin(fake_origin_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
