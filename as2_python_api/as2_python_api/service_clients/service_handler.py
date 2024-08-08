"""Service handler."""

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


__authors__ = 'Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import typing

from rclpy.client import Client
from std_srvs.srv import SetBool

if typing.TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class ServiceHandler:
    """Service handler class."""

    TIMEOUT = 3  # seconds

    def __init__(self, service_client: Client, logger) -> None:
        self._service_client = service_client

        # Wait for Action availability
        if not service_client.wait_for_service(timeout_sec=self.TIMEOUT):
            logger.error(f'{service_client.srv_name} not available')

    def __call__(self, request_msg):
        """Call the service."""
        return self._service_client.call(request_msg)


class ServiceBoolHandler(ServiceHandler):
    """Service SetBool handler class."""

    TIMEOUT = 3  # seconds

    def __init__(self, drone: 'DroneInterfaceBase', service_name: str) -> None:
        self._logger = drone.get_logger()
        try:
            self._service_client = drone.create_client(
                SetBool, service_name)
        except Exception as ex:
            self._logger.error(f'Coud not create client for {service_name}')
            raise ex

        return super().__init__(self._service_client, self._logger)

    def __call__(self, value: bool = True) -> bool:
        """Call the service."""
        request = SetBool.Request()
        request.data = value
        response = super().__call__(request)
        if not response.success:
            self._logger.error('Service returned failure')
        return response.success
