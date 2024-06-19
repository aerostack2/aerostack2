"""
Dummy Module for testing purpouses.

This module is a dummy module that does nothing but print the arguments it receives.
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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import time
import typing

from as2_python_api.modules.module_base import ModuleBase

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class DummyModule(ModuleBase):
    """Dummy Module."""

    __alias__ = 'dummy'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.stopped = False

    def __call__(self, arg1: float, arg2: int, wait: bool = True) -> None:
        """Do dummy call."""
        if isinstance(wait, str):
            wait = wait.lower() == 'true'
        self.stopped = not wait
        print(f'{self.__alias__} called with {arg1=}, {arg2=} and {wait=}')
        while not self.stopped:
            print(f'{self.__alias__} called with {arg1=}, {arg2=} and {wait=}')
            time.sleep(0.5)

    def stop(self):
        """Stop dummy module."""
        self.stopped = True

    def destroy(self):
        """
        Do nothing.

        Dummy Module does not inherit from a behavior with a destroy method, so self defining it
        """
