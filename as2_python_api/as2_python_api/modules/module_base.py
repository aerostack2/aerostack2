"""Module Base."""

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

from typing import Callable, TYPE_CHECKING

from as2_python_api.mission_interpreter.mission import MissionItem

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class ModuleBase:
    """Module Base."""

    __alias__ = ''
    __deps__ = []

    def __init__(self, drone: 'DroneInterface', alias: str) -> None:
        # ModuleBase used as mixin to call __init methods from next items at the mro
        try:
            super().__init__(drone)
        except TypeError:
            super().__init__()
        self.__drone = drone
        self.__alias__ = alias
        self.__drone.modules[self.__alias__] = self

    def __del__(self):
        try:
            # Delete when unloading module
            del self.__drone.modules[self.__alias__]
        except KeyError:
            pass  # Avoid exception when DroneInterface destruction

    @classmethod
    def get_plan_item(cls, method_name: Callable = None, *args, **kwargs) -> MissionItem:
        """
        Get a MissionItem from a method call.

        :param method_name: Method to be called. Defaults to "cls.__call__".
        :type method_name: Callable, optional
        :return: MissionItem with the method call
        :rtype: MissionItem
        """
        if method_name is None:
            method_name = cls.__call__  # Use the default __call__ method

        alias = cls.__alias__  # Access alias from the child class
        method_name_str = method_name.__name__

        # Convert positional arguments to a dictionary
        arg_dict = {key: value for key, value in zip(method_name.__code__.co_varnames[1:], args)}

        # Include keyword arguments in the dictionary
        arg_dict.update(kwargs)

        return MissionItem(behavior=alias, method=method_name_str, args=arg_dict)
