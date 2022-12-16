"""Platform info data wrapper"""

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


__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import threading
from typing import Callable, List, Union
from dataclasses import dataclass, field

from as2_msgs.msg import PlatformStatus
from as2_msgs.msg import ControlMode


lock = threading.Lock()


def lock_decor(func: Callable) -> Callable:
    """locker decorator"""

    def wrapper(self, *args, **kwargs) -> Callable:
        with lock:
            return func(self, *args, **kwargs)
    return wrapper


@dataclass
class PlatformInfoData:
    """Platform info"""
    __connected: bool = field(default_factory=lambda: False)
    __armed: bool = field(default_factory=lambda: False)
    __offboard: bool = field(default_factory=lambda: False)
    __state: int = field(default_factory=lambda: PlatformStatus.DISARMED)
    __yaw_mode: int = field(default_factory=lambda: ControlMode.NONE)
    __control_mode: int = field(default_factory=lambda: ControlMode.UNSET)
    __reference_frame: int = field(
        default_factory=lambda: ControlMode.UNDEFINED_FRAME)

    def __repr__(self) -> str:
        info = self.data
        return f"[{info[0]}, {info[1]}, {info[2]}, \
                 {info[3]}, {info[4]}, {info[5]}, {info[6]}]"

    @property
    @lock_decor
    def data(self) -> List[Union[bool, int]]:
        """locked getter"""
        return [self.__connected, self.__armed, self.__offboard, self.__state,
                self.__yaw_mode, self.__control_mode, self.__reference_frame]

    @data.setter
    @lock_decor
    def data(self, dat: List[Union[bool, int]]) -> None:
        """locked setter"""
        self.__connected = bool(dat[0])
        self.__armed = bool(dat[1])
        self.__offboard = bool(dat[2])
        self.__state = dat[3]
        self.__yaw_mode = dat[4]
        self.__control_mode = dat[5]
        self.__reference_frame = dat[6]
