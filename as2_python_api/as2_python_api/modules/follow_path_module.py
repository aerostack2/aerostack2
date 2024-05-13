"""Follow Path module."""

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


__authors__ = 'Pedro Arias Pérez, Miguel Fernández Cortizas, David Pérez Saura, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import typing

from as2_msgs.msg import YawMode
from as2_python_api.behavior_actions.followpath_behavior import FollowPathBehavior
from as2_python_api.modules.module_base import ModuleBase
from nav_msgs.msg import Path

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class FollowPathModule(ModuleBase, FollowPathBehavior):
    """Follow Path Module."""

    __alias__ = 'follow_path'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, path: Path, speed: float,
                 yaw_mode: int = YawMode.KEEP_YAW,
                 yaw_angle: float = None, frame_id: str = 'earth', wait: bool = True) -> bool:
        """
        Follow path.

        :param path: path to follow
        :type path: Path
        :param speed: speed (m/s) limit
        :type speed: float
        :param yaw_mode: yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :param yaw_angle: yaw angle (rad) when fixed yaw is set, defaults to None
        :type yaw_angle: float, optional
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__follow_path(path, speed, yaw_mode, yaw_angle, frame_id, wait)

    def __follow_path(self, path: Path,
                      speed: float, yaw_mode: int, yaw_angle: float, frame_id: str = 'earth',
                      wait: bool = True) -> bool:
        """
        Follow path.

        :param path: path to follow
        :type path: Path
        :param speed: speed (m/s) limit
        :type speed: float
        :param yaw_mode: yaw mode
        :type yaw_mode: int
        :param yaw_angle: yaw angle (rad) when fixed yaw is set
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.start(path=path, speed=speed, yaw_mode=yaw_mode, yaw_angle=yaw_angle,
                          frame_id=frame_id, wait_result=wait)

    # Method simplifications
    def follow_path_with_keep_yaw(self, path: Path, speed: float, frame_id: str = 'earth') -> bool:
        """
        Follow path. With keep yaw mode. Blocking call.

        :param path: path to follow
        :type path: Path
        :param speed: speed (m/s) limit
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__follow_path(path, speed,
                                  yaw_mode=YawMode.KEEP_YAW, yaw_angle=0.0, frame_id=frame_id)

    def follow_path_with_yaw(self, path: Path, speed: float, angle: float,
                             frame_id: str = 'earth') -> bool:
        """
        Follow path. With desired yaw angle. Blocking call.

        :param path: path to follow
        :type path: Path
        :param speed: speed (m/s) limit
        :type speed: float
        :param yaw_angle: yaw angle (rad) when fixed yaw is set
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__follow_path(path, speed,
                                  yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle, frame_id=frame_id)

    def follow_path_with_path_facing(self, path: Path, speed: float,
                                     frame_id: str = 'earth') -> bool:
        """
        Follow path. With path facing yaw mode. Blocking call.

        :param path: path to follow
        :type path: Path
        :param speed: speed (m/s) limit
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__follow_path(path, speed,
                                  yaw_mode=YawMode.PATH_FACING, yaw_angle=0.0, frame_id=frame_id)
