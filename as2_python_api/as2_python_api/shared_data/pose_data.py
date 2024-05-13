"""Pose data wrapper (position + orientation)."""

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

from dataclasses import dataclass, field

from ..shared_data.orientation_data import OrientationData
from ..shared_data.position_data import PositionData


@dataclass
class PoseData:
    """Pose data."""

    __pose: PositionData = field(default_factory=lambda: PositionData())
    __orientation: OrientationData = field(
        default_factory=lambda: OrientationData())

    def __repr__(self) -> str:
        pose = self.position
        orient = self.orientation
        return f'[{pose[0]}, {pose[1]}, {pose[2]}], [{orient[0]}, {orient[1]}, {orient[2]}]'

    @property
    def position(self) -> list[float]:
        """Position getter."""
        return self.__pose.position

    @position.setter
    def position(self, pos: list[float]) -> None:
        """Position setter."""
        self.__pose.position = pos

    @property
    def orientation(self) -> list[float]:
        """Orientation getter."""
        return self.__orientation.orientation

    @orientation.setter
    def orientation(self, orient: list[float]) -> None:
        """Orientation setter."""
        self.__orientation.orientation = orient
