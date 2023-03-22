"""Configuration values."""

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

__authors__ = "Javier Melero Deza, Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

from enum import Enum


class ExtendedEnum(Enum):

    @classmethod
    def list(cls):
        return list(map(lambda c: c.value, cls))


class KeyMappings(ExtendedEnum):
    TAKE_OFF_KEY = "t"
    LAND_KEY = "l"
    HOVER_KEY = "space"
    EMERGENCY_KEY = "Delete"
    UP_KEY = "w"
    DOWN_KEY = "s"
    ROTATE_RIGHT_KEY = "d"
    ROTATE_LEFT_KEY = "a"
    LEFT_KEY = "Left"
    RIGHT_KEY = "Right"
    FORWARD_KEY = "Up"
    BACKWARD_KEY = "Down"


class ControlValues(ExtendedEnum):
    SPEED_VALUE = 1.00
    VERTICAL_VALUE = 1.00
    TURN_SPEED_VALUE = 0.10
    POSITION_VALUE = 1.00
    ALTITUDE_VALUE = 1.00
    TURN_ANGLE_VALUE = 0.10


class ControlModes(ExtendedEnum):
    SPEED_CONTROL = "-SPEED-"
    POSE_CONTROL = "-POSE-"


class Options(ExtendedEnum):
    ARM_ON_TAKE_OFF = True
