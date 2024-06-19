"""Mission stack of behaviors to be executed by the drone."""

from __future__ import annotations

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

from collections import deque
from typing import Deque
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from as2_python_api.mission_interpreter.mission import MissionItem

# TODO: improve mission_stack
# Class MissionStack:
#       attributtes: current, done_deque, todo_deque
#       methods: append, insert, repeat_last


class MissionStack:
    """Mission stack."""

    def __init__(self, mission_stack: list = None) -> None:
        mission_stack = [] if mission_stack is None else mission_stack

        self.__pending: Deque[MissionItem] = deque(mission_stack)  # FIFO
        self.__done: Deque[MissionItem] = deque()  # LIFO
        self.__current: MissionItem = None

    def __str__(self) -> 'MissionItem':
        current = 'None' if self.current is None else self.current
        return current.json() + str(self.pending)

    def next_item(self) -> 'MissionItem':
        if self.__current is not None:
            self.__done.append(self.__current)

        if len(self.pending) > 0:
            self.__current = self.__pending.popleft()
        else:
            self.__current = None
        return self.__current

    def previous_item(self):
        raise NotImplementedError

    def add(self, item):
        self.__pending.append(item)

    @property
    def last_done(self):
        return self.__done[0]

    @property
    def pending(self) -> list:
        return list(self.__pending)

    @property
    def done(self) -> list:
        return list(self.__done)

    @property
    def current(self) -> 'MissionItem':
        # TEMP: use MissionItem instead tuple
        if self.__current is None:
            return None
        return self.__current
