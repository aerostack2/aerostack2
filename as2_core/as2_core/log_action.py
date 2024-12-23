# Copyright 2024 Universidad Politécnica de Madrid
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

"""Module for the LogWarn action."""

__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


from typing import List

import launch.logging
import logging

from launch.action import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser  # noqa: F401
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions


@expose_action('as2_log')
class Log(Action):
    """Action that warns a message when executed."""

    def __init__(self, *, msg: SomeSubstitutionsType, level: str = 'info',
                 user: str = 'user', **kwargs):
        """Create a LogInfo action."""
        super().__init__(**kwargs)

        self.__msg = normalize_to_list_of_substitutions(msg)
        self.__level = logging._nameToLevel[level.upper()]
        self.__logger = launch.logging.get_logger('launch.' + user)

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: 'Parser'
    ):
        """Parse `log` tag."""
        _, kwargs = super().parse(entity, parser)
        kwargs['msg'] = parser.parse_substitution(entity.get_attr('message'))
        return cls, kwargs

    @property
    def msg(self) -> List[Substitution]:
        """Getter for self.__msg."""
        return self.__msg

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        self.__logger.log(
            self.__level,
            ''.join([context.perform_substitution(sub) for sub in self.msg])
        )
        return None
