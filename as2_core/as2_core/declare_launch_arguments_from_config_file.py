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

"""Module for DeclareLaunchArgumentsFromConfigFile action."""

__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import List, Optional, Text

from as2_core.launch_param_utils import _get_declare_launch_argument
import launch
import launch.utilities


class DeclareLaunchArgumentsFromConfigFile(launch.actions.GroupAction):
    """
    Action to declare Launch Arguments from elements in a config file.

    Example:
    -------
    config_file:
        /**:
            ros__parameters:
                param_0: "value_0"  # Description for param_0
                param_int: 1  # Description for param_int
                param_bool: true  # Description for param_bool

    DeclareLaunchArgumentsFromConfigFile(
        name='config_file', source_file='/tmp/config.yaml',
        description='Configuration file description')

    Result:
    ------
    1. ros2 launch package_name launch_file.py -s

        Arguments (pass arguments as '<name>:=<value>'):

            'config_file':
                Configuration file description
                (default: '/tmp/config.yaml')

            'param_0':
                Description for param_0
                (default: 'value_0')

            'param_int':
                Description for param_int
                (default: '1')

            'param_bool':
                Description for param_bool
                (default: 'true')

    """

    def __init__(
        self,
        name: Text,
        source_file: launch.SomeSubstitutionsType,
        *,
        description: Optional[Text] = None,
        **kwargs
    ) -> None:

        self.__name = name
        self.__source_file = launch.utilities.normalize_to_list_of_substitutions(source_file)
        self.__description = description

        actions = [
            launch.actions.DeclareLaunchArgument(
                self.name, default_value=self.source_file, description=self.description),
            # *_get_declare_launch_argument(source_file)
        ]
        if isinstance(source_file, Text):
            actions.extend(_get_declare_launch_argument(source_file))

        super().__init__(actions=actions, scoped=False, **kwargs)

    @property
    def name(self) -> Text:
        """Getter for self.__name."""
        return self.__name

    @property
    def source_file(self) -> List[launch.substitution.Substitution]:
        """Getter for self.__default_value."""
        return self.__source_file

    @property
    def description(self) -> Text:
        """Getter for self.__description."""
        return self.__description

    def execute(self, context: launch.LaunchContext):
        source_file = launch.utilities.perform_substitutions(context, self.source_file)

        self._GroupAction__actions = [
            launch.actions.DeclareLaunchArgument(
                self.name, default_value=source_file, description=self.description),
            *_get_declare_launch_argument(source_file)
        ]
        return super().execute(context)
