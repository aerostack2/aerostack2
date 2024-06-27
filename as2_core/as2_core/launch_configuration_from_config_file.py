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

"""
Module for LaunchConfigurationFromConfigFile action.

    Parameter priority:
    1. Values in the default config file
    2. Values given as single arguments in the launch file
    3. Values given in the user config file argument
"""

__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import tempfile
from typing import List, Text

from as2_core.launch_param_utils import _open_yaml_file
import launch
import launch.utilities
import yaml


class LaunchConfigurationFromConfigFile(launch.substitution.Substitution):
    """Override Launch Configuration from a config file with arguments."""

    def __init__(
        self,
        name: Text,
        default_file: launch.SomeSubstitutionsType,
        **kwargs
    ) -> None:
        super().__init__(**kwargs)

        self.__name = name
        self.__default_file = launch.utilities.normalize_to_list_of_substitutions(default_file)

    @property
    def name(self) -> List[launch.Substitution]:
        """Getter for name."""
        return self.__name

    @property
    def default_file(self) -> List[launch.Substitution]:
        """Getter for source_file."""
        return self.__default_file

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> Text:
        """Perform the substitution."""
        # Get default config file
        yaml_filename = launch.utilities.perform_substitutions(context, self.default_file)
        with open(yaml_filename, 'r', encoding='utf-8') as file:
            lines = file.read()
            default_data = yaml.load(lines, Loader=yaml.FullLoader)
        # Update default values with context values.
        merged_data = self.update_leaf_keys(default_data, context.launch_configurations)

        # Create temporary file with merged data
        temp_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False)

        # Get user config file from launch argument, if not given return
        user_yaml_filename = launch.substitutions.LaunchConfiguration(self.name).perform(context)
        if user_yaml_filename == yaml_filename:
            # if no user config file, just dump default one with launch arguments overwritten
            yaml.dump(merged_data, temp_yaml_file)
            return temp_yaml_file.name

        # Append user config file to the temporary file
        with open(user_yaml_filename, 'r', encoding='utf-8') as file:
            user_data = yaml.load(file.read(), Loader=yaml.FullLoader)
            # Merge for avoiding inner duplicated keys
            merged_user_data = self.merge_dicts(merged_data, user_data)
            # And then split for every dict to have their own nammespace
            for elemet in self.split_and_populate_namespace(merged_user_data):
                yaml.dump(elemet, temp_yaml_file)
            data, _ = _open_yaml_file(user_yaml_filename)
            # update context with user config file
            context.launch_configurations.update(data)

        return temp_yaml_file.name

    def update_leaf_keys(self, data: dict, new_values: dict) -> dict:
        """Update leaf keys in a dictionary."""
        for key, item in data.items():
            if isinstance(item, dict):
                self.update_leaf_keys(item, new_values)
            else:
                v = new_values.get(key, item)
                data[key] = v
                try:
                    data[key] = float(v) if '.' in v else int(v)
                except ValueError:
                    v1 = v.lower()
                    if v1 == 'true':
                        data[key] = True
                    elif v1 == 'false':
                        data[key] = False
                except TypeError:
                    pass
        return data

    def merge_dicts(self, dict1: dict, dict2: dict) -> dict:
        """Merge two dictionaries."""
        for key, value in dict2.items():
            if key in dict1:
                if isinstance(value, dict):
                    dict1[key] = self.merge_dicts(dict1[key], value)
                else:
                    dict1[key] = value
            else:
                dict1[key] = value
        return dict1

    def split_and_populate_namespace(self, data: dict) -> list[dict]:
        """
        Split and populate namespace, following ROS2 yaml files.

        ROS2 YAML files allow to define multiple root keys in a single file
        to represent multiple namespaces. This function splits the data in multiple
        dictionaries to then build a param file.
        """
        data_list = []
        for namespace in data.keys():
            for key, value in data[namespace].items():
                data_list.append({namespace: {key: value}})
        return data_list
