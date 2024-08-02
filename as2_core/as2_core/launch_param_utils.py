#!/usr/bin/env python3

# Copyright 2023 Universidad Politécnica de Madrid
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

"""Launch as2_multirotor_simulator node."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

import re

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
import yaml


def try_to_convert_to_number(value: str) -> any:
    """
    Try to convert a string to a number.

    :param value: String to be converted to a number.
    :type value: str
    :return: Number if the conversion is possible, the string otherwise.
    :rtype: any
    """
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            if value.startswith('[') and value.endswith(']'):
                return [try_to_convert_to_number(val) for val in value[1:-1].split(',')]
            if value.startswith('{') and value.endswith('}'):
                return {try_to_convert_to_number(val.split(':')[0].strip()):
                        try_to_convert_to_number(val.split(':')[1].strip())
                        for val in value[1:-1].split(',')}
            if value.lower() == 'true':
                return True
            if value.lower() == 'false':
                return False
            if value.startswith('"') and value.endswith('"'):
                return value[1:-1]
            return value


def _regenerate_yaml_file(data_dict: dict) -> dict:
    final_dict = {}
    for key, value in data_dict.items():
        dict_ref = final_dict
        for keys in key.split(':'):
            if keys not in dict_ref:
                if keys == key.split(':')[-1]:
                    dict_ref[keys] = try_to_convert_to_number(value.strip())
                else:
                    dict_ref[keys] = {}
            dict_ref = dict_ref[keys]

    return final_dict


def _merge_yaml_keys(in_lines: str) -> dict:
    clean_lines = in_lines.split('\n')
    clean_lines = [line.split('#')[0] for line in clean_lines]
    clean_lines = [line for line in clean_lines if line.strip()]
    data_dict = {}
    preamble = ''
    n_spaces_sep = None
    list_keys = []

    for line in clean_lines:
        if line.find(':') == -1:
            if line.lstrip().startswith('-'):
                content = line.split('-')[1].strip()
                list_keys.append(content)
            continue

        if list_keys:
            result = ','.join(list_keys)
            result = f'[{result}]'
            data_dict[preamble[:-1]] = result
            list_keys.clear()
            preamble = ':'.join(preamble.split(':')[:-2]) + ':'

        index = line.find(':')
        content = ''
        key = line[:index]
        if len(key) + 1 == len(line):
            pass
        else:
            content = line[index + 1:].strip()

        preamble_len = preamble.count(':')
        n_spaces = len(line) - len(line.lstrip())

        if n_spaces == 0:
            preamble = key.strip() + ':'  # main key
            continue
        if n_spaces_sep is None:
            n_spaces_sep = n_spaces

        n_tabs = n_spaces // n_spaces_sep
        if n_tabs == preamble_len:
            if content:
                data_dict[preamble + key.strip()] = content
            else:
                preamble = preamble + key.strip() + ':'
        elif n_tabs > preamble_len:
            raise ValueError('Invalid YAML file: wrong indentation')
        else:
            preamble = ':'.join(preamble.split(':')[:n_tabs]) + ':' + key.strip() + ':'
            if content:
                data_dict[preamble[:-1]] = content

    if list_keys:
        result = ','.join(list_keys)
        result = f'[{result}]'
        data_dict[preamble[:-1]] = result
        list_keys.clear()
        preamble = ':'.join(preamble.split(':')[:-2]) + ':'

    out_lines = ''
    for key, value in data_dict.items():
        out_lines += f'{key}: {value}' + '\n'

    return _regenerate_yaml_file(data_dict)


def read_complete_yaml_text(yaml_text: str) -> dict:
    """Read ROS2 YAML file and return its content as a dictionary."""
    lines = yaml_text
    data = _merge_yaml_keys(lines)

    return data, lines


def _open_yaml_file(file_path: str) -> tuple:
    """
    Open a YAML file and return its content as a dictionary.

    :param file_path: Path of the YAML file.
    :type file_path: str
    :return: Dictionary containing the content of the YAML file and the
    content of the YAML file in string format.
    :rtype: tuple(dict, str)
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        lines = file.read()
        data = yaml.safe_load(lines)

    # Check if data dict has key '/**'
    if '/**' in data:
        data = data['/**']
    if 'ros__parameters' in data:
        data = data['ros__parameters']
    return data, lines


def _flat_dictionary(data: dict, prefix: str = '') -> dict:
    """
    Recursive function to parse dictionary and construct a flat dictionary.

    :param data: Dictionary to be flattened.
    :type data: dict
    :param prefix: Prefix for key names, defaults to ''
    :type prefix: str, optional
    :return: Flat dictionary.
    :rtype: dict
    """
    if isinstance(data, dict):
        result = {}
        for key, value in data.items():
            full_key = f'{prefix}{key}'
            result.update(_flat_dictionary(value, f'{full_key}.'))
        return result
    else:
        # Returns a dictionary with the key and value.
        return {prefix.rstrip('.'): data}


def _get_parameters_description_from_yaml(yaml_data: dict, lines: str) -> dict:
    """
    Extract descriptions from a YAML file according to the specified format in the lines.

    Format:
    key: value # description

    :param yaml_data: Dictionary containing the content of the YAML file.
    :type yaml_data: dict
    :param lines: Content of the YAML file in string format.
    :type lines: str
    :return: Dictionary with keys param name and values dictionary with value and description.
    :rtype: dict
    """
    parameters = _flat_dictionary(yaml_data)
    descriptions = {}
    base_names = []  # Stores keys without assigned values encountered so far.
    base_names_ext = []  # Stores keys combinations of base_names
    for line in lines.strip().split('\n'):
        # Check for matching format "key: value".
        match_base = re.match(r'^\s*(\w+):\s*(.*?)\s*(?:#\s*(.*))?$', line)

        # Check for matching format "key: value # description".
        match_description = re.match(r'^\s*(\w+):\s*(.*?)\s*#\s*(.*)$', line)

        if match_base:
            name = match_base.group(1)
            value = match_base.group(2)
            # If there's no description, assign a default message.
            description = 'No description provided.' if not match_description \
                else match_description.group(3)
            # Ignore keys without assigned values.
            if value == '' and name != 'ros__parameters':
                base_names.append(name)
                if len(base_names_ext) == 0:
                    base_names_ext.append(name)
                else:
                    for base_name in base_names:
                        base_names_ext.append(f'{base_name}.{name}')
                continue

            # If the key is found in the YAML data, add it to the descriptions dictionary.
            if name in parameters:
                descriptions[name] = {
                    'value': parameters[name],
                    'description': description
                }
            else:
                # If the key is not found directly, look for it in combination with keys without
                # values.
                # We asume that the keys iteration is in the same order as the file.
                for base_name in reversed(base_names):
                    full_name = f'{base_name}.{name}'
                    if full_name in parameters:
                        descriptions[full_name] = {
                            'value': parameters[full_name],
                            'description': description
                        }
                        break
                # If the key is not found in the previous iteration, look for it in combination
                # with keys without values.
                for base_name in reversed(base_names_ext):
                    full_name = f'{base_name}.{name}'
                    if full_name in parameters:
                        descriptions[full_name] = {
                            'value': parameters[full_name],
                            'description': description
                        }
                        break
    return descriptions


def _parse_yaml_file(file_path: str) -> dict:
    """
    Parse a YAML file and return and dictionary with the values and descriptions of the parameters.

    :param file_path: Path of the YAML file.
    :type file_path: str
    :return: Dictionary with keys param name and values dictionary with value and description.
    :rtype: dict
    """
    data, lines = _open_yaml_file(file_path)
    return _get_parameters_description_from_yaml(_flat_dictionary(data), lines)


def _dict_to_declare_launch_argument(data_dict: dict) -> list:
    """
    Convert a dictionary to a list of DeclareLaunchArgument.

    :param data_dict: Dictionary with keys param name and values dictionary with
    value and description.
    :type data_dict: dict
    :return: List of DeclareLaunchArgument with the parameters.
    :rtype: list
    """
    declare_launch_argument = []
    for key, value in data_dict.items():
        if isinstance(value, dict) and 'value' in value:
            description = 'No description provided.' if 'description' not in value \
                else value['description']
            declare_launch_argument.append(DeclareLaunchArgument(
                key, default_value=str(value['value']), description=description))
        else:
            declare_launch_argument.append(DeclareLaunchArgument(
                key, default_value=str(value), description='No description provided.'))
    return declare_launch_argument


def _get_declare_launch_argument(file_path: str) -> list:
    """
    Get declare launch argument for params from default configuration file.

    :param file_path: YAML file path with ROS 2 parameters.
    :type file_path: str
    :return: List of DeclareLaunchArgument with the parameters.
    :rtype: list
    """
    # For default config file:
    data_dict = _parse_yaml_file(file_path)
    return _dict_to_declare_launch_argument(data_dict)


def _dict_to_launch_configuration(data_dict: dict) -> dict:
    """
    Convert a dictionary to a dictionary of LaunchConfiguration.

    :param data_dict: Dictionary with keys param name and values dictionary with
    value and description.
    :type data_dict: dict
    :return: Dictionary of LaunchConfiguration with the parameters.
    :rtype: dict
    """
    launch_configurations = {}
    for key, _ in data_dict.items():
        launch_configurations[key] = LaunchConfiguration(key)
    return launch_configurations


def _get_launch_configuration(file_path: str) -> dict:
    """
    Get launch configuration for params from default configuration file.

    :param file_path: YAML file path with ROS 2 parameters.
    :type file_path: str
    :return: Dict with the parameters.
    :rtype: dict
    """
    data_dict = _parse_yaml_file(file_path)
    return _dict_to_launch_configuration(data_dict)


def _declare_launch_arguments_opaque_function(context: LaunchContext, *args, **kwargs) -> list:
    """
    Declare launch arguments opaque function.

    :param context: Launch context.
    :type context: LaunchContext
    :return: List of DeclareLaunchArgument with the parameters.
    :rtype: list
    """
    param_name = kwargs['name']
    param_value = LaunchConfiguration(param_name).perform(context)
    return _get_declare_launch_argument(param_value)


def declare_launch_arguments(name: str, default_value: str,
                             description: str = 'No description provided') -> list:
    """
    Return DeclareLaunchArgument for a parameter of type config file path.

    Declaring all parameters inside the file.

    :param name: Name of the parameter that will store the path to the config file.
    :type name: str
    :param default_value: Default path to the config file.
    :type default_value: str
    :param description: Parameter description, defaults to 'No description provided'
    :type description: str, optional
    :return: List of DeclareLaunchArgument
    :rtype: list

    Example:
    -------
    return LaunchDescription([
        DeclareLaunchArgument(...),
        *declare_launch_arguments(
            name,
            default_value=default_value,
            description=description),
        Node(...)
        ])

    """
    return [
        DeclareLaunchArgument(
            name, default_value=default_value, description=description),
        OpaqueFunction(function=_declare_launch_arguments_opaque_function,
                       kwargs={'name': name}),
        *_get_declare_launch_argument(default_value)
    ]


def launch_configuration(variable_name: str, default_value: str) -> list:
    """
    Return LaunchConfiguration for a parameter of type config file path.

    Declaring all parameters inside the file.

    :param variable_name: Name of the parameter that will store the path to the config file.
    :type variable_name: str
    :param default_value: Default path to the config file.
    :type default_value: str
    :return: List of LaunchConfiguration
    :rtype: list

    Example:
    -------
        Node(
            ...
            parameters=[
                *launch_configuration(variable_name, default_value=default_value),
                ...
            ]
            ...
        )

    """
    return [
        LaunchConfiguration(variable_name),
        _get_launch_configuration(default_value)
    ]
