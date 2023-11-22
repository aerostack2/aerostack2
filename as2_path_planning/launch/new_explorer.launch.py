"""
explorer.launch.py
"""
from typing import Dict, Optional
import tempfile
import yaml
from launch_ros.actions import Node, SetParameter
from launch_ros.parameters_type import SomeParameters, SomeParameterFile
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution,\
    PathJoinSubstitution
from launch.some_substitutions_type import SomeSubstitutionsType


class Test(Substitution):
    def __init__(self,
                 params_file: Optional[SomeParameterFile] = None,
                 namespace: Optional[SomeSubstitutionsType] = None,
                 node_name: Optional[SomeSubstitutionsType] = None,
                 params: Optional[SomeParameters] = None) -> None:
        super().__init__()
        self.__params_file = params_file
        self.__namespace = namespace
        self.__node_name = node_name
        self.__params = params

    def perform(self, context) -> str:
        params_file_ = self.__params_file.perform(context)
        print(params_file_)
        # TODO: if params_file_ is None, use NamedTempFile, otherwise use params_file_

        namespace = self.__namespace.perform(context)
        namespace = namespace if namespace is not None else '/**'

        node_name = self.__node_name.perform(context)
        node_name = node_name if node_name is not None else '/**'

        params = self.resolve_replacements(context, self.__params)
        with tempfile.NamedTemporaryFile('w', delete=False) as params_file:
            yaml.safe_dump(
                {namespace: {node_name: {'ros__parameters': params}}}, params_file)
            return params_file.name

    def resolve_replacements(self, context, replacements: Dict) -> Dict:
        '''Perform substitution on dict values if needed'''
        resolved_replacements = {}
        for key, value in replacements.items():
            if value == '':
                # Not overriding this parameter
                continue
            if isinstance(value, Substitution):
                resolved_replacements[key] = self.convert(
                    value.perform(context))
            else:
                resolved_replacements[key] = value
        return resolved_replacements

    def convert(self, text_value):
        '''
        Try to convert from text to float, int or bool. 
        If none of those work, return text
        '''
        # try converting to int or float
        try:
            return float(text_value) if '.' in text_value else int(text_value)
        except ValueError:
            pass

        # try converting to bool
        if text_value.lower() == "true":
            return True
        if text_value.lower() == "false":
            return False

        # nothing else worked so fall through and return text
        return text_value


def generate_launch_description():
    """entrypoint
    """
    config_file = PathJoinSubstitution([
        FindPackageShare('as2_path_planning'),
        'config', 'explorer.yaml'
    ])

    namespace = LaunchConfiguration('namespace')
    node_name = TextSubstitution(text='explorer')
    params = {
        'frontier_min_area': LaunchConfiguration('frontier_min_area'),
        'safety_distance': LaunchConfiguration('safety_distance'),
        'reached_dist_thresh': LaunchConfiguration('reached_dist_thresh'),
        'navigation_speed': LaunchConfiguration('navigation_speed'),
        'test': 'test'
    }
    params_file = LaunchConfiguration('params_file')

    configured_params = Test(params_file, namespace, node_name, params)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', description="Drone namespace",
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument(
            'use_sim_time', description="Use sim time flag", default_value='false'),
        # TODO: use default config file
        DeclareLaunchArgument(
            'params_file', description="Path to params file", default_value=config_file),
        DeclareLaunchArgument(
            'frontier_min_area', description="Minimum area size to be a frontier (in pixels)",
            default_value=''),
        DeclareLaunchArgument(
            'safety_distance', description="Safety distance to obstacles (drone size)",
            default_value=''),
        DeclareLaunchArgument(
            'reached_dist_thresh', description="Threshold to consider point as reached",
            default_value=''),
        DeclareLaunchArgument(
            'navigation_speed', description="Cruise speed of the drone during navigation",
            default_value=''),
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        Node(
            package="as2_path_planning",
            executable="explorer",
            name=node_name,
            namespace=LaunchConfiguration('namespace'),
            parameters=[configured_params],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])
