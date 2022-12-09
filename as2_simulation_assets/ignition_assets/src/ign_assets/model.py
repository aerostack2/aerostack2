# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import codecs
import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import ign_assets.bridges

import yaml
import json


UAVS = [
    'hexrotor',
    'quadrotor'
]


# TODO: semantic_camera segmentation
def camera_models():
    models = ['vga_camera',
              'hd_camera',
              'semantic_camera']
    return models


def rgbd_models():
    models = ['rgbd_camera']
    return models


# FIXME: point_lidar scan not working properly
def lidar_models():
    models = ['planar_lidar',
              'lidar_3d',
              'point_lidar']
    return models


def gps_models():
    models = ['gps']
    return models


def suction_gripper_models():
    models = ['suction_gripper']
    return models


class Model:

    def __init__(self, model_name, model_type, n=0, position=[0, 0, 0, 0, 0, 0]):
        self.model_name = model_name
        self.model_type = model_type
        self.n = n
        self.position = position
        self.flight_time = 0
        self.battery_capacity = 0
        self.payload = {}

    def __repr__(self) -> str:
        return f"{self.model_name}[{self.model_type}]"

    def bridges(self, world_name):
        bridges = [
            # IMU
            ign_assets.bridges.imu(world_name, self.model_name, 'imu', 'internal'),
            # Magnetometer
            ign_assets.bridges.magnetometer(world_name, self.model_name, 'magnetometer', 'internal'),
            # Air Pressure
            ign_assets.bridges.air_pressure(world_name, self.model_name, 'air_pressure', 'internal'),
            # odom: not used, use ground_truth instead
            # ign_assets.bridges.odom(self.model_name),
            # pose
            ign_assets.bridges.pose(self.model_name),
            # pose static
            ign_assets.bridges.pose_static(self.model_name),
            # twist
            ign_assets.bridges.cmd_vel(self.model_name),
            # arm
            ign_assets.bridges.arm(self.model_name)
        ]
        if self.battery_capacity != 0:
            bridges.append(ign_assets.bridges.battery(self.model_name))
        nodes = [
            # Odom --> ground_truth
            Node(
                package='ignition_assets',
                executable='ground_truth_bridge',
                namespace=self.model_name,
                output='screen',
                parameters=[
                    {'name_space': self.model_name,
                     'pose_frame_id': 'earth',
                     'twist_frame_id': self.model_name + '/base_link'},
                ]
            ),
            # Node(
            #     package='ignition_assets',
            #     executable='tf_broadcaster',
            #     namespace=self.model_name,
            #     output='screen',
            #     parameters=[
            #         {
            #             'world_frame': world_name,
            #             'name_space': self.model_name
            #         }
            #     ]
            # )
        ]

        bridges_, nodes_ = self.payload_bridges(world_name)

        bridges.extend(bridges_)
        nodes.extend(nodes_)

        return bridges, nodes

    def payload_bridges(self, world_name, payloads=None):
        if not payloads:
            payloads = self.payload

        bridges = []
        nodes = []
        for k in payloads.keys():
            p = payloads[k]
            if not p['sensor'] or p['sensor'] == 'None' or p['sensor'] == '':
                continue

            sensor_name = k
            sensor_type = p['sensor']
            model_prefix = sensor_name

            bridges_, nodes_ = self.sensor_bridges(
                    world_name, self.model_name, sensor_type, sensor_name, model_prefix)
            bridges.extend(bridges_)
            nodes.extend(nodes_)
        return bridges, nodes

    @staticmethod
    def sensor_bridges(world_name, model_name, payload, sensor_name, model_prefix=''):
        bridges = []
        nodes = []
        if payload in camera_models():
            bridges = [
                ign_assets.bridges.image(world_name, model_name, sensor_name,
                                         payload, model_prefix),
                ign_assets.bridges.camera_info(world_name, model_name,
                                               sensor_name, payload, model_prefix)
            ]
        elif payload in lidar_models():
            bridges = [
                ign_assets.bridges.lidar_scan(world_name, model_name, sensor_name, payload, model_prefix),
                ign_assets.bridges.lidar_points(world_name, model_name, sensor_name, payload, model_prefix)
            ]
        elif payload in rgbd_models():
            bridges = [
                ign_assets.bridges.image(world_name, model_name, sensor_name, payload, model_prefix),
                ign_assets.bridges.camera_info(world_name, model_name, sensor_name, payload, model_prefix),
                ign_assets.bridges.depth_image(world_name, model_name, sensor_name, payload, model_prefix),
                ign_assets.bridges.camera_points(world_name, model_name, sensor_name, payload, model_prefix)
            ]
        elif payload in gps_models():
            # bridges = [
            #     ign_assets.bridges.navsat(world_name, model_name, sensor_name, payload, model_prefix)
            # ]
            nodes.append(Node(
                package='ignition_assets',
                executable='gps_bridge',
                namespace=model_name,
                output='screen',
                parameters=[
                    {'world_name': world_name,
                     'name_space': model_name,
                     'sensor_name': sensor_name,
                     'link_name': payload,
                     'sensor_type': 'navsat'}
                ]
            ))
        elif payload in suction_gripper_models():
            bridges = [
                ign_assets.bridges.gripper_suction_control(model_name),
                ign_assets.bridges.gripper_contact(model_name, 'center'),
                ign_assets.bridges.gripper_contact(model_name, 'left'),
                ign_assets.bridges.gripper_contact(model_name, 'right'),
                ign_assets.bridges.gripper_contact(model_name, 'top'),
                ign_assets.bridges.gripper_contact(model_name, 'bottom')
            ]
        return bridges, nodes

    def set_flight_time(self, flight_time):
        # UAV specific, sets flight time
        self.flight_time = float(flight_time)

        # calculate battery capacity from time
        # capacity (Ah) = flight time (in hours) * load (watts) / voltage
        # assume constant voltage for battery to keep things simple for now.
        self.battery_capacity = (float(flight_time) / 60) * 6.6 / 12.694

    def set_payload(self, payload):
        self.payload = payload

    def generate(self):
        # Generate SDF by executing JINJA and populating templates

        # TODO: look for file in all IGN_GAZEBO_RESOURCE_PATH
        model_dir = os.path.join(get_package_share_directory('ignition_assets'), 'models')
        jinja_script = os.path.join(get_package_share_directory('ignition_assets'), 'scripts')

        payload = ""
        for sensor_name, sensor in self.payload.items():
            if 'sensor' not in sensor:
                continue
            sensor_type = sensor['sensor']

            x_s, y_s, z_s = 0, 0, 0
            if 'xyz' in sensor:
                x_s, y_s, z_s = sensor['xyz']

            roll_s, pitch_s, yaw_s = 0, 0, 0
            if 'rpy' in sensor:
                roll_s, pitch_s, yaw_s = sensor['rpy']

            payload += f"{sensor_name} {sensor_type} {x_s} {y_s} {z_s} {roll_s} {pitch_s} {yaw_s} "

        command = ['python3', f'{jinja_script}/jinja_gen.py', f'{model_dir}/{self.model_type}/{self.model_type}.sdf.jinja', \
            f'{model_dir}/..', '--namespace', f'{self.model_name}', '--sensors', f'{payload}', \
            '--battery', f'{self.flight_time}', '--output-file', f'/tmp/{self.model_type}_{self.n}.sdf']

        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output to see if there were undefined variables
        # for the JINJA process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        model_sdf = f"/tmp/{self.model_type}_{self.n}.sdf"
        return command, model_sdf

    def spawn_args(self, world_name, model_sdf=None):
        if not model_sdf:
            [command, model_sdf] = self.generate()

        return ['-world', world_name,
                '-file', model_sdf,
                '-name', self.model_name,
                '-allow_renaming', 'false',
                '-x', str(self.position[0]),
                '-y', str(self.position[1]),
                '-z', str(self.position[2]),
                '-R', str(self.position[3]),
                '-P', str(self.position[4]),
                '-Y', str(self.position[5])]

    @classmethod
    def FromConfig(cls, stream):
        # Generate a Model instance (or multiple instances) from a stream
        # Stream can be either a file input or string
        file_extension = stream.name.split('.')[-1]
        if file_extension in ['yaml', 'yml']:
            config = yaml.safe_load(stream)

            if type(config) == list:
                return cls._FromConfigList(config)
            elif type(config) == dict:
                return cls._FromConfigDict(config)
        elif file_extension in ['json']:
            config = json.load(stream)
            return cls._FromConfigListJson(config)

    @classmethod
    def _FromConfigList(cls, entries):
        # Parse an array of configurations
        ret = []
        for entry in entries:
            ret.append(cls._FromConfigDict(entry))
        return ret

    @classmethod
    def _FromConfigListJson(cls, config):
        ret = []
        for i, entry in enumerate(config['drones']):
            ret.append(cls._FromConfigDictJson(entry, i))
        return ret

    @classmethod
    def _FromConfigDict(cls, config):
        # Parse a single configuration
        if 'model_name' not in config:
            raise RuntimeError('Cannot construct model without model_name in config')
        if 'model_type' not in config:
            raise RuntimeError('Cannot construct model without model_type in config')

        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if 'position' not in config:
            print('Position not found in config, defaulting to (0, 0, 0), (0, 0, 0)')
        else:
            if 'xyz' in config['position']:
                xyz = config['position']['xyz']
            if 'rpy' in config['position']:
                rpy = config['position']['rpy']
        model = cls(config['model_name'], config['model_type'], [*xyz, *rpy])

        if 'flight_time' in config:
            model.set_flight_time(config['flight_time'])

        if 'payload' in config:
            model.set_payload(config['payload'])

        if 'gripper' in config:
            model.set_gripper(config['gripper'])

        return model

    @classmethod
    def _FromConfigDictJson(cls, config, n=0):
        if 'model' not in config:
            raise RuntimeError('Cannot construct model without model in config')
        if 'name' not in config:
            raise RuntimeError('Cannot construct model without name in config')

        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if 'xyz' in config:
            xyz = config['xyz']
        if 'rpy' in config:
            rpy = config['rpy']
        model = cls(config['name'], config['model'], n, [*xyz, *rpy])

        if 'flight_time' in config:
            model.set_flight_time(config['flight_time'])

        if 'payload' in config:
            model.set_payload(config['payload'])

        return model