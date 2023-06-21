"""
world.py
"""

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


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import os
import codecs
import subprocess
from typing import Union, List
from pathlib import Path
from pydantic import BaseModel, root_validator
from ament_index_python.packages import get_package_share_directory
from ign_assets.models.object import Object
from ign_assets.models.drone import Drone, DroneTypeEnum
from ign_assets.models.payload import Payload


class Origin(BaseModel):
    """GPS Point"""
    latitude: float
    longitude: float
    altitude: float


class World(BaseModel):
    """Gz World"""
    world_name: str
    origin: Origin = None
    drones: List[Drone] = []
    objects: List[Object] = []

    @root_validator
    def check_world_values(cls, values: dict):
        """Get world jinja file if exists"""
        is_jinja, jinja_templ_path = cls.get_world_file(values["world_name"])
        if is_jinja:
            _, values["world_path"] = cls.generate(
                values["world_name"], values["origin"], jinja_templ_path)
        return values

    def __str__(self) -> str:
        drones_str = ""
        for drone in self.drones:
            drones_str += f"\n\t{drone}"
        return f"{self.world_name}:{drones_str}"

    def get_drone_index(self, drone: Drone) -> int:
        """Get drone index"""
        return self.drones.index(drone)

    def get_object_index(self, object_: Object) -> int:
        """Get object index"""
        return self.drones.index(object_)

    # TODO: use generic get_assets_file() and merge with get_model_file()
    @staticmethod
    def get_world_file(world_name: str) -> Path:
        """Return Path of self jinja template"""
        # Concatenate the model directory and the IGN_GAZEBO_RESOURCE_PATH environment variable
        world_dir = Path(get_package_share_directory(
            'as2_ign_gazebo_assets'), 'worlds')
        resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH')

        paths = [world_dir]
        if resource_path:
            paths += [Path(p) for p in resource_path.split(':')]

        # Define the filename to look for
        filename = f'{world_name}.sdf.jinja'

        # Loop through each directory and check if the jinja file exists
        for path in paths:
            filepath = path / filename
            if filepath.is_file():
                # If the file exists, return is_jinja is true and the path
                return True, filepath

        # Loop through each directory and check if the sdf file exists
        filename = f'{world_name}.sdf'

        for path in paths:
            filepath = path / filename
            if filepath.is_file():
                # If the file exists, returns is_jinja is false, filepath won't be used
                return False, filepath

        raise FileNotFoundError(
            f'neither {world_name}.sdf and {world_name}.sdf.jinja not found in {paths}.')

    @staticmethod
    def generate(world_name: str, origin: Origin, jinja_template_path: Path) -> tuple[str, str]:
        """Generate SDF by executing JINJA and populating templates

        :raises RuntimeError: if jinja fails
        :return: python3 jinja command and path to model_sdf generated
        """

        # Concatenate the world directory and the IGN_GAZEBO_RESOURCE_PATH environment variable
        # world_dir = Path(get_package_share_directory(
        #     'as2_ign_gazebo_assets'), 'worlds')
        env_dir = jinja_template_path.parent
        jinja_script = os.path.join(
            get_package_share_directory('as2_ign_gazebo_assets'), 'scripts')

        origin_str = ""
        if origin is not None:
            origin_str += f"{origin.latitude} {origin.longitude} {origin.altitude}"

        output_file_sdf = f"/tmp/{world_name}.sdf"
        command = ['python3', f'{jinja_script}/jinja_gen.py', jinja_template_path,
                   f'{env_dir}', '--origin', f'{origin_str}',
                   '--output-file', f'{output_file_sdf}']

        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output to see if there were undefined variables
        # for the JINJA process
        # print(process.communicate()[0])
        stderr = process.communicate()[1]

        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]

        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        return command, output_file_sdf


def spawn_args(world: World, model: Union[Drone, Object]) -> List[str]:
    """Return args to spawn model_sdf in Gz"""
    command, model_sdf = model.generate(world)
    return ['-world', world.world_name,
            '-file', model_sdf,
            '-name', model.model_name,
            '-allow_renaming', 'false',
            '-x', str(model.xyz[0]),
            '-y', str(model.xyz[1]),
            '-z', str(model.xyz[2]),
            '-R', str(model.rpy[0]),
            '-P', str(model.rpy[1]),
            '-Y', str(model.rpy[2])]


def dummy_world() -> World:
    """Create dummy world
    """
    drone = Drone(model_name="dummy",
                  model_type=DroneTypeEnum.QUADROTOR, flight_time=60)
    cam = Payload(model_name="front_camera", model_type="hd_camera")
    gps = Payload(model_name="gps0", model_type="gps")
    drone.payload.append(cam)
    drone.payload.append(gps)

    world = World(world_name="empty", drones=[drone])
    return world


if __name__ == "__main__":
    WORLD_JSON = """
    {
        "world_name": "empty",
        "origin": {"latitude": 10.0, "longitude": 9.0, "altitude": 8.0},
        "drones": [
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_sim_0",
            "xyz": [ 0.0, 0.0, 0.2 ],
            "rpy": [ 0, 0, 1.57 ],
            "flight_time": 60,
            "payload": [
                {
                    "model_name": "front_camera",
                    "model_type": "hd_camera",
                    "xyz": [0.1, 0.2, 0.3]
                },
                {
                    "model_name": "lidar_0",
                    "model_type": "lidar_3d",
                    "rpy": [ 0.0, 0.0, 0.0 ]
                }
            ]
        },
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_sim_1",
            "xyz": [ 3.0, 0.0, 0.2 ],
            "rpy": [ 0, 0, 1.57 ],
            "payload": [
                {
                    "model_name": "camera",
                    "model_type": "hd_camera",
                    "rpy": [ 0.0, 0.0, 0.0 ]
                },
                {
                    "model_name": "gps0",
                    "model_type": "gps",
                    "xyz": [ 0.0, 0.0, 0.08 ]
                }
            ]
        }
        ]
    }
    """
    world_model = World.parse_raw(WORLD_JSON)

    for drone_ in world_model.drones:
        _, sdf = drone_.generate(world_model)
        print(sdf)
