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


from typing import Union
from pydantic import BaseModel
from ign_assets.models.object import Object
from ign_assets.models.drone import Drone, DroneTypeEnum
from ign_assets.models.payload import Payload


class World(BaseModel):
    """Gz World"""
    world_name: str
    drones: list[Drone] = []
    objects: list[Object] = []

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


def spawn_args(world: World, model: Union[Drone, Object]) -> list[str]:
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
    print(world_model)

    # for drone in world_model.drones:
    #     _, sdf = drone.generate(world_model)
    #     print(sdf)
    print(dummy_world())
    print(dict(dummy_world()))
