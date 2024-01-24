"""test_models.py"""
import unittest

from pydantic import ValidationError
from as2_gazebo_assets.world import World
from as2_gazebo_assets.models.drone import Drone
from as2_gazebo_assets.models.payload import Payload
from as2_gazebo_assets.models.object import Object


def get_world(name, drones=[], objects=[]):
    """Build a World"""
    return World(world_name=name, drones=drones, objects=objects)


def get_drone(name, model, xyz=[0, 0, 0], rpy=[0, 0, 0], flight_time=0, payload=[]):
    """Build a Drone"""
    return Drone(model_name=name, model_type=model, xyz=xyz, rpy=rpy,
                 flight_time=flight_time, payload=payload)


def get_payload(name, model, xyz=[0, 0, 0], rpy=[0, 0, 0], payload=None):
    """Build a Payload"""
    return Payload(model_name=name, model_type=model, xyz=xyz, rpy=rpy, payload=payload)


def get_object(name, model, xyz=[0, 0, 0], rpy=[0, 0, 0], joints=[], object_bridges=[],
               tf_broadcaster=False, use_sim_time=True):
    """Build a Object"""
    return Object(model_name=name, model_type=model, xyz=xyz, rpy=rpy, joints=joints,
                  object_bridges=object_bridges, tf_broadcaster=tf_broadcaster,
                  use_sim_time=use_sim_time)


class Tester(unittest.TestCase):
    """Tester"""

    def test_empty_world(self):
        """Test an empty world"""
        empty_world_json = """
        {
            "world_name": "empty"
        }
        """
        world = World.parse_raw(empty_world_json)
        self.assertEqual(get_world(name="empty"), world)

    def test_one_drone_world(self):
        """Test world with a drone"""
        world_json = """
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
                        "rpy": [ 0.0, 0.0, 1.57 ]
                    }
                ]
            }
            ]
        }
        """
        world = World.parse_raw(world_json)
        same_world = get_world("empty",
                               drones=[
                                   get_drone(name="drone_sim_0",
                                             model="quadrotor_base",
                                             xyz=[0, 0, 0.2],
                                             rpy=[0, 0, 1.57],
                                             flight_time=60,
                                             payload=[
                                                 get_payload(name="front_camera", model="hd_camera",
                                                             xyz=[0.1, 0.2, 0.3]),
                                                 get_payload(name="lidar_0", model="lidar_3d",
                                                             rpy=[0.0, 0.0, 1.57]),
                                             ]),
                               ]
                               )
        self.assertEqual(world, same_world)

    def test_multi_drone_world(self):
        """Test world with several drones"""
        world_json = """
        {
            "world_name": "empty",
            "drones": [
            {
                "model_type": "quadrotor_base",
                "model_name": "drone_sim_0",
                "xyz": [ 0.0, 0.0, 0.2 ]
            },
            {
                "model_type": "quadrotor_base",
                "model_name": "drone_sim_1",
                "xyz": [ 2.0, 0.0, 0.2 ]
            },
            {
                "model_type": "quadrotor_base",
                "model_name": "drone_sim_2",
                "xyz": [ 4.0, 0.0, 0.2 ]
            },
            {
                "model_type": "hexrotor_base",
                "model_name": "drone_sim_3",
                "xyz": [ 6.0, 0.0, 0.2 ]
            }
            ]
        }
        """
        world = World.parse_raw(world_json)
        same_world = get_world("empty",
                               drones=[
                                   get_drone(name="drone_sim_0",
                                             model="quadrotor_base",
                                             xyz=[0, 0, 0.2]),
                                   get_drone(name="drone_sim_1",
                                             model="quadrotor_base",
                                             xyz=[2, 0, 0.2]),
                                   get_drone(name="drone_sim_2",
                                             model="quadrotor_base",
                                             xyz=[4, 0, 0.2]),
                                   get_drone(name="drone_sim_3",
                                             model="hexrotor_base",
                                             xyz=[6, 0, 0.2])
                               ])
        self.assertEqual(world, same_world)

    def test_drone_and_object_world(self):
        """Test world with a drone and an object"""
        world_json = """
        {
            "world_name": "empty",
            "drones": [
            {
                "model_name": "drone_sim_0",
                "model_type": "quadrotor_base"
            }
            ],
            "objects": [
                {    
                    "model_name": "gate_0",
                    "model_type": "aruco_gate_1",
                    "xyz": [ 5.0, 5.0, 0.3 ],
                    "rpy": [ 0.0, 0.0, -1.57 ],
                    "object_bridges": ["pose"]
                }
            ]
        }
        """
        world = World.parse_raw(world_json)
        same_world = get_world(
            "empty",
            drones=[get_drone(name="drone_sim_0", model="quadrotor_base")],
            objects=[
                get_object(
                    name="gate_0",
                    model="aruco_gate_1",
                    xyz=[5.0, 5.0, 0.3],
                    rpy=[0.0, 0.0, -1.57],
                    object_bridges=["pose"],
                )
            ],
        )
        print(same_world.json())
        print(world.json())
        self.assertEqual(world, same_world)

    def test_gimbaled_sensor_world(self):
        """Test a world with a drone and a gimbaled sensor"""
        world_json = """
        {
            "world_name": "empty",
            "drones": [
                {
                    "model_type": "quadrotor_base",
                    "model_name": "drone0",
                    "xyz": [
                        -2.0,
                        0.0,
                        0.3
                    ],
                    "rpy": [
                        0,
                        0,
                        0.0
                    ],
                    "flight_time": 60,
                    "payload": [
                        {
                            "model_name": "gimbal",
                            "model_type": "gimbal_speed",
                            "payload": {
                                "model_name": "hd_camera",
                                "model_type": "hd_camera"
                            }
                        }
                    ]
                }
            ]
        }
        """
        world = World.parse_raw(world_json)
        same_world = get_world(
            "empty",
            drones=[
                get_drone(
                    name="drone0",
                    model="quadrotor_base",
                    xyz=[-2.0, 0.0, 0.3],
                    rpy=[0, 0, 0],
                    flight_time=60,
                    payload=[
                        get_payload(
                            name="gimbal",
                            model="gimbal_speed",
                            payload=get_payload(
                                name="hd_camera", model="hd_camera"),
                        )
                    ],
                )
            ],
        )
        self.assertEqual(world, same_world)

    def test_multi_drone_gimbaled_sensor_world(self):
        "Test a world with multiple drones and every drone with a gimbal"
        world_json = """
        {
            "world_name": "empty",
            "drones": [
                {
                    "model_type": "quadrotor_base",
                    "model_name": "drone0",
                    "xyz": [
                        -2.0,
                        0.0,
                        0.3
                    ],
                    "rpy": [
                        0,
                        0,
                        0.0
                    ],
                    "flight_time": 60,
                    "payload": [
                        {
                            "model_name": "hd_camera2",
                            "model_type": "hd_camera"
                        },
                        {
                            "model_name": "gimbal",
                            "model_type": "gimbal_speed",
                            "payload": {
                                "model_name": "hd_camera",
                                "model_type": "hd_camera"
                            }
                        }
                    ]
                },
                {
                    "model_type": "quadrotor_base",
                    "model_name": "drone1",
                    "xyz": [
                        2.0,
                        0.0,
                        0.3
                    ],
                    "rpy": [
                        0,
                        0,
                        0.0
                    ],
                    "flight_time": 60,
                    "payload": [
                        {
                            "model_name": "hd_camera2",
                            "model_type": "hd_camera"
                        },
                        {
                            "model_name": "gimbal",
                            "model_type": "gimbal_speed",
                            "payload": {
                                "model_name": "hd_camera",
                                "model_type": "hd_camera"
                            }
                        }
                    ]
                },
                {
                    "model_type": "quadrotor_base",
                    "model_name": "drone2",
                    "xyz": [
                        0.0,
                        0.0,
                        0.3
                    ],
                    "rpy": [
                        0,
                        0,
                        0.0
                    ],
                    "flight_time": 60,
                    "payload": [
                        {
                            "model_name": "hd_camera2",
                            "model_type": "hd_camera"
                        },
                        {
                            "model_name": "gimbal",
                            "model_type": "gimbal_position",
                            "xyz": [
                                0.0,
                                0.0,
                                0.0
                            ],
                            "rpy": [
                                0.0,
                                0.0,
                                0.0
                            ],
                            "payload": {
                                "model_name": "hd_camera",
                                "model_type": "hd_camera",
                                "xyz": [
                                    0.0,
                                    0.0,
                                    0.0
                                ],
                                "rpy": [
                                    0.0,
                                    0.0,
                                    0.0
                                ]
                            }
                        }
                    ]
                }
            ]
        }
        """
        world = World.parse_raw(world_json)
        same_world = get_world(
            "empty",
            drones=[
                get_drone(
                    name="drone0",
                    model="quadrotor_base",
                    xyz=[-2.0, 0.0, 0.3],
                    rpy=[0, 0, 0],
                    flight_time=60,
                    payload=[
                        get_payload(name="hd_camera2", model="hd_camera"),
                        get_payload(
                            name="gimbal",
                            model="gimbal_speed",
                            payload=get_payload(
                                name="hd_camera", model="hd_camera"),
                        ),
                    ],
                ),
                get_drone(
                    name="drone1",
                    model="quadrotor_base",
                    xyz=[2.0, 0.0, 0.3],
                    rpy=[0, 0, 0],
                    flight_time=60,
                    payload=[
                        get_payload(name="hd_camera2", model="hd_camera"),
                        get_payload(
                            name="gimbal",
                            model="gimbal_speed",
                            payload=get_payload(
                                name="hd_camera", model="hd_camera"),
                        ),
                    ],
                ),
                get_drone(
                    name="drone2",
                    model="quadrotor_base",
                    xyz=[0.0, 0.0, 0.3],
                    rpy=[0, 0, 0],
                    flight_time=60,
                    payload=[
                        get_payload(name="hd_camera2", model="hd_camera"),
                        get_payload(
                            name="gimbal",
                            model="gimbal_position",
                            xyz=[0, 0, 0],
                            rpy=[0, 0, 0],
                            payload=get_payload(
                                name="hd_camera",
                                model="hd_camera",
                                xyz=[0, 0, 0],
                                rpy=[0, 0, 0],
                            ),
                        ),
                    ],
                ),
            ],
        )
        self.assertEqual(world, same_world)

    def test_incorrect_gimbaled_sensor_world(self):
        """Test a world with a drone and a gimbaled sensor"""
        world_json = """
        {
            "world_name": "empty",
            "drones": [
                {
                    "model_type": "quadrotor_base",
                    "model_name": "drone0",
                    "xyz": [
                        -2.0,
                        0.0,
                        0.3
                    ],
                    "rpy": [
                        0,
                        0,
                        0.0
                    ],
                    "flight_time": 60,
                    "payload": [
                        {
                            "model_name": "gimbal",
                            "model_type": "hd_camera",
                            "payload": {
                                "model_name": "hd_camera",
                                "model_type": "hd_camera"
                            }
                        }
                    ]
                }
            ]
        }
        """
        with self.assertRaises(ValueError):
            World.parse_raw(world_json)

    def test_empty_gimbal_world(self):
        """Test a world with a drone and a gimbaled sensor"""
        world_json = """
        {
            "world_name": "empty",
            "drones": [
                {
                    "model_type": "quadrotor_base",
                    "model_name": "drone0",
                    "xyz": [
                        -2.0,
                        0.0,
                        0.3
                    ],
                    "rpy": [
                        0,
                        0,
                        0.0
                    ],
                    "flight_time": 60,
                    "payload": [
                        {
                            "model_name": "gimbal",
                            "model_type": "gimbal_speed"
                        }
                    ]
                }
            ]
        }
        """
        with self.assertRaises(ValueError):
            World.parse_raw(world_json)


if __name__ == "__main__":
    unittest.main()
