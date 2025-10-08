"""Interpreter test."""

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

import time
import unittest

from as2_python_api.mission_interpreter.mission import Mission, MissionItem
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
import rclpy


class TestDummyMission(unittest.TestCase):
    """Mission testing with dummy items."""

    @classmethod
    def setUpClass(cls):
        """Set up class."""
        dummy_mission = """
        {
            "target": "drone_0",
            "plan": [
                {
                    "behavior": "dummy",
                    "method": "__call__",
                    "args": {
                        "arg1": 1.0,
                        "arg2": 2.0,
                        "wait": "True"
                    }
                },
                {
                    "behavior": "dummy",
                    "args": {
                        "arg2": 98.0,
                        "arg1": 99.0,
                        "wait": "False"
                    }
                },
                {
                    "behavior": "dummy",
                    "method": "stop",
                    "args": {
                    }
                }
            ]
        }"""
        cls.mission = Mission.parse_raw(dummy_mission)

        rclpy.init()
        cls.interpreter = MissionInterpreter(verbose=True)
        cls.interpreter.load_mission(0, cls.mission)

    @classmethod
    def tearDownClass(cls):
        cls.interpreter.shutdown()
        rclpy.shutdown()

    def test_mission(self):
        """Test mission."""
        self.assertEqual(self.mission.target, 'drone_0')
        self.assertEqual(len(self.mission.plan), 3)

    def test_mission_stack(self):
        """Test mission stack."""
        stack = self.mission.stack
        item = stack.next_item()
        self.assertEqual(item.behavior, 'dummy')
        self.assertEqual(item.method, '__call__')
        self.assertEqual(item.args, {'arg1': 1.0, 'arg2': 2.0, 'wait': 'True'})

        item = stack.next_item()
        self.assertEqual(
            item, MissionItem(behavior='dummy', args={'arg2': 98.0, 'arg1': 99.0, 'wait': 'False'})
        )

        item = stack.next_item()
        self.assertEqual(item, MissionItem(behavior='dummy', method='stop', args={}))

    def test_drone(self):
        """Test drone modules."""
        self.assertEqual(self.interpreter.drone.namespace, 'drone_0')
        self.assertEqual(sorted(self.interpreter.drone.modules.keys()), ['dummy'])

    def test_start_mission(self):
        """Test mission start."""
        self.interpreter.start_mission(0)
        time.sleep(0.1)
        self.assertEqual(len(self.interpreter.mission_stack.pending), 2)
        self.assertEqual(len(self.interpreter.mission_stack.done), 0)
        self.assertEqual(
            self.interpreter.mission_stack.current,
            MissionItem(
                behavior='dummy',
                method='__call__',
                args={'arg1': 1.0, 'arg2': 2.0, 'wait': 'True'},
            ),
        )
        self.interpreter.next_item(0)
        self.interpreter.stop_mission(0)


class TestInterpreterModify(unittest.TestCase):
    """Test modifying pending items in the interpreter."""

    def setUp(self):
        """Set up class."""
        dummy_mission = """
        {
            "target": "drone_0",
            "plan": [
                {
                    "behavior": "dummy",
                    "method": "__call__",
                    "args": {
                        "arg1": 1.0,
                        "arg2": 2.0,
                        "wait": "True"
                    }
                },
                {
                    "behavior": "dummy",
                    "args": {
                        "arg2": 98.0,
                        "arg1": 99.0,
                        "wait": "False"
                    }
                },
                {
                    "behavior": "dummy",
                    "method": "stop",
                    "args": {
                    }
                }
            ]
        }"""
        self.mission = Mission.parse_raw(dummy_mission)

        rclpy.init()
        self.interpreter = MissionInterpreter(verbose=True)
        self.interpreter.load_mission(0, self.mission)

    def tearDown(self):
        self.interpreter.shutdown()
        rclpy.shutdown()

    def test_modify_pending(self):
        """Test modifying pending items."""
        print('Current idx = ', self.interpreter.mission_stack.current_idx)
        success_same_beh = self.interpreter.modify(
            1,
            0,
            MissionItem(
                behavior='dummy',
                method='__call__',
                args={'arg1': 100.0, 'arg2': 200.0, 'wait': 'True'},
            ),
        )
        self.assertTrue(success_same_beh)
        _ = self.interpreter.mission_stack.next_item()
        item = self.interpreter.mission_stack.next_item()
        self.assertEqual(
            item,
            MissionItem(
                behavior='dummy',
                method='__call__',
                args={'arg1': 100.0, 'arg2': 200.0, 'wait': 'True'},
            ),
        )

    def test_missmatch_args(self):
        """Test modifying pending items."""
        success_dif_args = self.interpreter.modify(
            1,
            0,
            MissionItem(
                behavior='dummy',
                method='__call__',
                args={'arg_test': 100.0, 'arg2': 200.0, 'wait': 'True'},
            ),
        )
        self.assertFalse(success_dif_args)

    def test_missmatch_type(self):
        """Test modifying pending items."""
        success_dif_type = self.interpreter.modify(
            1,
            0,
            MissionItem(
                behavior='dummy',
                method='__call__',
                args={'arg1': 'test', 'arg2': 200.0, 'wait': 'True'},
            ),
        )
        self.assertFalse(success_dif_type)

    def test_modify_done(self):
        _ = self.interpreter.mission_stack.next_item()
        _ = self.interpreter.mission_stack.next_item()
        success_done_idx = self.interpreter.modify(
            0,
            0,
            MissionItem(behavior='dummy', method='__call__', args={'arg1': 100.0, 'arg2': 200.0}),
        )
        self.assertFalse(success_done_idx)

    def test_bad_item_idx(self):
        success_bad_item_idx = self.interpreter.modify(
            1000,
            0,
            MissionItem(behavior='dummy', method='__call__', args={'arg1': 100.0, 'arg2': 200.0}),
        )
        self.assertFalse(success_bad_item_idx)

    def test_missing_mission(self):
        success_missing_mission = self.interpreter.modify(
            1,
            2,
            MissionItem(behavior='dummy', method='__call__', args={'arg1': 100.0, 'arg2': 200.0}),
        )
        self.assertFalse(success_missing_mission)

    def test_different_behavior(self):
        success_dif_beh = self.interpreter.modify(
            1,
            0,
            MissionItem(
                behavior='dummy200', method='__call__', args={'arg1': 100.0, 'arg2': 200.0}
            ),
        )
        self.assertFalse(success_dif_beh)

    def test_modify_different_method(self):
        success_dif_method = self.interpreter.modify(
            1,
            0,
            MissionItem(
                behavior='dummy', method='dummymethod', args={'arg1': 100.0, 'arg2': 200.0}
            ),
        )
        self.assertFalse(success_dif_method)


class TestMission(unittest.TestCase):
    """Mission testing."""

    def test_load_modules(self):
        """Test if modules are loaded correctly."""
        load_modules_mission = """
        {
            "target": "drone_sim_0",
            "plan": [
                {
                    "behavior": "takeoff",
                    "args": {
                        "height": 2.0,
                        "speed": 1.0
                    }
                },
                {
                    "behavior": "go_to",
                    "args": {
                        "x": 2.0,
                        "y": 2.0,
                        "z": 2.0,
                        "speed": 1.0
                    }
                },
                {
                    "behavior": "go_to",
                    "args": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 2.0,
                        "speed": 1.0
                    }
                },
                {
                    "behavior": "land",
                    "args": {
                        "speed": 1.0
                    }
                }
            ]
        }"""

        mission = Mission.parse_raw(load_modules_mission)

        rclpy.init()
        interpreter = MissionInterpreter(verbose=True)
        interpreter.load_mission(0, mission)
        assert sorted(interpreter.drone.modules.keys()) == ['go_to', 'land', 'takeoff']

        assert [item.behavior for item in interpreter.mission_stack.pending] == [
            'takeoff',
            'go_to',
            'go_to',
            'land',
        ]
        interpreter.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
