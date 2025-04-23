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
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
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

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import json
import tempfile
import unittest

from as2_visualization.drone_viz import AdapterBuilder, VizInfo
from as2_visualization.rviz_adapter import RvizAdapter
from as2_visualization.viz_parsing import JSONParser
from geometry_msgs.msg import Point
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker


class TestBuilding(unittest.TestCase):
    ADAPTERS_FILE = """
def func(x):
    return x+1
"""
    PRESET_ADAPTERS_TEST_FILE = """
{
"adapters" : {
    "custom" : {},
    "preset" : [
        {
            "in_topic" : "in_preset_test_topic",
            "out_topic" : "out_preset_test_topic",
            "id" : "test_adapter",
            "preset_type" : "PointAdapter",
            "sub_cfg" : {
                "namespaces" : ["test_namespace"],
                "depth" : 15,
                "durability" : "VOLATILE",
                "filtersize" : 20,
                "history" : "KEEP_LAST",
                "reliability" : "RELIABLE",
                "value" : true
            },
            "pub_cfg" : {
                "namespaces" : ["test_namespace2"],
                "depth" : 15,
                "durability" : "VOLATILE",
                "filtersize" : 20,
                "history" : "KEEP_LAST",
                "reliability" : "RELIABLE",
                "value" : true
            }
        }
    ]
},
"drones" : {
    "drone_0" : ["test_adapter"]
},
"adapters_per_process" : 1
}
"""
    CUSTOM_ADAPTERS_TEST_FILE = """
{
"adapters" : {
"preset" : {},
"custom" : [
    {
        "in_topic" : "in_test_topic",
        "out_topic" : "out_test_topic",
        "id" : "test_adapter",
        "sub_cfg" : {
            "namespaces" : ["test_namespace"],
            "depth" : 15,
            "durability" : "VOLATILE",
            "filtersize" : 20,
            "history" : "KEEP_LAST",
            "reliability" : "RELIABLE",
            "value" : true
        },
        "pub_cfg" : {
            "namespaces" : ["test_namespace2"],
            "depth" : 15,
            "durability" : "VOLATILE",
            "filtersize" : 20,
            "history" : "KEEP_LAST",
            "reliability" : "RELIABLE",
            "value" : true
        },
        "in_msg" : "geometry_msgs/Point",
        "out_msg" : "visualization_msgs/Marker",
        "adapter" : {
            "path" : "/root/emlanding_ws/project_emlanding/config_ground_station/adapters.py",
            "name" : "adapters",
            "func_name" : "func"
        }
    }
]
},
"drones" : {
"drone_0" : ["test_adapter"],
"drone_1" : ["test_adapter"]
},
"adapters_per_process" : 1
}
"""
    BOTH_ADAPTERS_TEST_FILE = """
{
"adapters" : {
    "preset" : [
        {
            "in_topic" : "in_preset_test_topic",
            "out_topic" : "out_preset_test_topic",
            "id" : "test_preset_adapter",
            "preset_type" : "PointAdapter",
            "sub_cfg" : {
                "namespaces" : ["test_namespace"],
                "depth" : 15,
                "durability" : "VOLATILE",
                "filtersize" : 20,
                "history" : "KEEP_LAST",
                "reliability" : "RELIABLE",
                "value" : true
            },
            "pub_cfg" : {
                "namespaces" : ["test_namespace2"],
                "depth" : 15,
                "durability" : "VOLATILE",
                "filtersize" : 20,
                "history" : "KEEP_LAST",
                "reliability" : "RELIABLE",
                "value" : true
            }
        }
    ],
    "custom" : [
        {
            "in_topic" : "in_custom_test_topic",
            "out_topic" : "out_custom_test_topic",
            "id" : "test_custom_adapter",
            "sub_cfg" : {
                "namespaces" : ["test_namespace"],
                "depth" : 15,
                "durability" : "VOLATILE",
                "filtersize" : 20,
                "history" : "KEEP_LAST",
                "reliability" : "RELIABLE",
                "value" : true
            },
            "pub_cfg" : {
                "namespaces" : ["test_namespace2"],
                "depth" : 15,
                "durability" : "VOLATILE",
                "filtersize" : 20,
                "history" : "KEEP_LAST",
                "reliability" : "RELIABLE",
                "value" : true
            },
            "in_msg" : "geometry_msgs/Point",
            "out_msg" : "visualization_msgs/Marker",
            "adapter" : {
                "path" : "/root/emlanding_ws/project_emlanding/config_ground_station/adapters.py",
                "name" : "adapters",
                "func_name" : "func"
            }
        }
    ]
},
"drones" : {
    "drone_0" : ["test_preset_adapter"],
    "drone_1" : ["test_custom_adapter"],
    "drone_2" : ["test_custom_adapter", "test_preset_adapter"]
},
"adapters_per_process" : 1
}"""

    def setUp(self):
        self.f_adapters = tempfile.NamedTemporaryFile(mode='w+', suffix='.py')
        self.f_adapters.write(self.ADAPTERS_FILE)
        self.f_adapters.flush()
        self.f_adapters.seek(0)

        self.f_preset = tempfile.NamedTemporaryFile(mode='w+')
        json.dump(json.loads(self.PRESET_ADAPTERS_TEST_FILE), self.f_preset)
        self.f_preset.flush()
        self.f_preset.seek(0)

        self.f_custom = tempfile.NamedTemporaryFile(mode='w+')
        jdict = json.loads(self.CUSTOM_ADAPTERS_TEST_FILE)
        jdict['adapters']['custom'][0]['adapter'] = {
            'path': self.f_adapters.name,
            'name': self.f_adapters.name.split('/')[-1],
            'func_name': 'func',
        }
        json.dump(jdict, self.f_custom)
        self.f_custom.flush()
        self.f_custom.seek(0)

        self.f_both = tempfile.NamedTemporaryFile(mode='w+')
        jdict = json.loads(self.BOTH_ADAPTERS_TEST_FILE)
        jdict['adapters']['custom'][0]['adapter'] = {
            'path': self.f_adapters.name,
            'name': self.f_adapters.name.split('/')[-1],
            'func_name': 'func',
        }
        json.dump(jdict, self.f_both)
        self.f_both.flush()
        self.f_both.seek(0)

        rclpy.init()

    def test_custom_builder(self):
        parser: JSONParser = JSONParser(self.f_custom.name)

        vinfo: VizInfo = VizInfo()
        vinfo = parser.insertToVizInfo(vinfo)

        builder: AdapterBuilder = AdapterBuilder()

        adapters = vinfo.generate_adapters(builder)

        assert len(adapters) == 2
        ad: RvizAdapter = adapters[0]
        assert ad.in_topic == '/drone_0/in_test_topic'
        assert ad.out_topic == '/viz/drone_0/out_test_topic'
        assert ad.qos_publisher.depth == 15
        assert ad.qos_publisher.durability == DurabilityPolicy.VOLATILE
        assert ad.qos_publisher.history == HistoryPolicy.KEEP_LAST
        assert ad.qos_publisher.reliability == ReliabilityPolicy.RELIABLE
        assert ad.adapter_f(1) == 2
        assert ad.in_topic_type == Point
        assert ad.out_topic_type == Marker
        ad: RvizAdapter = adapters[1]
        assert ad.in_topic == '/drone_1/in_test_topic'
        assert ad.out_topic == '/viz/drone_1/out_test_topic'
        assert ad.qos_publisher.depth == 15
        assert ad.qos_publisher.durability == DurabilityPolicy.VOLATILE
        assert ad.qos_publisher.history == HistoryPolicy.KEEP_LAST
        assert ad.qos_publisher.reliability == ReliabilityPolicy.RELIABLE
        assert ad.adapter_f(1) == 2
        assert ad.in_topic_type == Point
        assert ad.out_topic_type == Marker

    def test_preset_builder(self):
        parser: JSONParser = JSONParser(self.f_preset.name)

        vinfo: VizInfo = VizInfo()
        vinfo = parser.insertToVizInfo(vinfo)

        builder: AdapterBuilder = AdapterBuilder()

        adapters = vinfo.generate_adapters(builder)

        assert len(adapters) == 1
        ad: RvizAdapter = adapters[0]
        assert ad.in_topic == '/drone_0/in_preset_test_topic'
        assert ad.out_topic == '/viz/drone_0/out_preset_test_topic'
        assert ad.qos_publisher.depth == 15
        assert ad.qos_publisher.durability == DurabilityPolicy.VOLATILE
        assert ad.qos_publisher.history == HistoryPolicy.KEEP_LAST
        assert ad.qos_publisher.reliability == ReliabilityPolicy.RELIABLE
        point: Point = Point()
        point.x = 0.1
        point.y = 0.2
        point.z = 0.3
        m: Marker = ad.adapter_f(point)
        assert m.pose.position.x == 0.1
        assert m.pose.position.y == 0.2
        assert m.pose.position.z == 0.3

    def test_builder(self):
        parser: JSONParser = JSONParser(self.f_both.name)

        vinfo: VizInfo = VizInfo()
        vinfo = parser.insertToVizInfo(vinfo)

        builder: AdapterBuilder = AdapterBuilder()

        adapters = vinfo.generate_adapters(builder)
        for ad in adapters:
            print(ad)
        assert len(adapters) == 4

        l2 = list(
            filter(lambda ad: 'drone_2' in ad.in_topic and 'drone_2' in ad.out_topic, adapters)
        )
        assert len(l2) == 2

        l3 = list(filter(lambda ad: 'in_preset_test_topic' in ad.in_topic, l2))
        assert len(l3) == 1

    def tearDown(self):
        self.f_preset.close()
        self.f_custom.close()
        self.f_both.close()
        rclpy.shutdown()
