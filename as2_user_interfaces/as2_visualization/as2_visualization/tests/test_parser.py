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

from as2_visualization.viz_params import CustomAdapterParams, PresetAdapterParams
from as2_visualization.viz_parsing import JSONParser


class TestParser(unittest.TestCase):
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
            "in_topic" : "in_test_topic",
            "out_topic" : "out_test_topic",
            "id" : "test_adapter",
            "preset_type" : "CrashingPointAdapter",
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
            "id" : "test_custom_adapter",
            "in_topic" : "in_test_topic",
            "id" : "test_adapter",
            "out_topic" : "out_test_topic",
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
    "drone_0" : ["test_adapter"]
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

    def test_preset_parsing(self):
        parser: JSONParser = JSONParser(self.f_preset.name)

        assert len(parser.drones.keys()) == 1
        assert 'drone_0' in parser.drones.keys()
        custom_adapters = parser.drones['drone_0']['custom']
        assert len(custom_adapters) == 0
        preset_adapters = parser.drones['drone_0']['preset']
        assert len(preset_adapters) == 1
        adapters: list[PresetAdapterParams] = list(preset_adapters)  # type: ignore
        a: PresetAdapterParams = adapters[0]
        assert a.adapter_name == 'test_adapter'
        assert a.in_topic == 'in_test_topic'
        assert a.out_topic == 'out_test_topic'
        assert a.preset_type == 'CrashingPointAdapter'
        assert a.sub_cfg.namespaces[0] == 'test_namespace'
        assert a.sub_cfg.depth == 15
        assert a.sub_cfg.durability == 'VOLATILE'
        assert a.sub_cfg.filtersize == 20
        assert a.sub_cfg.history == 'KEEP_LAST'
        assert a.sub_cfg.reliability == 'RELIABLE'

        assert a.pub_cfg.namespaces[0] == 'test_namespace2'
        assert a.pub_cfg.depth == 15
        assert a.pub_cfg.durability == 'VOLATILE'
        assert a.pub_cfg.filtersize == 20
        assert a.pub_cfg.history == 'KEEP_LAST'
        assert a.pub_cfg.reliability == 'RELIABLE'

    def test_custom_parsing(self):
        parser: JSONParser = JSONParser(self.f_custom.name)

        assert len(parser.drones.keys()) == 1
        assert 'drone_0' in parser.drones.keys()
        custom_adapters = parser.drones['drone_0']['custom']
        assert len(custom_adapters) == 1
        preset_adapters = parser.drones['drone_0']['preset']
        assert len(preset_adapters) == 0
        adapters: list[CustomAdapterParams] = list(custom_adapters)  # type: ignore
        a: CustomAdapterParams = adapters[0]
        assert a.adapter_name == 'test_adapter'
        assert a.in_topic == 'in_test_topic'
        assert a.out_topic == 'out_test_topic'
        assert a.sub_cfg.namespaces[0] == 'test_namespace'
        assert a.sub_cfg.depth == 15
        assert a.sub_cfg.durability == 'VOLATILE'
        assert a.sub_cfg.filtersize == 20
        assert a.sub_cfg.history == 'KEEP_LAST'
        assert a.sub_cfg.reliability == 'RELIABLE'

        assert a.pub_cfg.namespaces[0] == 'test_namespace2'
        assert a.pub_cfg.depth == 15
        assert a.pub_cfg.durability == 'VOLATILE'
        assert a.pub_cfg.filtersize == 20
        assert a.pub_cfg.history == 'KEEP_LAST'
        assert a.pub_cfg.reliability == 'RELIABLE'

        assert a.in_msg_type_name == 'geometry_msgs/Point'
        assert a.out_msg_type_name == 'visualization_msgs/Marker'
        assert a.adapter(1) == 2

    def tearDown(self):
        self.f_preset.close()
        self.f_custom.close()
