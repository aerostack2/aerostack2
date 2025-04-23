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


from dataclasses import field
from typing import Callable

from pydantic.dataclasses import dataclass


@dataclass
class TopicParams:
    """Dataclass for topic qos parameters."""

    namespaces: list = field(default_factory=list)
    depth: int = 10
    durability: str = 'VOLATILE'
    filtersize: int = 5
    history: str = 'KEEP_LAST'
    reliability: str = 'RELIABLE'
    value: bool = True

    def __post_init__(self):
        """Check if params are correctly set."""
        ...

    # Check if str values for depth, durability... are valid

    @staticmethod
    def fromDict(dict_params) -> 'TopicParams':
        """Generate TopicParams from dictionary."""
        namespaces = dict_params['namespaces']
        depth = dict_params['depth']
        durability = dict_params['durability']
        filtersize = dict_params['filtersize']
        history = dict_params['history']
        reliability = dict_params['reliability']
        value = dict_params['value']
        params = TopicParams(
            namespaces, depth, durability, filtersize, history, reliability, value
        )
        return params


@dataclass
class AdapterParams:
    """Dataclass for RvizAdapter parameters."""

    adapter_name: str
    in_topic: str
    out_topic: str
    sub_cfg: TopicParams
    pub_cfg: TopicParams

    def to_yml(self, drone_id: str):
        """Generate yml dict to be written in rviz config file."""
        outname = '/viz/' + drone_id + '/' + self.out_topic
        print(f'Creating yml for adapter with output {outname}')
        custom_yml = {}
        custom_yml['Enabled'] = True
        custom_yml['Name'] = 'Marker'
        custom_yml['Namespaces'] = {}
        for ns in self.pub_cfg.namespaces:
            custom_yml['Namespaces'][ns] = True
        custom_yml['Topic'] = {}
        custom_yml['Topic']['Depth'] = self.pub_cfg.depth
        custom_yml['Topic']['Durability Policy'] = self.pub_cfg.durability
        custom_yml['Topic']['Filter size'] = self.pub_cfg.filtersize
        custom_yml['Topic']['History Policy'] = self.pub_cfg.history
        custom_yml['Topic']['Reliability Policy'] = self.pub_cfg.reliability
        custom_yml['Topic']['Value'] = '/viz/' + drone_id + '/' + self.out_topic
        custom_yml['Value'] = True

        return custom_yml


@dataclass
class PresetAdapterParams(AdapterParams):
    """Dataclass for preset RvizAdapter parameters."""

    preset_type: str

    def to_yml(self, drone_id):
        """Generate yml dict to be written in rviz config file."""
        preset_yml = super().to_yml(drone_id)
        preset_yml['Class'] = 'rviz_default_plugins/Marker'

        return preset_yml


@dataclass
class CustomAdapterParams(AdapterParams):
    """Dataclass for custom RvizAdapter parameters."""

    adapter: Callable
    in_msg_type_name: str
    out_msg_type_name: str

    def to_yml(self, drone_id: str):
        """Generate yml dict to be written in rviz config file."""
        custom_yml = super().to_yml(drone_id)
        if self.out_msg_type_name == 'visualization_msgs/Marker':
            custom_yml['Class'] = 'rviz_default_plugins/Marker'
        elif self.out_msg_type_name == 'visualization_msgs/MarkerArray':
            custom_yml['Class'] = 'rviz_default_plugins/MarkerArray'

        return custom_yml
