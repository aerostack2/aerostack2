# Copyright 2025 Universidad Politécnica de Madrid
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

import importlib
import inspect

import as2_visualization.rviz_adapter as ra
from as2_visualization.rviz_adapter import RvizAdapter
from as2_visualization.viz_params import CustomAdapterParams, PresetAdapterParams, TopicParams
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class AdapterBuilder:
    def build_preset(
        self,
        name: str,
        preset_type: str,
        in_topic: str,
        out_topic: str,
        sub_cfg: TopicParams,
        pub_cfg: TopicParams,
    ) -> RvizAdapter:  # type: ignore
        # Generate docstring
        """
        Build a preset adapter.

        :param name: Name of the adapter
        :param preset_type: Type of the adapter
        :param in_topic: Input topic
        :param out_topic: Output topic
        :param sub_cfg: Subscription configuration
        :param pub_cfg: Publication configuration
        :return: RvizAdapter object
        """
        sub_qos = self.generateQos(sub_cfg)
        pub_qos = self.generateQos(pub_cfg)
        members = inspect.getmembers(
            ra, lambda x: inspect.isclass(x) and (issubclass(x, RvizAdapter))
        )
        for name, obj in members:
            try:
                if obj.__name__ == preset_type:
                    return obj(name, in_topic, out_topic, sub_qos, pub_qos)
            except Exception:
                raise ValueError(f'Preset type {preset_type} not found')

    def generateQos(self, cfg: TopicParams) -> QoSProfile:
        qos: QoSProfile
        try:
            history = QoSHistoryPolicy[cfg.history.upper()]
            durability = QoSDurabilityPolicy[cfg.durability.upper()]
            depth = cfg.depth
            reliability = QoSReliabilityPolicy[cfg.reliability.upper()]
        except ValueError:
            raise ValueError(f'Wrong topic qos settings {cfg}')

        qos: QoSProfile = QoSProfile(
            history=history, durability=durability, depth=depth, reliability=reliability
        )

        return qos

    def build_custom(
        self,
        name: str,
        adapter_func,
        in_topic: str,
        in_msg_type: str,
        out_topic: str,
        out_msg_type: str,
        sub_cfg: TopicParams,
        pub_cfg: TopicParams,
    ) -> RvizAdapter:
        """MSG type is to be specified with <package_name>/<msg_name>."""
        sub_qos: QoSProfile = self.generateQos(sub_cfg)
        pub_qos: QoSProfile = self.generateQos(pub_cfg)
        in_msg = None
        out_msg = None
        # Exctact module name from in_msg_type
        package_name = in_msg_type.split('/')[0] + '.msg'
        module_name = in_msg_type.split('/')[1]
        # Import module
        mod_obj = importlib.import_module(package_name)
        # Get message type
        try:
            in_msg = getattr(mod_obj, module_name)
        except AttributeError:
            raise ValueError(f'Message type {in_msg_type} not found')

        # Since only Marker or MarkerArray can be output types, check it directly
        if out_msg_type == 'visualization_msgs/Marker':
            out_msg = Marker
        elif out_msg_type == 'visualization_msgs/MarkerArray':
            out_msg = MarkerArray
        else:
            raise ValueError(
                'Output message type must be either visualization_msgs/Marker or '
                'visualization_msgs/MarkerArray'
            )
        return RvizAdapter(
            name, adapter_func, in_topic, out_topic, in_msg, out_msg, sub_qos, pub_qos
        )


class VizInfo:
    """Class to represent RViz user configurations."""

    def __init__(self):
        self.custom_adapters: list[tuple[str, CustomAdapterParams]] = []
        self.preset_adapters: list[tuple[str, PresetAdapterParams]] = []

    def add_preset_adapter(self, drone_id: str, presetParams: PresetAdapterParams):
        self.preset_adapters.append((drone_id, presetParams))

    def add_custom_adapter(
        self,
        drone_id: str,
        customParams: CustomAdapterParams,
    ):
        self.custom_adapters.append((drone_id, customParams))

    def generate_adapters(self, builder: AdapterBuilder) -> list[RvizAdapter]:
        """Build list of RvizAdapters using stored parameters."""
        adapters: list[RvizAdapter] = []
        for pa in self.preset_adapters:
            preset: PresetAdapterParams = pa[1]
            in_topic, out_topic = self.generate_topic_names(pa)
            adapters.append(
                builder.build_preset(
                    preset.adapter_name,
                    preset.preset_type,
                    in_topic,
                    out_topic,
                    preset.sub_cfg,
                    preset.pub_cfg,
                )
            )
        for cu in self.custom_adapters:
            in_topic, out_topic = self.generate_topic_names(cu)
            custom: CustomAdapterParams = cu[1]
            adapters.append(
                builder.build_custom(
                    custom.adapter_name,
                    custom.adapter,
                    in_topic,
                    custom.in_msg_type_name,
                    out_topic,
                    custom.out_msg_type_name,
                    custom.sub_cfg,
                    custom.pub_cfg,
                )
            )

        return adapters

    def generate_topic_names(self, adap_params):
        if adap_params[0] == 'abs':
            in_topic: str = f'/{adap_params[1].in_topic}'
            out_topic: str = f'/viz/abs/{adap_params[1].out_topic}'
        else:
            in_topic: str = f'/{adap_params[0]}/{adap_params[1].in_topic}'
            out_topic: str = f'/viz/{adap_params[0]}/{adap_params[1].out_topic}'
        return in_topic, out_topic

    def to_yml(self):
        """Generate list of yml dicts representation of stored adapters."""
        yml_list = []
        for custom in self.custom_adapters:
            yml_list.append(custom[1].to_yml(custom[0]))
        for preset in self.preset_adapters:
            yml_list.append(preset[1].to_yml(preset[0]))
        return yml_list
