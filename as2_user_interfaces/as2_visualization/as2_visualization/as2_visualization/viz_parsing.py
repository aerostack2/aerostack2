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

import importlib.util
import json
import sys
from typing import Callable

from as2_visualization.drone_viz import VizInfo
from as2_visualization.viz_params import (
    AdapterParams,
    CustomAdapterParams,
    PresetAdapterParams,
    TopicParams,
)


class JSONParser:
    """Parser for the JSON configuration file."""

    def __init__(self, file: str) -> None:
        info: dict
        self.drones: dict[str, dict[str, list]] = {}

        with open(file) as f:
            info = json.load(f)

        adapters: dict[str, AdapterParams] = {}
        # Using set to avoid duplicates and faster search
        custom_adapters: set[str] = set()
        preset_adapters: set[str] = set()

        self.adapters_per_process = info['adapters_per_process']

        if 'preset' in info['adapters']:
            for adapter_info in info['adapters']['preset']:
                padapter_params: PresetAdapterParams = self._parsePresetAdapter(adapter_info)
                name: str = padapter_params.adapter_name
                if name in preset_adapters or name in preset_adapters:
                    print(f'WARNING : Adapter {name} being overriden')

                adapters[name] = padapter_params

                preset_adapters.add(name)

        if 'custom' in info['adapters']:
            for adapter_info in info['adapters']['custom']:
                cadapter_params: CustomAdapterParams = self._parseCustomAdapter(adapter_info)
                name: str = cadapter_params.adapter_name
                if name in adapters or name in custom_adapters:
                    print(f'WARNING : Adapter {name} being overriden')

                adapters[name] = cadapter_params
                custom_adapters.add(name)

        for dname in info['drones']:
            drone_custom = []
            drone_preset = []
            for ad in info['drones'][dname]:
                if ad in custom_adapters:
                    drone_custom.append(adapters[ad])
                elif ad in preset_adapters:
                    drone_preset.append(adapters[ad])
                else:
                    print(f'WARNING: Adapter {ad} not defined')
            if dname not in self.drones:
                self.drones[dname] = {}
                self.drones[dname]['custom'] = []
                self.drones[dname]['preset'] = []
            else:
                print(f'WARNING: Drone {dname} being overriden')
            self.drones[dname]['custom'] = drone_custom
            self.drones[dname]['preset'] = drone_preset

    def insertToVizInfo(self, vizinfo: VizInfo) -> VizInfo:
        for d in self.drones:
            drone_adapters = self.drones[d]
            for ca in drone_adapters['custom']:
                vizinfo.add_custom_adapter(d, ca)
            for pa in drone_adapters['preset']:
                vizinfo.add_preset_adapter(d, pa)

        return vizinfo

    def _parsePresetAdapter(self, adapter_info: dict) -> PresetAdapterParams:
        if 'sub_cfg' in adapter_info:
            sub_params: dict = adapter_info['sub_cfg']
            sub_cfg: TopicParams = TopicParams.fromDict(sub_params)
        else:
            sub_cfg: TopicParams = TopicParams()
        if 'pub_cfg' in adapter_info:
            pub_params: dict = adapter_info['pub_cfg']
            pub_cfg: TopicParams = TopicParams.fromDict(pub_params)
        else:
            pub_cfg: TopicParams = TopicParams()

        sub_topic: str = adapter_info['in_topic']
        pub_topic: str = adapter_info['out_topic']
        adapter_name = adapter_info['id']
        preset_type: str = adapter_info['preset_type']
        adapter_params: PresetAdapterParams = PresetAdapterParams(
            adapter_name, sub_topic, pub_topic, sub_cfg, pub_cfg, preset_type
        )
        return adapter_params

    def _parseCustomAdapter(self, adapter_info: dict) -> CustomAdapterParams:
        if 'sub_cfg' in adapter_info:
            sub_params: dict = adapter_info['sub_cfg']
            sub_cfg: TopicParams = TopicParams.fromDict(sub_params)
        else:
            sub_cfg: TopicParams = TopicParams()
        if 'pub_cfg' in adapter_info:
            pub_params: dict = adapter_info['pub_cfg']
            pub_cfg: TopicParams = TopicParams.fromDict(pub_params)
        else:
            pub_cfg: TopicParams = TopicParams()
        sub_topic: str = adapter_info['in_topic']
        pub_topic: str = adapter_info['out_topic']
        adapter_name = adapter_info['id']
        sub_msg_type_name: str = adapter_info['in_msg']
        pub_msg_type_name: str = adapter_info['out_msg']
        adapter: Callable = self._parseCallable(adapter_info['adapter'])
        adapter_params: CustomAdapterParams = CustomAdapterParams(
            adapter_name,
            sub_topic,
            pub_topic,
            sub_cfg,
            pub_cfg,
            adapter,
            sub_msg_type_name,
            pub_msg_type_name,
        )
        return adapter_params

    def _parseCallable(self, callable_info: dict) -> Callable:
        module_path: str = callable_info['path']
        module_name: str = callable_info['name']
        callable_name: str = callable_info['func_name']

        # idk how safe / unsafe this may be: check
        spec = importlib.util.spec_from_file_location(module_name, module_path)
        if spec is None:
            raise ModuleNotFoundError(f'Module {module_name} not found in {module_path}')
        module = importlib.util.module_from_spec(spec)  # get module from spec
        sys.modules[f'{module_name}'] = module  # adding manually to loaded modules
        spec.loader.exec_module(module)  # type: ignore

        # get callable from module
        return getattr(module, callable_name)
