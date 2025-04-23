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

import argparse

from as2_visualization.drone_viz import VizInfo
from as2_visualization.viz_parsing import JSONParser
import yaml


def options():
    parser = argparse.ArgumentParser(description='Launch rviz adapters from json config file')
    parser.add_argument('base_config', type=str, help='Path to base rviz yml config file')
    parser.add_argument('adapter_config', type=str, help='Path to rviz adapters config file')
    parser.add_argument('dest_file', type=str, help='Path to destination config file')
    opt = parser.parse_args()
    return vars(opt)


def main():
    args = options()
    rviz_file: str = args['base_config']
    cfg_file: str = args['adapter_config']

    parser: JSONParser = JSONParser(cfg_file)
    vinfo: VizInfo = VizInfo()
    vinfo = parser.insertToVizInfo(vinfo)
    adapters_yml = vinfo.to_yml()

    with open(rviz_file) as f:
        rviz_yml = yaml.safe_load(f)
        marker_list = rviz_yml['Visualization Manager']['Displays']
        for ad in adapters_yml:
            marker_list.append(ad)

    with open(args['dest_file'], 'w') as f2:
        yaml.dump(rviz_yml, f2)


if __name__ == '__main__':
    main()
