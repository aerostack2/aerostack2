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

import argparse

from as2_visualization.drone_viz import AdapterBuilder, VizInfo
from as2_visualization.rviz_adapter import VizBridge, RvizAdapter
from as2_visualization.viz_parsing import JSONParser

from functools import partial

from multiprocessing import Pool

import os

import psutil

import rclpy
from rclpy.executors import MultiThreadedExecutor

import signal

import rclpy.logging


def options():

    parser = argparse.ArgumentParser(description='Launch rviz adapters from json config file')
    parser.add_argument('file', type=str, help='Path to json config file')
    opt = parser.parse_args()
    return vars(opt)


def run_bridge(adapters: list[list[RvizAdapter]], idx):
    rclpy.init()

    bridge: VizBridge = VizBridge(f'bridge{idx}')
    for ad in adapters[idx]:
        bridge.register_adapter(ad)

    rclpy.logging.get_logger(f"Bridge {idx}").info("Node created")

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    executor.spin()

    rclpy.shutdown()


def sigkill_handler(sig, frame):
    pid = os.getpid()
    act_process = psutil.Process(pid)
    for child in act_process.children(recursive=True):
        child.kill()

    act_process.kill()


def main():
    # signal.signal(signal.SIGINT, sigkill_handler)

    args = options()
    cfg_file: str = args['file']

    parser: JSONParser = JSONParser(cfg_file)
    vinfo: VizInfo = VizInfo()
    vinfo = parser.insertToVizInfo(vinfo)
    builder = AdapterBuilder()
    adapter_list: list[RvizAdapter] = vinfo.generate_adapters(builder)
    adapters_per_bridge = parser.adapters_per_process
    num_bridges: int = round(len(adapter_list) / adapters_per_bridge)
    bridge_list = [[] for i in range(num_bridges)]
    b_idx: int = 0
    for ad in adapter_list:
        b = bridge_list[b_idx]
        b.append(ad)
        b_idx = (b_idx + 1) % num_bridges

    rclpy.init()
    executor = MultiThreadedExecutor()
    for i, b in enumerate(bridge_list):
        vb = VizBridge(f"bridge_{i}")
        for ad in b:
            vb.register_adapter(ad)
        executor.add_node(vb)

    executor.spin()

    rclpy.shutdown()

    # worker_func = partial(run_bridge, bridge_list)

    # rclpy.logging.get_logger("Global viz").info("Bridge list created")

    # p = Pool()
    # p.map(worker_func, range(num_bridges))


if __name__ == '__main__':

    main()
