"""Mission interpreter ROS 2 adapter."""

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
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse

from as2_msgs.msg import MissionUpdate
from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
import rclpy
from rclpy.executors import Executor, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy
from std_msgs.msg import String


class Adapter(Node):
    """ROS 2 Adapter to mission interpreter."""

    def __init__(self, drone_id: str, timer_freq: float, use_sim_time: bool = False,
                 add_namespace: bool = False, executor: Executor = SingleThreadedExecutor):
        super().__init__('adapter', namespace=drone_id)

        self.param_use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.namespace = drone_id
        self.interpreter = MissionInterpreter(use_sim_time=use_sim_time, executor=executor)
        self.last_mid: int = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic_prefix = '' if add_namespace else '/'
        self.mission_update_sub = self.create_subscription(
            MissionUpdate, topic_prefix + 'mission_update', self.mission_update_callback,
            qos_profile_system_default)

        self.mission_status_pub = self.create_publisher(
            String, topic_prefix + 'mission_status', qos_profile)

        self.mission_state_timer = self.create_timer(
            1 / timer_freq, self.status_timer_callback)

        self.get_logger().info('Mission Interpreter Adapter ready')

    def status_timer_callback(self):
        """Publish new mission status."""
        msg = String()
        try:
            msg.data = self.interpreter.status.json()
        except TypeError as e:
            self.get_logger().warn(f'Failed to deserialize status: {e}')
        else:
            self.mission_status_pub.publish(msg)

    def mission_update_callback(self, msg: MissionUpdate):
        """Mission update callback."""
        if msg.drone_id != self.namespace:
            self.get_logger().info(
                f'Received mission update for {msg.drone_id} but I am {self.namespace}')
            return

        if msg.action == MissionUpdate.EXECUTE:
            self.execute_callback(msg.mission_id, Mission.parse_raw(msg.mission))
        elif msg.action == MissionUpdate.LOAD:
            mission = Mission.parse_raw(msg.mission)
            self.interpreter.load_mission(msg.mission_id, mission)
            # Send updated status
            self.status_timer_callback()
            self.get_logger().info(f'Mission: {msg.mission_id} loaded.')
            self.get_logger().info(f'Mission: {mission}')
            self.last_mid = msg.mission_id
        elif msg.action == MissionUpdate.START:
            self.start_callback(msg.mission_id)
        elif msg.action == MissionUpdate.PAUSE:
            self.interpreter.pause_mission(msg.mission_id)
        elif msg.action == MissionUpdate.RESUME:
            self.interpreter.resume_mission(msg.mission_id)
        elif msg.action == MissionUpdate.STOP:
            self.interpreter.stop_mission(msg.mission_id)
            self.interpreter.reset(msg.mission_id, self.interpreter._missions[msg.mission_id])
        elif msg.action == MissionUpdate.NEXT_ITEM:
            self.interpreter.next_item(msg.mission_id)
        else:
            self.get_logger().error(f'Unimplemented action: {msg.action}')

    def execute_callback(self, mid: int, mission: Mission):
        """Load and start mission."""
        self.interpreter.reset(mid, mission)
        self.start_callback(mid)

    def start_callback(self, mid: int):
        """Start mission on interpreter."""
        try:
            # TODO: where to arm and offboard? Avoid calling interpreter.drone property directly
            if not self.interpreter._drone.info['armed']:
                self.interpreter._drone.arm()
            if not self.interpreter._drone.info['offboard']:
                self.interpreter._drone.offboard()
            self.get_logger().info(f'Starting mission: {mid}')
            self.interpreter.start_mission(mid)
        except AttributeError:
            self.get_logger().error('Trying to start mission but no mission is loaded.')


def main():
    """Run node."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=str, default='drone0',
                        help='Namespace')
    parser.add_argument('--timer_freq', type=float, default=0.5, help='Status timer frequency')
    parser.add_argument('--use_sim_time', action='store_true', help='Use sim time')
    parser.add_argument(
        '--add_namespace', action='store_true', help='Add namespace to topics')
    parser.add_argument(
        '--use_multi_threaded_executor', action='store_true', help='Use MultiThreadedExecutor in'
        + ' Drone Interface')

    argument_parser = parser.parse_args()

    rclpy.init()

    if argument_parser.use_multi_threaded_executor:
        executor_class = MultiThreadedExecutor
    else:
        executor_class = SingleThreadedExecutor

    adapter = Adapter(
        drone_id=argument_parser.n, timer_freq=argument_parser.timer_freq,
        use_sim_time=argument_parser.use_sim_time, add_namespace=argument_parser.add_namespace,
        executor=executor_class)
    rclpy.spin(adapter)

    adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
