#!/usr/bin/env python3
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
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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

# @file replay_mcap.py
#
# Replay a bag through simple_ekf_core, without ROS, and plot what the filter estimates.
#
# @authors Rodrigo Da Silva Gómez
"""
Replay a bag through simple_ekf_core, without ROS, and plot what the filter estimates.

The bag is only read with rosbags. The filter runs through the simple_ekf_core bindings, fed
as the simple_ekf plugin feeds it: the IMU predicts, the mocap corrects, the platform info says
when the drone is offboard, and on_tick runs at timer_hz. Each message is handled at the time
it was recorded, which is the clock the filter sees.
"""

import argparse
import logging
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import simple_ekf_core as ekf
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path,
                        help='rosbag2 directory, with its metadata.yaml and .mcap file')
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).with_name('replay_mcap.yaml'))
    parser.add_argument('--output', type=Path, default=Path('simple_ekf_replay.png'))
    parser.add_argument('--show', action='store_true', help='also open the plot in a window')
    return parser.parse_args()


def load_config(path):
    """
    Read the YAML file, and build the filter's Config and the mocap's SourceConfig from it.

    Every key under `filter` and `mocap.source` is set on the field of the same name, so a
    misspelt key fails instead of being ignored.
    """
    with open(path) as file:
        settings = yaml.safe_load(file)

    config = ekf.Config()
    for name, value in settings['filter'].items():
        if name == 'preflight_pose':
            value = ekf.Transform.from_rpy(value['position'], *value['rpy'])
        setattr(config, name, value)

    source_config = ekf.SourceConfig()
    for name, value in settings['mocap']['source'].items():
        setattr(source_config, name, value)

    return settings, config, source_config


def read_messages(bag, topics):
    """Yield (topic, time it was recorded in nanoseconds, message), in the order recorded."""
    # The bag carries the definitions of the message types this typestore lacks
    # (mocap4r2_msgs, as2_msgs), and AnyReader adds them from it
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        missing = set(topics) - {c.topic for c in connections}
        if missing:
            raise SystemExit(f'error: {bag} has no topic {", ".join(sorted(missing))}')
        for connection, time, data in reader.messages(connections=connections):
            yield connection.topic, time, reader.deserialize(data, connection.msgtype)


def to_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def to_transform(pose):
    orientation = pose.orientation
    return ekf.Transform(xyz(pose.position),
                         [orientation.x, orientation.y, orientation.z, orientation.w])


def find_body_pose(rigid_bodies, name):
    """Return the pose of the rigid body with this name, or None if the message lacks it."""
    for body in rigid_bodies.rigidbodies:
        if body.rigid_body_name == name:
            return body.pose
    return None


def is_unseen(pose):
    """Whether the pose is all zeros, which the mocap publishes while it does not see the body."""
    p, q = pose.position, pose.orientation
    return p.x == p.y == p.z == q.x == q.y == q.z == 0.0


def replay(bag, settings, config, source_config):
    """
    Run the filter over the bag, as the plugin would have, and record what it estimates.

    Returns three lists of (time, value): the filter's outputs every time the plugin would
    publish them, the body's mocap poses, and the drone's offboard status.
    """
    estimator = ekf.Filter(config)
    mocap_source = estimator.add_source(source_config)
    mocap_covariance = ekf.generate_covariance_from_config(source_config)

    imu_topic = settings['imu']['topic']
    platform = settings['platform_info']
    mocap = settings['mocap']
    topics = [imu_topic, mocap['topic']]
    if platform['topic']:
        topics.append(platform['topic'])
    else:
        estimator.set_offboard(True)

    tick_period = int(1e9 / settings['timer_hz'])
    next_tick = None
    estimates, measurements, offboard = [], [], []

    for topic, now, msg in read_messages(bag, topics):
        if next_tick is None:
            next_tick = now
        while next_tick <= now:
            # True when it applied the pre-flight correction, the only time it changes the state
            if estimator.on_tick(next_tick):
                estimates.append((next_tick, estimator.outputs))
            next_tick += tick_period

        if topic == imu_topic:
            if not mocap['set_earth_map']:
                # Nothing will set earth->map, so it stays the identity and the estimation
                # starts now
                estimator.mark_earth_to_map_set()
            if estimator.is_earth_to_map_set():
                estimator.on_imu(ekf.ImuSample(to_nanoseconds(msg.header.stamp),
                                               xyz(msg.linear_acceleration),
                                               xyz(msg.angular_velocity)))
                estimates.append((now, estimator.outputs))

        elif topic == platform['topic']:
            is_offboard = msg.armed if platform['use_arm'] else msg.offboard
            estimator.set_offboard(is_offboard)
            offboard.append((now, is_offboard))

        else:
            # The checks run in the plugin's order: the throttle and the repeated position
            # checks remember what they let through
            pose = find_body_pose(msg, mocap['rigid_body_name'])
            stamp = to_nanoseconds(msg.header.stamp)
            if (pose is None or estimator.should_throttle_update(mocap_source, stamp) or
                    is_unseen(pose)):
                continue
            measurements.append((now, to_transform(pose)))
            if estimator.is_repeated_position(mocap_source, xyz(pose.position), now):
                continue

            sample = ekf.PoseSample(stamp, ekf.SourceFrame.EARTH, to_transform(pose),
                                    mocap_covariance)
            if estimator.is_earth_to_map_set():
                estimator.on_pose(mocap_source, sample, now)
                estimates.append((now, estimator.outputs))
            elif mocap['set_earth_map'] and estimator.set_earth_to_map_from_first_pose(
                    sample.pose, ekf.SourceFrame.EARTH):
                estimator.mark_earth_to_map_set()

    return estimates, measurements, offboard


def without_wraps(degrees):
    """Blank the samples where an angle wraps around, so that -180 and 180 are not joined."""
    degrees[1:][np.abs(np.diff(degrees, axis=0)) > 180.0] = np.nan
    return degrees


def plot(title, estimates, measurements, offboard):
    """Plot the filter's estimate over the mocap's poses."""
    start = min(estimates[0][0], measurements[0][0])

    def seconds(times):
        return (np.array(times) - start) * 1e-9

    ekf_time = seconds([time for time, _ in estimates])
    ekf_poses = [outputs.earth_to_base for _, outputs in estimates]
    ekf_position = np.array([pose.position for pose in ekf_poses])
    ekf_rpy = without_wraps(np.degrees([pose.rpy for pose in ekf_poses]))
    ekf_velocity = np.array([outputs.twist_in_base.linear for _, outputs in estimates])
    mocap_time = seconds([time for time, _ in measurements])
    mocap_position = np.array([pose.position for _, pose in measurements])
    mocap_rpy = without_wraps(np.degrees([pose.rpy for _, pose in measurements]))

    figure, axes = plt.subplots(2, 2, figsize=(15, 9))
    figure.suptitle(f'{title}. Lines: EKF, wide bands: mocap')
    top_view, position_axis, orientation_axis, velocity_axis = axes.flat

    top_view.plot(mocap_position[:, 0], mocap_position[:, 1], color='0.75', lw=4, label='mocap')
    top_view.plot(ekf_position[:, 0], ekf_position[:, 1], lw=1, label='EKF')
    top_view.set(title='Top view, earth frame', xlabel='x [m]', ylabel='y [m]', aspect='equal')

    for axis, ekf_values, mocap_values, names, unit in (
            (position_axis, ekf_position, mocap_position, 'xyz', 'm'),
            (orientation_axis, ekf_rpy, mocap_rpy, ('roll', 'pitch', 'yaw'), 'deg')):
        for i, name in enumerate(names):
            axis.plot(mocap_time, mocap_values[:, i], color=f'C{i}', lw=4, alpha=0.3)
            axis.plot(ekf_time, ekf_values[:, i], color=f'C{i}', lw=1, label=name)
        axis.set(ylabel=unit)
    position_axis.set_title('Position, earth frame')
    orientation_axis.set_title('Orientation, earth frame')

    for i, name in enumerate(('vx', 'vy', 'vz')):
        velocity_axis.plot(ekf_time, ekf_velocity[:, i], color=f'C{i}', lw=1, label=name)
    velocity_axis.set(title='EKF velocity, body frame', ylabel='m/s')

    # The pre-flight correction holds the state at the pre-flight pose until the drone is
    # first offboard
    if offboard:
        end = next((time for time, is_offboard in offboard if is_offboard), estimates[-1][0])
        for axis in (position_axis, orientation_axis, velocity_axis):
            axis.axvspan(0.0, seconds(end), color='0.9', label='pre-flight')
    for axis in (position_axis, orientation_axis, velocity_axis):
        axis.set_xlabel('time [s]')
    for axis in axes.flat:
        axis.grid(True)
        axis.legend(fontsize='small')
    figure.tight_layout()
    return figure


def main() -> None:
    args = parse_args()
    # The filter logs to the "simple_ekf_core" logger
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(name)s: %(message)s')

    settings, config, source_config = load_config(args.config)
    estimates, measurements, offboard = replay(args.bag, settings, config, source_config)
    if not measurements:
        raise SystemExit(f"error: no pose of rigid body '{settings['mocap']['rigid_body_name']}'"
                         f" on {settings['mocap']['topic']}")
    if not estimates:
        raise SystemExit('error: the filter never started')
    print(f'{len(estimates)} estimates, {len(measurements)} mocap poses')

    figure = plot(f'simple_ekf_core on {args.bag.name}', estimates, measurements, offboard)
    figure.savefig(args.output, dpi=150)
    print(f'Saved the plot to {args.output}')
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
