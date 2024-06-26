"""bridges.py."""

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

__authors__ = 'Pedro Arias Pérez, Javier Melero Deza, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

from as2_gazebo_assets.bridges.bridge import Bridge, BridgeDirection


def prefix(world_name, drone_model_name, sensor_model_name, sensor_model_type):
    """Gz prefix for topics."""
    prefix = f'/world/{world_name}/model/{drone_model_name}'
    prefix = prefix + f'/model/{sensor_model_name}/link/{sensor_model_type}/sensor'
    return prefix


def clock():
    """Clock bridge."""
    return Bridge(
        gz_topic='/clock',
        ros_topic='/clock',
        gz_type='ignition.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def imu(world_name, model_name, sensor_name, link_name, model_prefix=''):
    """Imu bridge."""
    sensor_prefix = prefix(world_name, model_name, sensor_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/imu/imu',
        ros_topic='sensor_measurements/imu',
        gz_type='ignition.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def magnetometer(world_name, model_name, sensor_name, link_name, model_prefix=''):
    """Magnetometer bridge."""
    sensor_prefix = prefix(world_name, model_name, sensor_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/magnetometer/magnetometer',
        ros_topic='sensor_measurements/magnetic_field',
        gz_type='ignition.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def air_pressure(world_name, model_name, sensor_name, link_name, model_prefix=''):
    """Air pressure bridge."""
    sensor_prefix = prefix(world_name, model_name, sensor_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='sensor_measurements/air_pressure',
        gz_type='ignition.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.GZ_TO_ROS,
    )


# NOT USED, USE CUSTOM BRIDGE INSTEAD: ODOM --> GROUND_TRUTH
def odom(model_name):
    """Odom bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/odometry',
        ros_topic='sensor_measurements/odom',
        gz_type='ignition.msgs.Odometry',
        ros_type='nav_msgs/msg/Odometry',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def pose(model_name):
    """Pose bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic=f'{model_name}/pose',
        gz_type='ignition.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def tf_pose(model_name):
    """Tf pose bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic='/tf',
        gz_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def tf_pose_static(model_name):
    """Tf pose static bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/pose_static',
        ros_topic='/tf',  # TODO, tf_static not working
        gz_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def cmd_vel(model_name):
    """Input command vel bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/cmd_vel',
        ros_topic=f'/gz/{model_name}/cmd_vel',
        gz_type='ignition.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_GZ,
    )


def cmd_actuators(model_name):
    """Input command actuators bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/command/motor_speed',
        ros_topic=f'/gz/{model_name}/cmd_actuators',
        gz_type='ignition.msgs.Actuators',
        ros_type='actuator_msgs/msg/Actuators',
        direction=BridgeDirection.ROS_TO_GZ,
    )


def joint_cmd_vel(model_name, joint_name):
    """Input joint command vel bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/joint/{joint_name}/cmd_vel',
        ros_topic=f'/gz/{model_name}/joint/{joint_name}/cmd_vel',
        gz_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ,
    )


def arm(model_name):
    """Arming bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/velocity_controller/enable',
        ros_topic=f'/gz/{model_name}/arm',
        gz_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.ROS_TO_GZ,
    )


def battery(model_name):
    """Battery bridge."""
    return Bridge(
        gz_topic=f'/model/{model_name}/battery/linear_battery/state',
        ros_topic='sensor_measurements/battery',
        gz_type='ignition.msgs.BatteryState',
        ros_type='sensor_msgs/msg/BatteryState',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def image(world_name, drone_model_name, sensor_model_name,
          sensor_model_type, sensor_model_prefix=''):
    """Image bridge."""
    sensor_prefix = prefix(world_name, drone_model_name, sensor_model_name, sensor_model_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/camera/image',
        ros_topic=f'sensor_measurements/{sensor_model_prefix}/image_raw',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def depth_image(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    """Depth image bridge."""
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/camera/depth_image',
        ros_topic=f'sensor_measurements/{model_prefix}/depth',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def camera_info(world_name, drone_model_name, sensor_model_name,
                sensor_model_type, sensor_model_prefix=''):
    """Camera info bridge."""
    sensor_prefix = prefix(world_name, drone_model_name, sensor_model_name, sensor_model_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/camera/camera_info',
        ros_topic=f'sensor_measurements/{sensor_model_prefix}/camera_info',
        gz_type='ignition.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def lidar_scan(world_name, drone_model_name, sensor_model_name,
               sensor_model_type, sensor_model_prefix=''):
    """Lidar scan bridge."""
    sensor_prefix = prefix(world_name, drone_model_name, sensor_model_name, sensor_model_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/gpu_ray/scan',
        ros_topic=f'sensor_measurements/{sensor_model_prefix}/scan',
        gz_type='ignition.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def lidar_points(world_name, drone_model_name, sensor_model_name,
                 sensor_model_type, sensor_model_prefix=''):
    """Lidar point bridge."""
    sensor_prefix = prefix(world_name, drone_model_name, sensor_model_name, sensor_model_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/gpu_ray/scan/points',
        ros_topic=f'sensor_measurements/{sensor_model_prefix}/points',
        gz_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def camera_points(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    """Camera points bridge."""
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/camera/points',
        ros_topic=f'sensor_measurements/{model_prefix}/points',
        gz_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS,
    )


# NOT USED; BRIDGE NOT SUPPORTED IN FORTRESS
def navsat(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    """Navsat bridge."""
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/navsat/navsat',
        ros_topic=f'sensor_measurements/{model_prefix}/gps',
        gz_type='ignition.msgs.NavSat',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def gripper_suction_contacts(model_name):
    """Gripper suction contact bridge."""
    _prefix = 'gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{_prefix}/contact',
        ros_topic=f'sensor_measurements/{_prefix}/contact',
        gz_type='ignition.msgs.Contacts',
        ros_type='ros_gz_interfaces/msg/Contacts',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def gripper_contact(model_name, direction):
    """Gripper contact bridge."""
    _prefix = 'gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{_prefix}/contacts/{direction}',
        ros_topic=f'sensor_measurements/{_prefix}/contacts/{direction}',
        gz_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.GZ_TO_ROS,
    )


def gripper_suction_control(model_name):
    """Gripper suction control bridge."""
    _prefix = 'gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{_prefix}/suction_on',
        ros_topic=f'sensor_measurements/{_prefix}/suction_on',
        gz_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.ROS_TO_GZ,
    )
