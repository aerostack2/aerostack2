from ign_assets.bridge import Bridge, BridgeDirection


def prefix(world_name, model_name, model_sensor_name, link_name='sensor_link'):
    return f'/world/{world_name}/model/{model_name}/model/{model_sensor_name}/link/{link_name}/sensor'


def clock():
    return Bridge(
        ign_topic='/clock',
        ros_topic='/clock',
        ign_type='ignition.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.IGN_TO_ROS)


def imu(world_name, model_name, sensor_name, link_name, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/imu/imu',
        ros_topic='sensor_measurements/imu',
        ign_type='ignition.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.IGN_TO_ROS)


def magnetometer(world_name, model_name, sensor_name, link_name, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/magnetometer/magnetometer',
        ros_topic='sensor_measurements/magnetic_field',
        ign_type='ignition.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.IGN_TO_ROS)


def air_pressure(world_name, model_name, sensor_name, link_name, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='sensor_measurements/air_pressure',
        ign_type='ignition.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.IGN_TO_ROS)


# NOT USED, USE CUSTOM BRIDGE INSTEAD: ODOM --> GROUND_TRUTH
def odom(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/odometry',
        ros_topic='sensor_measurements/odom',
        ign_type='ignition.msgs.Odometry',
        ros_type='nav_msgs/msg/Odometry',
        direction=BridgeDirection.IGN_TO_ROS)


def pose(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/pose',
        # ros_topic='pose',
        ros_topic='/tf',
        ign_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.IGN_TO_ROS)


def pose_static(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/pose_static',
        # ros_topic='pose_static',
        ros_topic='/tf',
        ign_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.IGN_TO_ROS)


def cmd_vel(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/cmd_vel',
        ros_topic=f'/ign/{model_name}/cmd_vel',
        ign_type='ignition.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_IGN)


def arm(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/velocity_controller/enable',
        ros_topic=f'/ign/{model_name}/arm',
        ign_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.ROS_TO_IGN)


def battery(model_name):
    return Bridge(
        ign_topic=f"/model/{model_name}/battery/linear_battery/state",
        ros_topic="sensor_measurements/battery",
        ign_type="ignition.msgs.BatteryState",
        ros_type="sensor_msgs/msg/BatteryState",
        direction=BridgeDirection.IGN_TO_ROS)


def image(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        ign_topic=f'{sensor_prefix}/camera/image',
        ros_topic=f'sensor_measurements/{model_prefix}/image_raw',
        ign_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.IGN_TO_ROS)


def depth_image(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        ign_topic=f'{sensor_prefix}/camera/depth_image',
        ros_topic=f'sensor_measurements/{model_prefix}/depth',
        ign_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.IGN_TO_ROS)


def camera_info(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        ign_topic=f'{sensor_prefix}/camera/camera_info',
        ros_topic=f'sensor_measurements/{model_prefix}/camera_info',
        ign_type='ignition.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.IGN_TO_ROS)


def lidar_scan(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        ign_topic=f'{sensor_prefix}/gpu_ray/scan',
        ros_topic=f'sensor_measurements/{model_prefix}/scan',
        ign_type='ignition.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.IGN_TO_ROS)


def lidar_points(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        ign_topic=f'{sensor_prefix}/gpu_ray/scan/points',
        ros_topic=f'sensor_measurements/{model_prefix}/points',
        ign_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.IGN_TO_ROS)


def camera_points(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        ign_topic=f'{sensor_prefix}/camera/points',
        ros_topic=f'sensor_measurements/{model_prefix}/points',
        ign_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.IGN_TO_ROS)


# NOT USED; BRIDGE NOT SUPPORTED IN FORTRESS
def navsat(world_name, model_name, sensor_name, sensor_type, model_prefix=''):
    sensor_prefix = prefix(world_name, model_name, sensor_name, sensor_type)
    return Bridge(
        ign_topic=f'{sensor_prefix}/navsat/navsat',
        ros_topic=f'sensor_measurements/{model_prefix}/gps',
        ign_type='ignition.msgs.NavSat',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.IGN_TO_ROS)


def gripper_suction_contacts(model_name):
    prefix = 'gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/contact',
        ros_topic='sensor_measurements/{prefix}/contact',
        ign_type='ignition.msgs.Contacts',
        ros_type='ros_gz_interfaces/msg/Contacts',
        direction=BridgeDirection.IGN_TO_ROS
    )


def gripper_contact(model_name, direction):
    prefix = 'gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/contacts/{direction}',
        ros_topic=f'sensor_measurements/{prefix}/contacts/{direction}',
        ign_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.IGN_TO_ROS
    )


def gripper_suction_control(model_name):
    prefix = 'gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/suction_on',
        ros_topic=f'sensor_measurements/{prefix}/suction_on',
        ign_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.ROS_TO_IGN
    )
