# PX4-ROS2 bridge

[![GitHub license](https://img.shields.io/github/license/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/blob/master/LICENSE) [![GitHub (pre-)release](https://img.shields.io/github/release-pre/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/releases/tag/beta) [![DOI](https://zenodo.org/badge/142936318.svg)](https://zenodo.org/badge/latestdoi/142936318) [![Build and Test package](https://github.com/PX4/px4_ros_com/workflows/Build%20and%20Test%20package/badge.svg?branch=master)](https://github.com/PX4/px4_ros_com/actions)

This package materializes the ROS2 side of PX4-FastRTPS/DDS bridge, establishing a bridge between the PX4 autopilot stack through a micro-RTPS bridge, Fast-RTPS(DDS) and ROS2. It has a straight dependency on the [`px4_msgs`](https://github.com/PX4/px4_msgs) package, as it depends on the IDL files, to generate the micro-RTPS bridge agent, and on the ROS interfaces and typesupport, to allow building and running the example nodes.

The [`master`](https://github.com/PX4/px4_ros_com/tree/master) branch of this package composes the ROS2 package and the ROS2 side (agent) of the bridge. The [`ros1`](https://github.com/PX4/px4_ros_com/tree/ros1) branch is a product of the `master` and represents the ROS(1) package and the ROS(1) side of the bridge, for wich it is required using the [`ros1_bridge`](https://github.com/ros2/ros1_bridge).

## Install, build and usage

Check the [RTPS/ROS2 Interface](https://dev.px4.io/en/middleware/micrortps.html) section on the PX4 Devguide for details on how to install the required dependencies, build the package (composed by the two branches) and use it.

## Bug tracking and feature requests

Use the [Issues](https://github.com/PX4/px4_ros_com/issues) section to create a new issue. Report your issue or feature request [here](https://github.com/PX4/px4_ros_com/issues/new).

## Questions and troubleshooting

Reach the PX4 development team on the `#messaging` or `#ros` PX4 Slack channels:
[![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)
