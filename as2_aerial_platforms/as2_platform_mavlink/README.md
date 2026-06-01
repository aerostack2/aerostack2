# as2_platform_mavlink

[Aerostack2](https://aerostack2.github.io/) Aerial platform for autopilots that uses MAVLINK as communication standard

## Setup guides

From [MAVROS ROS INDEX](https://index.ros.org/p/mavros/).

Setup with ```rosdep``` or 
```
sudo apt install ros-<distro>-mavros ros-<distro>-mavros-extras -y 
```

After this is required to install geographic_lib_datasets

```
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```


