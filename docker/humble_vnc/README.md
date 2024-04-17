# ROS2 Humble + Aerostack2 latest Docker Image

## Run
```
./start.sh
```

First time it will build image (inside image will be installed aerostack2). 

It will take long time (big download and long dependency installation). 
Fully builded image take about 8.3Gb. 

Docker container will be automatically runned after build image. 
When running you can type in your browser (on host computer) http://0.0.0.0:6080/ URL and see desktop of your Ubuntu 22.04 with ROS Humble, Aerostack2 and Ignition Gazebo.

Also it binds your host aerostack2 directory to container /home/ubuntu/ros2/aerostack2_ws directory (bidirectionally). 
