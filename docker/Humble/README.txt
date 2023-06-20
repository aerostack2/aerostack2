#Run 
sh start.sh
#First time it will build image (inside image will be instaled aerostack2). 
#It will take long time (big download and long dependency installation). 
#Fully builded image take about 7gb. 
#Docker container will be automatically runned after build image. 
#When it will be runned and healthy you can write in your browser (on host computer) http://0.0.0.0:6080/ URL
#and see desktop of your Ubuntu 22.04 with ROS Humble with Aerostack2 with Ignition Gazebo and etc.

#Also it binds your host aerostack2 directory to container /home/ubuntu/ros2/aerostack2_ws dicrectory (bidirectionally).