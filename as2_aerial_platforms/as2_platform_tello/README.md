# tello_platform
AS2 DJI Tello platform interface.

This repository includes the necessary functions to communicate with Tello DJI drone and create a link with aerostack2 framework. This package has been developed in C++ and ROS2.
## tello_connection
Inside this part, there are the main C++ classes to make the connection with the robot platform. Check the tello.hpp and socketudp.hpp within include folder. It can be tested separately with clientudp.cpp. In this case, it is possible to compile it with the /tello_connection/CMakeLists.txt
Create a folder and move it inside:

		mkdir build && cd build

Create compilation files:

		cmake ..

Make the compilation:

		make

But in the test tello_connection clientudp.cpp, currently, there are not too many tests, however any test can be tested there and done separately from the framework. 
## tello_platform 
It is the main section. Check the include/tello_platform.hpp library where the main class with its methods and attributes are stated. By other hand, in src/tello_platform.cpp they are coded. 
