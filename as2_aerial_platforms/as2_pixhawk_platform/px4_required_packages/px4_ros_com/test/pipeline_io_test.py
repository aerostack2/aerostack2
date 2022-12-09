#!/usr/bin/env python3

################################################################################
#
#   Copyright 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
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
#
################################################################################

# This script launches the microRTPS bridge pipeline to verify if there is
# data being exchanged between the flight controller (in SITL) and companion
# computer side, with its DDS participants

import argparse
import os
from time import sleep
from sys import exit
from subprocess import call, CalledProcessError, check_output, DEVNULL, Popen, STDOUT

if __name__ == "__main__":

    default_px4_ros_com_install_dir = os.path.expanduser("~") + "/PX4/px4_ros2_ws/install"
    default_px4_dir = os.path.expanduser("~") + "/PX4/Firmware"
    default_px4_target = "iris_rtps"
    default_test_dir = os.path.dirname(os.path.realpath(__file__))
    default_test_type = "fcu_output"
    default_subscriber_topic = "sensor_combined"
    default_publisher_topic = "debug_vect"

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--test-dir", dest='test_dir', type=str,
                        help="Absolute path to test script", default=default_test_dir)
    parser.add_argument("-f", "--px4-firmware-dir", dest='px4_dir', type=str,
                        help="Absolute path to PX4 Firmware dir, defaults to $HOME/PX4/Firmware", default=default_px4_dir)
    parser.add_argument("-b", "--px4-build-target", dest='px4_target', type=str,
                        help="PX4 SITL target, defaults to iris_rtps", default=default_px4_target)
    parser.add_argument("-t", "--test-type", dest='test_type', type=str,
                        help="Test type [fcu_input, fcu_output], defaults to 'fcu_output'", default=default_test_type)
    parser.add_argument("-s", "--subscriber", dest='subscriber_topic', type=str,
                        help="ROS2 topic to test, defaults to 'sensor_combined'", default=default_subscriber_topic)
    parser.add_argument("-p", "--publisher", dest='publisher_topic', type=str,
                        help="ROS2 publisher data, meaning the data to be received on the flight controller side, defaults to 'debug_vect'", default=default_publisher_topic)

    # Parse arguments
    args = parser.parse_args()
    if os.path.isabs(args.px4_dir):
        px4_dir = args.px4_dir
    else:
        raise Exception("Please provide PX4 Firmware absolute path")
    px4_target = args.px4_target
    test_dir = args.test_dir
    test_type = args.test_type
    listener = args.subscriber_topic
    advertiser = args.publisher_topic

    # get ROS distro
    ros_distro = check_output("rosversion -d", shell=True)

    # get PX4 Firmware tag
    px4_tag = check_output(
        "cd " + px4_dir + " && git describe --abbrev=0 && cd " + os.getcwd(), shell=True)

    print(
        "\n\033[34m-------------- PX4 MICRORTPS COMMUNICATION TEST --------------\033[0m")
    print("\n-- Test configuration:\n")
    print("    > ROS 2 distro: \033[36m" +
          str(ros_distro.strip().decode("utf-8")).capitalize() + "\033[0m")
    print("    > PX4 Firmware: \033[36m" + px4_tag.decode() + "\033[0m")
    print("\033[5m-- Running " + ("Output test" if(test_type ==
                                                   "fcu_output") else "Input test") + "...\033[0m")

    # launch the microRTPS agent
    print("\n\033[93m-- Starting microRTPS bridge agent..." + "\033[0m\n")
    call("micrortps_agent -t UDP &", shell=True, stderr=STDOUT)

    # waits for the agent to load
    sleep(3)

    # launch PX4 SITL daemon, in headless mode
    print(
        "\n\033[93m-- Starting the PX4 SITL daemon and Gazebo (without GUI)...\033[0m\n")
    call("cd " + px4_dir + " && (NO_PXH=1 HEADLESS=1 make px4_sitl_rtps gazebo_" +
         px4_target + " &) && cd " + os.getcwd(), shell=True, stderr=DEVNULL)

    # waits for PX4 daemon and Gazebo to load
    sleep(20)

    # setup the PX4 SITL bin dir
    px4_bin_dir = os.path.join(px4_dir, "build/px4_sitl_rtps/bin")

    # launch the specified test
    topic = ""
    test_format = list()
    test_result = -1

    if(test_type == "fcu_output"):
        # Flight controller output tests
        if (listener == "sensor_combined"):
            node = "sensor_combined_listener"
            topic = "sensor_combined"
            test_format = ["output", "from"]
            test_result = call(
                "python3 " + test_dir + "/test_output.py -t " + topic.replace("_", " ").title().replace(" ", ""), stderr=STDOUT, shell=True,
                universal_newlines=True)

    elif(test_type == "fcu_input"):
        # Flight controller input tests
        if (advertiser == "debug_vect"):
            package_name = "px4_ros_com"
            node = "debug_vect_advertiser"
            topic = "debug_vect"
            test_format = ["input", "on"]
            test_result = call(
                "python3 " + test_dir + "/test_input.py -b " + px4_bin_dir + " -p " + package_name + " -n " + node + " -t " + topic, stderr=STDOUT, shell=True,
                universal_newlines=True)
        elif (advertiser == "onboard_computer_status"):
            # specific test for https://github.com/Auterion/system_monitor_ros
            package_name = "system_monitor_ros"
            node = "system_monitor_node"
            topic = "onboard_computer_status"
            test_format = ["input", "on"]
            test_result = call(
                "python3 " + test_dir + "/test_input.py -b " + px4_bin_dir + " -p " + package_name + " -n " + node + " -t " + topic, stderr=STDOUT, shell=True,
                universal_newlines=True)

    call("killall gzserver micrortps_agent px4 ros2",
         shell=True, stdout=DEVNULL, stderr=DEVNULL)

    print(
        "\033[34m------------------------ TEST RESULTS ------------------------\033[0m")
    if (test_result):
        print("\033[91m" + ("Output test" if(test_type == "fcu_output") else "Input test") + ": [FAILED]\tFlight controller " + test_format[0] + " test failed! Failed to get data " + test_format[1] + " the '" +
              topic + "' uORB topic\033[0m")
        print(
            "\033[34m" + "--------------------------------------------------------------" + "\033[0m\n")
        exit(1)
    else:
        print("\033[92m" + ("Output test" if(test_type == "fcu_output") else "Input test") + ": [SUCCESS]\tFlight controller " + test_format[0] + " test successfull! Successfully retrieved data " + test_format[1] + " the '" +
              topic + "' uORB topic\033[0m")
        print(
            "\033[34m--------------------------------------------------------------" + "\033[0m\n")
        exit(0)
