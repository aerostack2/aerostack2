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

# This script tests the input data on a specific topic using the "listener"
# app in PX4. If no output comes or there's a "never published" output
# out of the app, the test fails.

import argparse
import os
from sys import exit
from subprocess import call, check_output, DEVNULL
from time import sleep

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--px4-binary-dir", dest='px4_binary', type=str,
                        help="Directory where the PX4 SITL daemon binaries are stored", required=True)
    parser.add_argument("-n", "--node-name", dest='node_name', type=str,
                        help="Name of the node publishing the to the topic", required=True)
    parser.add_argument("-p", "--package-name", dest='package_name', type=str,
                        help="ROS 2 package name of the node", required=True)
    parser.add_argument("-t", "--topic-name", dest='topic_name', type=str,
                        help="Topic name to test the output to the autopilot", required=True)

    # Parse arguments
    args = parser.parse_args()

    test_cmd = "/bin/bash -c '" + \
        os.path.join(args.px4_binary,
                     "px4-listener") + " " + args.topic_name + "'"

    print("\n\033[93m-- " + args.topic_name +
          " uORB data test launched:\033[0m")
    # start the ROS 2 node
    call("ros2 run " + args.package_name + " " + args.node_name + " &", shell=True)

    sleep(3)

    # call the 'listener' command for the '<topic_name>' uORB topic
    output = check_output(test_cmd, shell=True, universal_newlines=True)

    call("killall " + args.node_name,
         shell=True, stdout=DEVNULL, stderr=DEVNULL)

    if output and "never published" not in output:
        print(
            "\n\033[42m-- Successfully obtained data on '" + args.topic_name + "' uORB topic. microRTPS bridge is up! Output:\033[0m")
        print("\033[97m" + output + "\033[0m")
        exit(0)
    else:
        print(
            "\n\033[41m-- Something went wrong. Please check if the microRTPS bridge is functioning correctly.\033[0m\n")
        exit(1)
