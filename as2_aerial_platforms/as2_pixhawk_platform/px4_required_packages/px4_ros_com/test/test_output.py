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

# This script tests the output of 'ros2 topic echo /<TopicName>'
# to evaluate if there's data output coming out of it. If the log output file
# is empty, means that there's no output on the topic, so the test fails

import argparse
from os import remove
from sys import exit
from subprocess import call, TimeoutExpired

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic-name", dest='topic_name', type=str,
                        help="Topic name to test the output to the autopilot", required=True)

    # Parse arguments
    args = parser.parse_args()

    timeout = 3  # seconds

    try:
        print(
            "\n\033[93m" + "-- " + args.topic_name + "_PubSubTopic output test launched:\033[0m")
        call("ros2 topic echo /" + args.topic_name + "_PubSubTopic", timeout=timeout, stdout=open(
            "ros2_topic_echo_out", "w"), shell=True)
    except TimeoutExpired as e:
        output = open("ros2_topic_echo_out", "r").read()
        if output:
            print(
                "\n\033[42m" + "-- Successfully obtained data on " + args.topic_name + "_PubSubTopic topic. microRTPS bridge is up! Output:\033[0m\n\n")
            print("\033[97m" + output + "\033[0m")
            remove("ros2_topic_echo_out")
            exit(0)
        else:
            print(
                "\n\033[41m" + "-- Something went wrong. Please check if the microRTPS bridge is functioning correctly.\033[0m\n")
            remove("ros2_topic_echo_out")
            exit(1)
