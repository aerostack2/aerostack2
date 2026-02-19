#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
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
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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

"""Visual Odometry"""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class VisualOdometry(Node):
    def __init__(self):
        super().__init__('visual_odometry')
        self.declare_parameter(
            'image_topic', '/drone0/sensor_measurements/camera/image/compressed')
        self.image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value

        self.qos_profile = QoSProfile(depth=10)
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            self.qos_profile
        )
        self.bridge = CvBridge()

        # Initialize OpenCV ORB detector
        self.orb = cv2.ORB_create(
            nfeatures=25,  # Number of keypoints to Detect
            scaleFactor=1.2,  # Scale factor for the pyramid
            nlevels=8,  # Number of pyramid levels
            edgeThreshold=5,  # Size of the border where the features are not detected
            firstLevel=0,  # Level of pyramid to start detecting features
            WTA_K=2,  # Number of points that produce each element of the vectors
            scoreType=cv2.ORB_HARRIS_SCORE,  # Type of score to use for keypoint detection
            patchSize=31,  # Size of the patch used to compute the descriptors
            fastThreshold=5  # Threshold for the FAST feature detector
        )
        # Initialize CLAHE (Contrast Limited Adaptive Histogram Equalization)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        # Matcher
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.last_keypoints = None
        self.last_descriptors = None
        self.last_image = None

    def image_callback(self, msg):
        try:
            # Convert the compressed image to a CV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            # Paint top half of the image in black
            # cv_image[:int(cv_image.shape[0] * 0.6), :] = 0
            # Paint the bottom part of the image in black
            cv_image[int(cv_image.shape[0] * 0.9):, :] = 0

            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # gray_image = self.clahe.apply(gray_image)
            h, w = gray_image.shape[:2]
            tiles_r, tiles_c = 3, 2
            per_tile = 25  # so total = tiles_r * tiles_c * per_tile
            all_kp = []
            for i in range(tiles_r):
                for j in range(tiles_c):
                    mask = np.zeros_like(gray_image, dtype=np.uint8)
                    y0, y1 = i*h//tiles_r, (i+1)*h//tiles_r
                    x0, x1 = j*w//tiles_c, (j+1)*w//tiles_c
                    mask[y0:y1, x0:x1] = 255

                    kps = self.orb.detect(gray_image, mask)
                    # sort by response and take the top N of each tile
                    kps = sorted(kps, key=lambda kp: kp.response,
                                 reverse=True)[:per_tile]
                    all_kp += kps
            keypoints, descriptors = self.orb.compute(gray_image, all_kp)
            # # Detect ORB keypoints and descriptors
            # keypoints, descriptors = self.orb.detectAndCompute(
            #     gray_eq, None)
            if descriptors is not None and self.last_descriptors is not None:
                # Match descriptors with the last frame
                matches = self.bf_matcher.match(
                    self.last_descriptors, descriptors)
                matches = sorted(matches, key=lambda x: x.distance)

                # Draw matches
                match_image = cv2.drawMatches(
                    self.last_image,
                    self.last_keypoints,
                    gray_image,
                    keypoints,
                    matches,
                    None,
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                )

                # Compute the translation and rotation
                if len(matches) > 10:
                    src_pts = np.float32(
                        [self.last_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                    dst_pts = np.float32(
                        [keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

                    # Find the essential matrix
                    E, mask = cv2.findEssentialMat(
                        src_pts, dst_pts, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                    print(f'E:\n{E}\n')
                    if E is not None:
                        _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts)
                        self.get_logger().info(
                            f'Rotation:\n{R}\nTranslation:\n{t}\n')
                    else:
                        self.get_logger().warn('Essential matrix could not be computed.')

                # Show the matched image
                cv2.imshow('Matches', match_image)
                k = cv2.waitKey(1)
                if k == ord('q'):
                    cv2.destroyAllWindows()
                    exit()
            else:
                self.get_logger().info('No previous frame to match against.')
            self.last_keypoints = keypoints
            self.last_descriptors = descriptors
            self.last_image = cv_image

            # Show the image (for debugging purposes)
            # cv2.imshow('Visual Odometry Image', cv_image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
