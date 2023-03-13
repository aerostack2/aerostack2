#!/bin/python3

from as2_platform_crazyflie.viewer import aideckPublisher
import rclpy

def main(args=None):
    rclpy.init(args=args)
    stream_pub = aideckPublisher()
    rclpy.spin(stream_pub)
    stream_pub.close()
    stream_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
