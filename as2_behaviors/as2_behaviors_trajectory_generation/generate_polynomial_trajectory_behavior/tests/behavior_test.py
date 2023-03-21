import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from as2_msgs.action import TrajectoryGenerator
from as2_msgs.msg import PoseWithID, YawMode


class TrajectoryGeneratorClient(Node):

    def __init__(self):
        super().__init__('trajectory_generator_action_client')
        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, False)
        self.set_parameters([self.param_use_sim_time])
        self._action_client = ActionClient(
            self, TrajectoryGenerator, '/drone_sim_0/TrajectoryGeneratorBehavior')

    def send_goal(self):
        goal = TrajectoryGenerator.Goal()
        goal.header.frame_id = "drone_sim_0/odom"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.max_speed = 1.0
        yaw_mode = YawMode()
        yaw_mode.mode = YawMode.FIXED_YAW
        yaw_mode.angle = 0.0
        goal.yaw = yaw_mode

        # pose0 = PoseWithID()
        # pose0.id = "0"
        # pose0.pose.position.x = 0.0
        # pose0.pose.position.y = 0.0
        # pose0.pose.position.z = 1.0
        # goal.path.append(pose0)

        pose1 = PoseWithID()
        pose1.id = "1"
        pose1.pose.position.x = 0.0
        pose1.pose.position.y = 0.0
        pose1.pose.position.z = 2.0
        goal.path.append(pose1)

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryGeneratorClient()
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
