# AS2_behavior library

Aerostack2 uses Behaviors to handle mission execution

Each behavior is separated in a behavior Server and a behavior client.
They are similar to rclcpp actions but with some differences. 
- Each behavior server only attends to a single goal (client) at each moment.
- Feedback is visible for all nodes 
- They extend start, stop, and feedback with modify, pause, resume, and behavior state data.

We use ros2 services for:
- start
- modify (the same syntax than start)
- pause (std_srvs::srv::Empty)
- resume (std_srvs::srv::Empty)
- stop (std_srvs::srv::Empty)

We use ros2 topics for:
- Feedback
- State

We take advantage of ros2 action message definitions:
- Goal: will be used in start and modify services
- Feedback: Will be used in feedback
- Response: (Maybe in state, or in inmediate behaviors)

We difference between Inmediate, Recurrent and Regular (or goal oriented) behaviors:
- Inmediate: act like a service, they dont give Fb and cannot be paused or modified.
- Recurrent: they give feedback until they are stopped or paused.
- Goal Oriented: they keep runing until a termination condition is met.
