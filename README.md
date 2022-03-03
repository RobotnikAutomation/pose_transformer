# pose_transformer

Node to transform pose from one frame of reference to another. 
Allows transformation using both [geometry_msgs/Pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html) messages and poses split in 
two geometry_msgs/Point messages, one for position and other for orientation (rpy). 

## Installation

1. Clone the repository in your workspace

```bash
git clone https://www.github.com/RobotnikAutomation/pose_transformer
```

2. Download Robotnik packages

    - [rcomponent](https://www.github.com/RobotnikAutomation/rcomponent)
    - [robotnik_msgs](https://www.github.com/RobotnikAutomation/robotnik_msgs)

3. Install dependencies using rosdep

```bash
rosdep install --from-path src --ignore-src -r -y 
```

## Topics

By inheriting from RComponent, this node shares all topics subscriptions and publications of the base RComponent node.

- Publications
    - ~/source_pose ([geometry_msgs/Pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)): Publishes the pose passed by service.
    - ~/target_pose ([geometry_msgs/Pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)): Publishes the pose passed by service but transformed to the target frame.

## Services
- Servers
    - ~/transform_pose ([pose_transformer/TransformPose](./srv/TransformPose.srv)): Transforms pose from source_frame to target_frame.

    - ~/transform_pose_rpy ([pose_transformer/TransformPoseRPY](./srv/TransformPoseRPY.srv)): Transforms position and orientation from source_frame to target_frame
    
## Launch example
```bash
roslaunch pose_transformer pose_transformer.launch 
```