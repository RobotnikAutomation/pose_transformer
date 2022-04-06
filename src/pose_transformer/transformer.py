#!/usr/bin/env python
from operator import truediv
import rospy

from tf.listener import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped

from rcomponent.rcomponent import *
from pose_transformer.srv import TransformPose, TransformPoseResponse, \
                                TransformPoseRequest, TransformPoseRPY, \
                                TransformPoseRPYResponse

class PoseTransformer(RComponent):

    def __init__(self):
        self.transform_pose_service = None
        self.source_pose_msg = PoseStamped()
        self.target_pose_msg = PoseStamped()

        RComponent.__init__(self)
    
    def ros_read_params(self):
        RComponent.ros_read_params(self)


    def setup(self):
        RComponent.setup(self)

    def ros_setup(self):

        RComponent.ros_setup(self)

        self.transform_pose_service = rospy.Service('~transform_pose', TransformPose, \
            self.transform_pose_service_cb)
        
        self.transform_pose_service = rospy.Service('~transform_pose_rpy', TransformPoseRPY, \
            self.transform_pose_rpy_service_cb)
        self.source_pose_pub = rospy.Publisher('~source_pose', PoseStamped, queue_size=10)
        self.target_pose_pub = rospy.Publisher('~target_pose', PoseStamped, queue_size=10)

        self.tf_listener = TransformListener()
        
        return 0
    
    def ready_state(self):
        return
    
    def shutdown(self):        
        RComponent.shutdown(self)
    
    def ros_publish(self):
        self.source_pose_pub.publish(self.source_pose_msg)
        self.target_pose_pub.publish(self.target_pose_msg)

        RComponent.ros_publish(self)

    def transform_pose_service_cb(self, msg):
        response = TransformPoseResponse()
        response.success = False
        response.message = "Unable to transform pose from " + msg.source_frame \
            + " to " + msg.target_frame
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = msg.source_frame
        pose.pose = msg.pose

        try:
            self.tf_listener.waitForTransform(msg.source_frame, msg.target_frame,\
                 rospy.Time.now(), rospy.Duration(2.0))
            transformed_point = self.tf_listener.transformPose(msg.target_frame, pose)
            response.message = "Pose transformed correctly"
            response.pose = transformed_point.pose
            response.success = True

            self.source_pose_msg = pose
            self.target_pose_msg.header.stamp = rospy.Time.now()
            self.target_pose_msg.header.frame_id = msg.target_frame
            self.target_pose_msg.pose = transformed_point.pose
        except Exception as e: 
            rospy.logerr("%s::transform_pose_service_cb: Couldn't get transform (%s --> %s): %s" \
                %(self._node_name, msg.source_frame,  msg.target_frame, e))
            response.message = str(e)

            self.source_pose_msg = PoseStamped()
            self.target_pose_msg = PoseStamped()
            

        return response

    def transform_pose_rpy_service_cb(self, msg):

        # Transforms euler orientation to quaternion
        roll = msg.orientation.x
        pitch = msg.orientation.y
        yaw = msg.orientation.z
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        # Simulates a transform_pose_service call
        request = TransformPoseRequest()
        request.source_frame = msg.source_frame
        request.target_frame = msg.target_frame
        request.pose.position = msg.position
        request.pose.orientation.x  = quaternion[0]
        request.pose.orientation.y  = quaternion[1]
        request.pose.orientation.z  = quaternion[2]
        request.pose.orientation.w  = quaternion[3]
        response_pose = self.transform_pose_service_cb(request)

        # Extracts result, position and orientation(rpy) from transform_pose_service 
        response = TransformPoseRPYResponse()
        response.success = response_pose.success
        response.message = response_pose.message
        response.position = response_pose.pose.position

        response_quat = [response_pose.pose.orientation.x, \
                        response_pose.pose.orientation.y,  \
                        response_pose.pose.orientation.z,  \
                        response_pose.pose.orientation.w]

        response_orientation = euler_from_quaternion(response_quat)
        response.orientation.x = response_orientation[0]
        response.orientation.y = response_orientation[1]
        response.orientation.z = response_orientation[2]

        return  response
        
