#!/usr/bin/env python

import rospy
from transformer import PoseTransformer


def main():

    rospy.init_node("pose_transformer_node")

    rc_node = PoseTransformer()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()