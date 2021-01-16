#! /usr/bin/env python
'''Publish ground truth pose of base_link from Gazebo.'''
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import rospy

def publish_ground_truth():
    rospy.init_node('pose_ground_truth')

    odom_pub=rospy.Publisher ('/pose_ground_truth', 
    							Odometry, queue_size=10)

    rospy.wait_for_service ('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', 
                                        GetModelState)

    odom=Odometry()
    header = Header()
    header.frame_id='odom'
    child_frame='base_link'

    model = GetModelStateRequest()
    model.model_name='rrbot'

    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        result = get_model_srv(model)

        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        header.stamp = rospy.Time.now()
        odom.header = header
        odom.child_frame_id=child_frame

        odom_pub.publish (odom)

        r.sleep()

if __name__ == "__main__":
    publish_ground_truth()