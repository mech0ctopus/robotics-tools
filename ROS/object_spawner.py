#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
from gazebo_msgs.srv import DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Pose, Point, Quaternion
from time import sleep

#For reference, to spawn/delete model with command line
#rosrun gazebo_ros spawn_model -file object.urdf -urdf -z 1 -model debris
#rosservice call gazebo/delete_model '{model_name: debris}'

def spawn(template_path, name, x, y, z): #r
    '''Spawn model in Gazebo.'''
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    try:
        #Read template
        with open(template_path, "r") as f:
            template = f.read()

        #Define service proxy
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        #Format location and define orientation
        item_pose = Pose(Point(x=x, y=y, z=z), Quaternion(0,0,0,1))

        #Spawn model
        spawn_model(name, template, "", item_pose, "map")

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def delete_model(model_name):
    '''Delete model in Gazebo.'''
    rospy.wait_for_service("/gazebo/delete_model")

    try:
        #Define service proxy
        del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        del_model(model_name)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        #Spawn and delete several items for testing
        for i in range(10):
            spawn(template_path="object.urdf", name="debris_"+str(i), x=10*i,y=5*i,z=0) #r=.05, 
            sleep(3)
            delete_model("debris_"+str(i))
    except rospy.ROSInterruptException:
        pass
