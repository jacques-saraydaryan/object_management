#!/usr/bin/env python  
__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy 
import actionlib

from robocup_msgs.msg import Entity2D,Entity2DList
from object_management.msg import ObjectDetectionAction,ObjectDetectionResult


class ObjectManagement():

    def __init__(self):
        rospy.init_node('object_management_node', anonymous=False)
        self.configure()


        # Subscribe to the image 
        #self.sub_rgb = rospy.Subscriber("/image", Image, self.rgb_callback, queue_size=1)
        
         # create action server and start it
        self._actionServer = actionlib.SimpleActionServer('object_detection_action', ObjectDetectionAction, self.executeObjectDetectionActionServer, False)
        self._actionServer.start()

        rospy.spin()


    def configure(self):
        #load face files form data directory
        #self.NB_KMEAN_CLUSTER=rospy.get_param('kmean_cluster',3)
        #rospy.loginfo("Param: kmean_cluster:"+str(self.NB_KMEAN_CLUSTER))
        pass

    def processObjectDetection(self,object_group_list):
        #FIXME TODO
        pass

    def executeObjectDetectionActionServer(self, goal):
        isActionSucceed=False
        action_result = ObjectDetectionResult()
        try:
            entityList =self.processObjectDetection(goal.labels)
            #Create associated entityList
            action_result.entityList=entityList
            
            isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to execute action %s:, error:[%s]",str(action_result), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded(action_result)
        else:
            self._actionServer.set_aborted()

def main():
    #""" main function
    #"""
    node = ObjectManagement()

if __name__ == '__main__':
    main()