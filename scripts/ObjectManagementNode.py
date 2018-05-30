#!/usr/bin/env python  
__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy 
import actionlib

from robocup_msgs.msg import Entity2D,Entity2DList
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from object_management.msg import ObjectDetectionAction,ObjectDetectionResult
from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv


class ObjectManagementNode():
    MOVE_HEAD_AROUND_NB_HIT=4
    MOVE_HEAD_YAW_ANGLE=0.2
    MOVE_HEAD_PITCH_ANGLE=0.2

    def __init__(self):
        rospy.init_node('object_management_node', anonymous=False)
        self.configure()

        # Connect to move_head_pose_srv service
        try:
            rospy.wait_for_service('/move_head_pose_srv',5)
            rospy.loginfo("end service move_head_pose_srv wait time")
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)


        # Connect to move_head_pose_srv service
        try:
            rospy.wait_for_service('/object_detection_gateway_srv',5)
            rospy.loginfo("end service object_detection_gateway_srv wait time")
            self._objectDetectionGateway = rospy.ServiceProxy('object_detection_gateway_srv', ObjectsDetectionGateway_Srv)
        except Exception as e:
            rospy.logerr("Service object_detection_gateway_srv call failed: %s" % e)


        # Connect to move_head_pose_srv service
        try:
            rospy.wait_for_service('/move_head_pose_srv',5)
            rospy.loginfo("end service move_head_pose_srv wait time")
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
        
         # create action server and start it
        self._actionServer = actionlib.SimpleActionServer('object_detection_action', ObjectDetectionAction, self.executeObjectDetectionActionServer, False)
        self._actionServer.start()

        rospy.spin()


    def configure(self):
        #load face files form data directory
        #self.NB_KMEAN_CLUSTER=rospy.get_param('kmean_cluster',3)
        #rospy.loginfo("Param: kmean_cluster:"+str(self.NB_KMEAN_CLUSTER))
        pass
    
    def moveHead(self,pitch_value,yaw_value):
        try:
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
            result=self._moveHeadPose(pitch_value,yaw_value,True)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            return

    def executeObjectDetectionActionServer(self, goal):
        isActionSucceed=False
        action_result = ObjectDetectionResult()
        try:
            labelList =self.processObjectDetection(goal.labels)
            #Create associated entityList
            action_result.labelList=labelList
            
            isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to execute action %s:, error:[%s]",str(action_result), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded(action_result)
        else:
            self._actionServer.set_aborted()

    def processObjectDetection(self,object_group_list):
        resultObjLabelMap={}
        rospy.loginfo("----------BEGIN---------")
        for i in range(0,self.MOVE_HEAD_AROUND_NB_HIT):
            rospy.loginfo("------------------->")
            resultListA=self._processMoveHeadAndImg(self.MOVE_HEAD_PITCH_ANGLE,self.MOVE_HEAD_YAW_ANGLE*i,object_group_list)
            for label in resultListA:
                if label in resultObjLabelMap:
                    resultObjLabelMap[label]=resultObjLabelMap[label]+1
                else:
                     resultObjLabelMap[label]=1

        for i in range(0,self.MOVE_HEAD_AROUND_NB_HIT):
            rospy.loginfo("------------------->")
            resultListB=self._processMoveHeadAndImg(self.MOVE_HEAD_PITCH_ANGLE,self.MOVE_HEAD_YAW_ANGLE*i*-1,object_group_list)
            for label in resultListB:
                if label in resultObjLabelMap:
                    resultObjLabelMap[label]=resultObjLabelMap[label]+1
                else:
                     resultObjLabelMap[label]=1
        rospy.loginfo("----------END---------")
        rospy.loginfo(resultObjLabelMap)

        #TODO return list of detected entity 2D
        return resultObjLabelMap.keys()

    def _processMoveHeadAndImg(self,pitch_value,yaw_value,object_group_list):
        resultLabelList=[]
        self.moveHead(pitch_value,yaw_value)
        rospy.sleep(2.0)
        result=self._objectDetectionGateway(object_group_list)
        #rospy.loginfo(result)
        for entity in result.entities.entity2DList:
            rospy.loginfo(entity.label)
            resultLabelList.append(entity.label)
        return resultLabelList


def main():
    #""" main function
    #"""
    node = ObjectManagementNode()

if __name__ == '__main__':
    main()