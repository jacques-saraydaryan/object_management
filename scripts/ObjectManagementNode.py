#!/usr/bin/env python
__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy
import actionlib

from math import sqrt, pow

from robocup_msgs.msg import Entity2D,Entity2DList
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from object_management.msg import ObjectDetectionAction,ObjectDetectionResult
from object_management.msg import LookAtObjectAction,LookAtObjectResult
from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv, ObjectsDetectionGateway_distSorted_Srv

from dialogue_hri_srvs.srv import MoveTurn, PointAt


class ObjectManagementNode():
    MOVE_HEAD_AROUND_NB_HIT=4
    MOVE_HEAD_YAW_ANGLE=0.2
    MOVE_HEAD_PITCH_ANGLE=0.35

    def __init__(self):
        rospy.init_node('object_management_node', anonymous=False)
        self.configure()

        # Connect to move_head_pose_srv service
        try:
            rospy.wait_for_service('/move_head_pose_srv', 10.0)
            rospy.loginfo("end service move_head_pose_srv wait time")
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)


        # Connect to move_turn_service service
        try:
            rospy.wait_for_service('move_turn_service', 10.0)
            rospy.loginfo("end service move_turn_service wait time")
            self._moveTurn = rospy.ServiceProxy('move_turn_service', MoveTurn)
        except Exception as e:
            rospy.logerr("Service move_turn_service call failed: %s" % e)

        # Connect to move_turn_service service
        try:
            rospy.wait_for_service('point_at', 10.0)
            rospy.loginfo("end service point_at wait time")
            self._moveTurn = rospy.ServiceProxy('point_at', PointAt)
        except Exception as e:
            rospy.logerr("Service point_at call failed: %s" % e)

        # Connect to object_detection_gateway_srv service
        try:
            rospy.wait_for_service('/object_detection_gateway_srv', 10.0)
            rospy.loginfo("end service object_detection_gateway_srv wait time")
            self._objectDetectionGateway = rospy.ServiceProxy('object_detection_gateway_srv', ObjectsDetectionGateway_Srv)
        except Exception as e:
            rospy.logerr("Service object_detection_gateway_srv call failed: %s" % e)

        # Connect to object_detection_gateway_distSorted_srv service
        try:
            rospy.wait_for_service('/object_detection_gateway_distSorted_srv', 10.0)
            rospy.loginfo("end service object_detection_gateway_distSorted_srv wait time")
            self._objectDetectionDistSortedGateway = rospy.ServiceProxy('object_detection_gateway_distSorted_srv', ObjectsDetectionGateway_distSorted_Srv)
        except Exception as e:
            rospy.logerr("Service object_detection_gateway_distSorted_srv call failed: %s" % e)


         # create action server and start it
        self._actionServerObjectDetection = actionlib.SimpleActionServer('object_detection_action', ObjectDetectionAction, self.executeObjectDetectionActionServer, False)
        self._actionServerObjectDetection.start()

         # create action server and start it
        self._actionServerLookAtObject = actionlib.SimpleActionServer('look_at_object_action', LookAtObjectAction, self.executeLookAtObjectActionServer, False)
        self._actionServerLookAtObject.start()

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

    def moveTurn(self,rad):
        try:
            self._moveTurn = rospy.ServiceProxy('move_turn_service', MoveTurn)
            result=self._moveTurn(rad)
        except Exception as e:
            rospy.logerr("Service move_turn_service call failed: %s" % e)
            return

    def pointAt(self,x, y, z, head, arm, duration):
        try:
            self._pointAt = rospy.ServiceProxy('point_at', PointAt)
            result=self._pointAt(x, y, z, head, arm, duration)
        except Exception as e:
            rospy.logerr("Service point_at call failed: %s" % e)
            return


    def executeObjectDetectionActionServer(self, goal):
        isActionSucceed=False
        action_result = ObjectDetectionResult()
        try:
            labeldict = {}
            if goal.moveHead == True:
                labeldict = self.processObjectDetectionWithLookAround(goal.labels)
            else:
                labeldict = self.processObjectDetectionWithLookAhead(goal.labels)
            #Create associated entityList
            action_result.labelList=labeldict.keys()
            action_result.labelFound=labeldict.values()
            isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to execute action %s:, error:[%s]",str(action_result), str(e))
        if isActionSucceed:
            self._actionServerObjectDetection.set_succeeded(action_result)
        else:
            self._actionServerObjectDetection.set_aborted()

    def processObjectDetectionWithLookAround(self, object_group_list):
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
            resultListB=self._processMoveHeadAndImg(-self.MOVE_HEAD_PITCH_ANGLE,self.MOVE_HEAD_YAW_ANGLE*i*-1,object_group_list)
            for label in resultListB:
                if label in resultObjLabelMap:
                    resultObjLabelMap[label]=resultObjLabelMap[label]+1
                else:
                    resultObjLabelMap[label]=1
        self.moveHead(0.0,0.0)
        rospy.loginfo("----------END---------")
        rospy.loginfo(resultObjLabelMap)
        #TODO return list of detected entity 2D
        return resultObjLabelMap

    def processObjectDetectionWithLookAhead(self, object_group_list):
        resultObjLabelMap={}
        rospy.loginfo("----------BEGIN---------")
        result=self._objectDetectionGateway(object_group_list)
        #rospy.loginfo(result)
        for entity in result.entities.entity2DList:
            rospy.loginfo(entity.label)
            if entity.label in resultObjLabelMap:
                resultObjLabelMap[entity.label]=resultObjLabelMap[entity.label]+1
            else:
                resultObjLabelMap[entity.label]=1
        rospy.loginfo("----------END---------")
        rospy.loginfo(resultObjLabelMap)
        #TODO return list of detected entity 2D
        return resultObjLabelMap

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

    #TODO :
    def executeLookAtObjectActionServer(self, goal):
        isActionSucceed=False
        action_result = LookAtObjectResult()
        try:
            result = self.processTurnToObjectCenter(goal.labels, goal.index, goal.head, goal.base, goal.finger)
            #Action output
            action_result.nb_people=len(result.pitchList)
            isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to execute action %s:, error:[%s]",str(action_result), str(e))
        if isActionSucceed:
            self._actionServerLookAtObject.set_succeeded(action_result)
        else:
            self._actionServerLookAtObject.set_aborted()

    def processTurnToObjectCenter(self, object_group_list, index, head, base, finger):
        result=self._objectDetectionDistSortedGateway(object_group_list)
        if len(result.yawList) > 0:
            if index >= len(result.pitchList):
                index = len(result.pitchList) - 1
            if index < 0 :
                index = len(result.pitchList) - index
                if index < 0:
                    index = 0

            print "pitch = %.3f \t yaw = %.3f" % (result.pitchList[index],result.yawList[index])

            if head == True and base == False :
                self.moveHead(result.pitchList[index],result.yawList[index])

            if head == False and base == True :
                self.moveTurn(result.yawList[index])

            if head == True and base == True :
                self.moveTurn(result.yawList[index])
                self.moveHead(result.pitchList[index],0.0)

            if finger == True:
                self.pointAt(3, 0, 0, False, True, 1)
        else:
            rospy.logwarn("Nothing found")

        return result

def main():
    #""" main function
    #"""
    node = ObjectManagementNode()

if __name__ == '__main__':
    main()
