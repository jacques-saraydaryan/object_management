#! /usr/bin/env python
__author__ ='Jacques Saraydaryan'
import rospy
from  object_management.msg import ObjectDetectionAction, ObjectDetectionGoal
import actionlib


def UseAction():
    rospy.init_node('object_management_node_action_test', anonymous=False)

    client = actionlib.SimpleActionClient('object_detection_action', ObjectDetectionAction)

    client.wait_for_server()
    labels=[]
    goal = ObjectDetectionGoal()
    goal.labels=labels

    client.send_goal(goal)

    client.wait_for_result()

    content_result=client.get_result()

    result=content_result.labelList

    #rospy.loginfo("color:%s, color_web:%s, color_temp:%s, color_brightness_name:%s, RGB:[%s,%s,%s], percentage:%s",str(result.color_name),str(result.color_web),str(result.color_temp),str(result.color_brightness_name),str(result.rgb[0]),str(result.rgb[1]),str(result.rgb[2]),str(result.percentage_of_img))
    #rospy.loginfo("ALL MAIN COLORS")
    rospy.loginfo(result)


if __name__ == '__main__':
    try:
        UseAction()
    except rospy.ROSInterruptException:
        pass