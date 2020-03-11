#!/usr/bin/env python
import rospy
#from std_msgs import image_raw
from sensor_msgs.msg import Image

class KinectTranslator():

    pub = ''
    image = Image

    def __init__(self):
        
        rospy.init_node('kinectTranslator', anonymous=False)
        self.pub = rospy.Publisher('/palbator/object_mng/kinect2_video', Image, queue_size=10)
        rospy.Subscriber('/kinect2/hd/image_color', Image, self.callback)

        self.publishRate()

    def callback(self, data):
        self.image=data

    def publishRate(self):
        rrrr= rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub.publish(self.image)
            # rospy.loginfo("Image published")
            rrrr.sleep()   

if __name__ == '__main__':
    try:
        K = KinectTranslator()
    except rospy.ROSInterruptException:
        pass