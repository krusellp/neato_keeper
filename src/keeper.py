#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import cv2
import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class Keeper(object):
    """ The Keeper is a Python object that encompasses a ROS node
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('keeper')
        self.blue = 150
        self.red = 100
        self.green = 100
        self.max_radius_1 = 150
        self.min_radius_1 = 10
        self.param1 = 50
        self.param2 = 20

        self.image_info_window = None
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.binary_image = None
        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        cv2.namedWindow('image_info')
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        self.center_x=None
        self.center_y = None
        self.circles = None

        cv2.createTrackbar('param1', 'video_window', 0, 255, self.set_param1)
        cv2.createTrackbar('param2', 'video_window', 0, 255, self.set_param2)
        cv2.createTrackbar('min_radius_1', 'video_window', 0, 255, self.set_min_radius_1)
        cv2.createTrackbar('max_radius_1', 'video_window', 0, 2000, self.set_max_radius_1)
        cv2.createTrackbar('red', 'video_window', 0, 255, self.set_red)
        cv2.createTrackbar('green', 'video_window', 0, 255, self.set_green)
        cv2.createTrackbar('blue', 'video_window', 0, 255, self.set_blue)

    def set_blue(self, val):
        """set value of blue max"""
        self.blue = val

    def set_red(self, val):
        """ set value of red"""
        self.red = val

    def set_green(self, val):
        """ set val of green """
        self.green = val

    def set_param1(self, val):
        """ set val of param1"""
        self.param1 = val

    def set_param2(self, val):
        """ sets val of param2"""
        self.param2 = val

    def set_min_radius_1(self, val):
        """ sets min radius"""
        self.min_radius_1 = val

    def set_max_radius_1(self, val):
        """ sets max radius"""
        self.max_radius_1 = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(self.cv_image, (50,0,0), (self.blue,self.green,self.red))
        #moments = cv2.moments(self.binary_image)

        # if moments['m00'] != 0:

        #     self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
        self.binary_image = cv2.medianBlur(self.binary_image, 5)
        self.circles = cv2.HoughCircles(cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY),cv2.HOUGH_GRADIENT,1,20,
                            param1=self.param1,param2=self.param2,minRadius=self.min_radius_1,maxRadius=self.max_radius_1)
        #self.circles = cv2.HoughCircles(self.binary_image,cv.CV_HOUGH_GRADIENT,1,20,
                            #param1=self.param1,param2=self.param2,minRadius=self.min_radius_1,maxRadius=self.max_radius_1)
        # cv2.circle(self.cv_image, (int(self.center_x),int(self.center_y)), 10, (255,255,255))
        if not (self.circles == None):
            for i in self.circles[0,:]:
                # draw the outer circle
                cv2.circle(self.cv_image,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(self.cv_image,(i[0],i[1]),2,(0,0,255),3)
                # print(len(self.circles))

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():


            if not self.binary_image is None:
                cv2.imshow('video_window2', self.binary_image)
            if not self.cv_image is None:
                cv2.imshow('video_window', self.cv_image)
            if not self.image_info_window is None:
                cv2.imshow('image_info', self.image_info_window)

            cv2.waitKey(5)

            # start out not issuing any motor commands
            r.sleep()

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((500,500,3))
        cv2.putText(self.image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))


if __name__ == '__main__':
    node = Keeper("/camera/image_raw")
    node.run()
