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
if not cv2.__version__ == '3.1.0-dev':
    import cv2.cv as cv
import math

class Keeper(object):
    """ The Keeper is a Python object that encompasses a ROS node
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('keeper')
        self.cmd_vel = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
        self.blue = 150
        self.red = 100
        self.green = 100
        self.max_radius_1 = 150
        self.min_radius_1 = 10
        self.param1 = 50
        self.param2 = 20
        self.kp = 0.05
        if cv2.__version__ == '3.1.0-dev':
            self.houghGrad = cv2.HOUGH_GRADIENT
        else:
            self.houghGrad = cv.CV_HOUGH_GRADIENT
        self.image_info_window = None
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.binary_image = None
        self.lastCircle = [0, 0, 0]
        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        cv2.namedWindow('image_info')
        # cv2.setMouseCallback('video_window', self.process_mouse_event)
        self.center_x=None
        self.center_y = None
        self.circles = None
        self.min_dist = 0

        cv2.createTrackbar('Minimum Distance', 'video_window', 60, 255, self.set_min_dist)
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

    def set_min_dist(self, val):
        """ Sets min dist btw circles"""
        self.min_dist = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(self.cv_image, (50,0,0), (self.blue,self.green,self.red))
        #moments = cv2.moments(self.binary_image)

        # if moments['m00'] != 0:

        #     self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
        self.binary_image = cv2.medianBlur(self.binary_image, 5)
        self.circles = cv2.HoughCircles(cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY),self.houghGrad,1,255,
                            param1=70,param2=30,minRadius=self.min_radius_1,maxRadius=self.max_radius_1)
        #self.circles = cv2.HoughCircles(self.binary_image,cv.CV_HOUGH_GRADIENT,1,20,
                            #param1=self.param1,param2=self.param2,minRadius=self.min_radius_1,maxRadius=self.max_radius_1)
        # cv2.circle(self.cv_image, (int(self.center_x),int(self.center_y)), 10, (255,255,255))

    def blue_checker(self, circle):
        """ checks each circle to see how blue it is at several pixels within the circle"""
        thetas = (0, 45, 90, 135, 180, 125,270, 315)
        s = 0
        for t in thetas:
            y_cord = int(.75*circle[2]*math.sin(math.radians(t))+circle[0])
            x_cord = int(.75*circle[2]*math.cos(math.radians(t))+circle[1])
            if x_cord < self.cv_image.shape[1] and y_cord < self.cv_image.shape[0]:
                s += self.cv_image[y_cord, x_cord,0]
        return s


    def run(self):
        """ The main run loop, in this node it doesn't do anything """

        r = rospy.Rate(5)
        xLin = 0
        xAng = 0
        while not rospy.is_shutdown():
            if not (self.circles == None):
                bestFitCircle = [0, 0, 0]
                current_max = 0
                for q in self.circles[0,:]:
                    # draw the outer circle
                    if self.blue_checker(q) > current_max:
                        current_max = self.blue_checker(q)
                        bestFitCircle = q

                if not self.cv_image == None:
                    cv2.circle(self.cv_image,(bestFitCircle[0],bestFitCircle[1]), bestFitCircle[2],(0,255,0),2)
                        # draw the center of the circle
                    cv2.circle(self.cv_image,(bestFitCircle[0],bestFitCircle[1]),2,(0,0,255),3)
                        # print(len(self.circles))
                print (bestFitCircle[1] - self.lastCircle[1])
                if bestFitCircle[1] - self.lastCircle[1] > 4:
                    xAng = max(min(self.kp*(bestFitCircle[1] - self.lastCircle[1]), 1), -1)


                self.lastCircle = bestFitCircle
            if not self.binary_image is None:
                cv2.imshow('video_window2', self.binary_image)
            if not self.cv_image is None:
                cv2.imshow('video_window', self.cv_image)
            if not self.image_info_window is None:
                cv2.imshow('image_info', self.image_info_window)

            self.cmd_vel = Twist(linear=Vector3(x=xLin), angular=Vector3(z=xAng))
            print(self.cmd_vel)
            cv2.waitKey(5)
            self.pub.publish(self.cmd_vel)
            # start out not issuing any motor commands
            r.sleep()

    # def process_mouse_event(self, event, x,y,flags,param):
    #     """ Process mouse events so that you can see the color values associated
    #         with a particular pixel in the camera images """
    #     self.image_info_window = 255*np.ones((500,500,3))
    #     cv2.putText(self.image_info_window,
    #                 'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
    #                 (5,50),
    #                 cv2.FONT_HERSHEY_SIMPLEX,
    #                 1,
    #                 (0,0,0))


if __name__ == '__main__':
    node = Keeper("/camera/image_raw")
    node.run()
