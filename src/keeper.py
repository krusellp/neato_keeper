#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import cv2
import rospy
import copy
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
        self.blue_ub = 150
        self.red_ub = 100
        self.green_ub = 100
        self.max_radius_1 = 150
        self.min_radius_1 = 10
        self.blue_lb= 0
        self.red_lb = 0
        self.kp = 0.05
        if cv2.__version__ == '3.1.0-dev':
            self.houghGrad = cv2.HOUGH_GRADIENT
        else:
            self.houghGrad = cv.CV_HOUGH_GRADIENT
        self.image_info_window = None
        self.cv_image = None
        self.all_circles = None                      # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.binary_image = None
        self.bestImage = None
        self.lastCircle = [0, 0, 0]
        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        cv2.namedWindow('video_window2')
        cv2.namedWindow('image_info')
        cv2.namedWindow('mask')
        # cv2.setMouseCallback('video_window', self.process_mouse_event)
        self.center_x=None
        self.center_y = None
        self.green_lb = 0
        self.xAng = 0
        self.maskList= []
        self.update = True
        self.last_time = 0
        self.time_stamp = 1
        self.delta_t = .01

        cv2.createTrackbar('blue_lb', 'video_window', 0, 255, self.set_blue_lb)
        cv2.createTrackbar('green_lb', 'video_window', 60, 255, self.set_green_lb)
        cv2.createTrackbar('red_lb', 'video_window', 0, 255, self.set_red_lb)
        cv2.createTrackbar('blue_ub', 'video_window', 0, 255, self.set_blue_ub)
        cv2.createTrackbar('red_ub', 'video_window', 0, 255, self.set_red_ub)
        cv2.createTrackbar('green_ub', 'video_window', 0, 255, self.set_green_ub)
        cv2.createTrackbar('min_radius_1', 'video_window', 0, 255, self.set_min_radius_1)
        cv2.createTrackbar('max_radius_1', 'video_window', 0, 2000, self.set_max_radius_1)



    def set_blue_ub(self, val):
        """set value of blue max"""
        self.blue_ub = val

    def set_red_ub(self, val):
        """ set value of red"""
        self.red_ub = val

    def set_green_ub(self, val):
        """ set val of green """
        self.green_ub = val

    def set_blue_lb(self, val):
        """ set val of param1"""
        self.blue_lb = val

    def set_red_lb(self, val):
        """ sets val of param2"""
        self.red_lb = val

    def set_min_radius_1(self, val):
        """ sets min radius"""
        self.min_radius_1 = val

    def set_max_radius_1(self, val):
        """ sets max radius"""
        self.max_radius_1 = val

    def set_green_lb(self, val):
        """ Sets min dist btw circles"""
        self.green_lb = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(self.cv_image, (self.blue_lb,self.green_lb,self.red_lb), (self.blue_ub,self.green_ub,self.red_ub))
        #moments = cv2.moments(self.binary_image)
        # print msg.header.stamp.to_sec()
        if self.update:
            self.time_stamp = msg.header.stamp.to_sec()
            self.update = False

        #self.circles = cv2.HoughCircles(self.binary_image,cv.CV_HOUGH_GRADIENT,1,20,
                            #param1=self.param1,param2=self.param2,minRadius=self.min_radius_1,maxRadius=self.max_radius_1)
        # cv2.circle(self.cv_image, (int(self.center_x),int(self.center_y)), 10, (255,255,255))

    def blue_checker(self, circle):
        """ checks each circle to see how blue it is at several pixels within the circle"""
        thetas = (0, 45, 90, 135, 180, 125,270, 315)
        r = 0
        b = 0
        for t in thetas:
            y_cord = int(.75*circle[2]*math.sin(math.radians(t))+circle[0])
            x_cord = int(.75*circle[2]*math.cos(math.radians(t))+circle[1])
            if x_cord < self.cv_image.shape[1] and y_cord < self.cv_image.shape[0]:
                b += self.cv_image[y_cord, x_cord, 0]
                r += self.cv_image[y_cord, x_cord, 2]
        print('blueness = ', b, 'redness =', r)
        return r,b

    def mask_checker(self, circle):
        """ checks each circle to see how masked it is at several pixels within the circle"""
        thetas = (0, 45, 90, 135, 180, 125, 270, 315)
        Rs = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)
        s = 0
        for r in Rs:
            for t in thetas:
                y_cord = int(r*circle[2]*math.sin(math.radians(t))+circle[0])
                x_cord = int(r*circle[2]*math.cos(math.radians(t))+circle[1])
                if x_cord < self.binary_image.shape[1] and y_cord < self.binary_image.shape[0]:
                    s += min(self.binary_image[y_cord, x_cord], 1)

        print('binary val : ', s)
        return s

    def bestCircleCalc(self, maskImg):

        current_max = 0
        bestCircleIndex = 0
        for i, mask in enumerate(self.maskList):
            moment = cv2.moments(mask)



            if moment['m00'] > current_max:
                current_max = moment['m00']
                bestCircleIndex = i


        print bestCircleIndex
        self.bestCircle = self.circles[0, bestCircleIndex]


    def predictor(self, circle_prev, circle_current):
        """choses a direction based on the ball's path fro 2 frames"""
        # if circle_current[1] - circle_prev[1] > 0:
        #     # return negative value
        #     self.xAng = .5 * -1
        # if circle_current[1] - circle_prev[1] < 0:
        #     # return positive value
        #     self.xAng = .5 * 1
        # return self.xAng

        # finds distance of first and second circles
        a = 221.2
        b = -0.3178
        # fps = 60

        d1 = circle_prev[2]
        d2 = circle_current[2]

        # print('last time', self.last_time)
        # print('time stamp', self.time_stamp)
        if self.last_time != self.time_stamp:
            self.delta_t = self.last_time - self.time_stamp

        if not d1 == 0:

            ft_px1 = 7 / (24 * circle_prev[2])
            ft_px2 = 7 / (24 * circle_current[2])

            y_dist_1 = math.log((d1 / a)) / b
            y_dist_2 = math.log((d2 / a)) / b

            size = (self.cv_image.shape[0] / 2) + 90
            x_dist_1 = (circle_prev[0] - size) * ft_px1
            x_dist_2 = (circle_current[0] - size) * ft_px2
            
            y_diff = (y_dist_2 - y_dist_1) / self.delta_t # gives speed in ft/s
            x_diff = (x_dist_2 - x_dist_1) / self.delta_t
            return y_diff, x_diff
        else:
            return 0,0



    def vector_finder(self, dx, dy):
        """ creates a vector"""
        if dy != 0:
            angle = math.atan(dx/dy)
            mag = math.hypot(dx,dy)
            return angle, mag
        else:
            return 0, 0


    # def get_dist(self, radius):
    #     a = 221.2
    #     b = -0.3178
    #     d =  radius
    #     dist = math.log((d / a)) / b
    #     return dist




    def run(self):
        """ The main run loop, in this node it doesn't do anything """

        r = rospy.Rate(5)
        xLin = 0
        xAng = 0

        while not rospy.is_shutdown():

            self.all_circles = copy.deepcopy(self.cv_image)
            cvImg = copy.deepcopy(self.cv_image)
            binImg = copy.deepcopy(self.binary_image)

            if not cvImg is None:
            # if moments['m00'] != 0:

            #     self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
                binImg= cv2.medianBlur(binImg, 5)
                self.circles = cv2.HoughCircles(cv2.cvtColor(cvImg, cv2.COLOR_BGR2GRAY),self.houghGrad,1,100,
                                    param1=70,param2=30,minRadius=self.min_radius_1,maxRadius=self.max_radius_1)

                bestFitCircle = [0, 0, 0]
                self.maskList = []
                if not (self.circles == None):
                    # current_max_rd = 0
                    # current_max_bl = 0
                    current_max = 0

                    for q in self.circles[0, :]:
                        # draw the outer circle
                        cv2.circle(self.all_circles, (q[0], q[1]), q[2], (0,255,0), 2)
                        cv2.circle(self.all_circles, (q[0], q[1]), 2, (0,0,255), 3)

                        mask = np.full((self.all_circles.shape[0], self.all_circles.shape[1]), 0, dtype=np.uint8)
                        cv2.circle(mask, (q[0], q[1]), q[2], 255, -1)
                        temp = np.full((self.all_circles.shape[0], self.all_circles.shape[1]), 0, dtype=np.uint8)
                        cv2.bitwise_and(mask, binImg, temp)
                        self.maskList.append(temp)



                        # rd, bl = self.blue_checker(q)
                        # if bl > current_max_bl and rd < current_max_rd:
                        #     current_max_rd = rd
                        #     current_max_bl = bl
                        # s = self.mask_checker(q)
                        # if s > current_max:
                        #     bestFitCircle = q
                        #     current_max = s
                    self.bestCircleCalc(binImg)

                    if not self.cv_image == None:
                        cv2.circle(cvImg,(self.bestCircle[0], self.bestCircle[1]), self.bestCircle[2],(0,255,0),2)
                            # draw the center of the circle
                        cv2.circle(cvImg,(self.bestCircle[0], self.bestCircle[1]),2,(0,0,255),3)
                            # print(len(self.circles))
                    #print (bestFitCircle[1] - self.lastCircle[1])
                    # if bestFitCircle[1] - self.lastCircle[1] > 4:
                        # xAng = max(min(self.kp*(bestFitCircle[1] - self.lastCircle[1]), 1), -1)




                if not self.all_circles is None:
                    cv2.imshow('video_window2', self.all_circles)
                if not self.cv_image is None:
                    cv2.imshow('video_window', cvImg)
                if not self.image_info_window is None:
                    cv2.imshow('image_info', self.image_info_window)
                if not self.binary_image is None:
                    cv2.imshow('mask', binImg)

                # print('radius:  ', self.bestCircle[2])
                # dist = self.get_dist(self.bestCircle[2])
                # print('dist =', dist)
                # print('last', self.lastCircle)
                # print('best', self.bestCircle[0])
                if self.lastCircle[0] != self.bestCircle[0]:
                    dy, dx = self.predictor(self.lastCircle, self.bestCircle)
                    print('dy',dy)
                    print('dx', dx)
                    ang, mag = self.vector_finder(dx, dy)
                    print('ang', ang)
                    print('mag', mag)
                # print('pixels', self.bestCircle[0] - self.cv_image.shape[0]/2)
                self.lastCircle = self.bestCircle

                # self.cmd_vel = Twist(linear=Vector3(x=xLin), angular=Vector3(z=self.predictor(self.lastCircle, bestFitCircle)))
                # print(self.cmd_vel)
                cv2.waitKey(5)
                #self.pub.publish(self.cmd_vel)

                self.last_time = self.time_stamp
                self.update = True
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
