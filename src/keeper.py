#!/usr/bin/env python

import cv2
import rospy
import copy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Twist, Vector3, Point
if not cv2.__version__ == '3.1.0-dev':
    import cv2.cv as cv
import math
from visualization_msgs.msg import Marker


class Keeper(object):
    """ The Keeper is a Python object that encompasses a ROS node
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('keeper')

        #Initialize attributes
        self.cmd_vel = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
        self.blue_ub = 150
        self.green_ub = 100
        self.red_ub = 100
        self.max_radius_1 = 150
        self.min_radius_1 = 10
        self.blue_lb= 0
        self.green_lb = 0
        self.red_lb = 0
        self.kp = 0.05

        # for visualizing the vector
        self.visualizer = rospy.Publisher("/Marker", Marker, queue_size=10)

        #For different versions of OpenCV. Selects which
        #variable based on which version you have
        if cv2.__version__ == '3.1.0-dev':
            self.houghGrad = cv2.HOUGH_GRADIENT
        else:
            self.houghGrad = cv.CV_HOUGH_GRADIENT

        self.cv_image = None
        self.all_circles = None                      # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.binary_image = None
        self.bestImage = None
        self.lastCircle = [0, 0, 0]

        #Publisher to cmd_vel
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        #  initializing variables needed later
        self.center_x=None
        self.center_y = None

        self.xAng = 0
        self.maskList= []
        self.update = True
        self.last_time = 0
        self.time_stamp = 1
        self.delta_t = .01

        #Open windows for images
        cv2.namedWindow('video_window')
        cv2.namedWindow('video_window2')
        cv2.namedWindow('mask')

        #Subscriber gets image data
        rospy.Subscriber(image_topic, Image, self.process_image)

        #Sliders for selecting parameters. Allows adjustment of camera
        cv2.createTrackbar('blue_lb', 'video_window', 15, 255, self.set_blue_lb)
        cv2.createTrackbar('green_lb', 'video_window', 60, 255, self.set_green_lb)
        cv2.createTrackbar('red_lb', 'video_window', 0, 255, self.set_red_lb)
        cv2.createTrackbar('blue_ub', 'video_window', 230, 255, self.set_blue_ub)
        cv2.createTrackbar('red_ub', 'video_window', 0, 255, self.set_red_ub)
        cv2.createTrackbar('green_ub', 'video_window', 117, 255, self.set_green_ub)
        cv2.createTrackbar('min_radius_1', 'video_window', 0, 255, self.set_min_radius_1)
        cv2.createTrackbar('max_radius_1', 'video_window', 0, 2000, self.set_max_radius_1)


    #Callback functions for the sliders
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

        #Create color mask
        self.binary_image = cv2.inRange(self.cv_image, (self.blue_lb,self.green_lb,self.red_lb), (self.blue_ub,self.green_ub,self.red_ub))

        #Conditional for storing time stamp
        if self.update:
            #Only store time stamp if the frame updates to the window
            self.time_stamp = msg.header.stamp.to_sec()
            self.update = False


    def blue_checker(self, circle):
        """ checks each circle to see how blue it is at several pixels within the circle.
            NO LONGER USE THIS FUNCTION"""

        #Initialize range and return variables
        thetas = (0, 45, 90, 135, 180, 125,270, 315)
        r = 0
        b = 0

        #Loop through angles and add blue and red values at 3/4 radius
        for t in thetas:
            #Get coordinates
            y_cord = int(.75*circle[2]*math.sin(math.radians(t))+circle[0])
            x_cord = int(.75*circle[2]*math.cos(math.radians(t))+circle[1])
            #If not outside bound of image
            if x_cord < self.cv_image.shape[1] and y_cord < self.cv_image.shape[0]:
                #sum blue and red vals
                b += self.cv_image[y_cord, x_cord, 0]
                r += self.cv_image[y_cord, x_cord, 2]

        return r,b

    def mask_checker(self, circle):
        """ checks each circle to see how masked it is at several pixels within the circle
            Same as blue_checker, but loop through radiuses as well
            Only sums binary values
            NO LONGER USE THIS FUNCTION."""

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

    def bestCircleCalc(self):
        """Function sets circle that contains the most white pixels"""

        current_max = 0
        bestCircleIndex = 0

        #Loop through created circle-masks
        for i, mask in enumerate(self.maskList):
            #Calculate moments of mask
            moment = cv2.moments(mask)

            #Check if number of pixels is bigger than last max
            if moment['m00'] > current_max:
                #Set new max
                current_max = moment['m00']
                bestCircleIndex = i

        #Set bestCircle attribute to store the best circle
        self.bestCircle = self.circles[0, bestCircleIndex]


    def predictor(self, circle_prev, circle_current):
        """Calculates current position and velocity of ball
            Takes previous and current position of ball
            Returns X, Y, X Velocity, Y Velocity"""


        # finds distance of previous (1) and current (2) circles
        
        #Coefficients for best fit Function
        #y = a*exp(b*x)
        a = 221.2
        b = -0.3178

        #stores radii of current and previous circles
        d1 = circle_prev[2]
        d2 = circle_current[2]

        #Checks if time stamp has changed since last run
        if self.last_time != self.time_stamp:
            #set new delta T
            self.delta_t = self.last_time - self.time_stamp

        #If the previous circle isn't 0
        if not d1 == 0:

            #Calculate foot to pixel ratio for each circle position
            ft_px1 = 7 / (24 * d1)
            ft_px2 = 7 / (24 * d2)

            #Calculate y depth
            y_dist_1 = math.log((d1 / a)) / b
            y_dist_2 = math.log((d2 / a)) / b

            #Calculate "middle" of camera
            size = (self.cv_image.shape[0] / 2) + 90
            #Calculate x depth
            x_dist_1 = (circle_prev[0] - size) * ft_px1
            x_dist_2 = (circle_current[0] - size) * ft_px2

            #Based on last 2 depths, calculate x and y velocity
            y_vel = (y_dist_2 - y_dist_1) / self.delta_t # gives speed in ft/s
            x_vel = (x_dist_2 - x_dist_1) / self.delta_t

            #Return current ball position and velocity
            return x_dist_2, y_dist_2, y_vel, x_vel
        else:
            #if last circle is 0, return 0
            return 0, 0, 0, 0



    def vector_finder(self, vx, vy):
        """Converts velocity from cartesian to polar
            Takes x and y velocity
            Returns angle and magnitude of velocity"""

        #If y changed
        if vy != 0:
            #Calc angle and magnitude
            angle = math.atan(vx/vy)
            mag = math.hypot(vx,vy)
            return angle, mag
        else:
            return 0, 0

    def visualize_obstacle(self, dy, dx, vy, vx, mag):
        """ 
        visualizes the magnitude and velocity of the vector
        dy = y cord of ball
        dx = x cord of ball
        vy = y vel of ball
        vx = x vel of ball
        mag = magnitude of vector
        returns an arrow vector
        """
        ft_to_m = 0.3 # rviz uses meters

        # prepping
        vel_marker = Marker(type=Marker.ARROW)
        vel_marker.header.frame_id = "base_link"

        # creating the start point
        start_point = Point()
        start_point.x = dx * ft_to_m
        start_point.y = dy * ft_to_m

        # creating the end point
        end_point = Point()
        end_point.x = vx * ft_to_m
        end_point.y = vy * ft_to_m
        vel_marker.points = [start_point, end_point]

        # scaling the length of the arrow by the magnitude
        vel_marker.scale.y = mag * ft_to_m
        vel_marker.scale.x = .2
        vel_marker.color.a = 1

        # publishing
        self.visualizer.publish(vel_marker)

    def run(self):
        """ The main run loop"""

        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            #Create copies of images for manipulation
            self.all_circles = copy.deepcopy(self.cv_image)
            cvImg = copy.deepcopy(self.cv_image)
            binImg = copy.deepcopy(self.binary_image)

            #If we have a camera image
            if not cvImg is None:

                #apply blur to binary
                binImg= cv2.medianBlur(binImg, 5)

                #Detect Circles
                self.circles = cv2.HoughCircles(cv2.cvtColor(cvImg, cv2.COLOR_BGR2GRAY),self.houghGrad,1,100,
                                    param1=70,param2=30,minRadius=self.min_radius_1,maxRadius=self.max_radius_1)

                bestFitCircle = [0, 0, 0]
                self.maskList = []
                #If circles exist, calculate their masks on binary
                if not (self.circles == None):
                    current_max = 0

                    #loop through circles
                    for q in self.circles[0, :]:
                        # draw the circle on all_circles
                        cv2.circle(self.all_circles, (q[0], q[1]), q[2], (0,255,0), 2)
                        #Draw the center of the circle on all_circles
                        cv2.circle(self.all_circles, (q[0], q[1]), 2, (0,0,255), 3)

                        #Create mask
                        mask = np.full((self.all_circles.shape[0], self.all_circles.shape[1]), 0, dtype=np.uint8)
                        cv2.circle(mask, (q[0], q[1]), q[2], 255, -1)

                        #Create return mask
                        temp = np.full((self.all_circles.shape[0], self.all_circles.shape[1]), 0, dtype=np.uint8)

                        #Mask with binary image
                        cv2.bitwise_and(mask, binImg, temp)

                        #Append to the list of masks
                        self.maskList.append(temp)

                    #From list of masks, calculate the best circle
                    self.bestCircleCalc()

                    #Draw best circle on cvImg
                    if not self.cv_image == None:
                        #Draw Circle
                        cv2.circle(cvImg,(self.bestCircle[0], self.bestCircle[1]), self.bestCircle[2],(0,255,0),2)
                        #Draw Circle Center
                        cv2.circle(cvImg,(self.bestCircle[0], self.bestCircle[1]),2,(0,0,255),3)




                #Display images
                if not self.all_circles is None:
                    cv2.imshow('video_window2', self.all_circles)
                if not self.cv_image is None:
                    cv2.imshow('video_window', cvImg)
                if not self.binary_image is None:
                    cv2.imshow('mask', binImg)

                #If last two circles were different
                if self.lastCircle[0] != self.bestCircle[0]:

                    #Predict position and velocity
                    xCoord, yCoord, vy, vx = self.predictor(self.lastCircle, self.bestCircle)
                    print('vy',vy)
                    print('vx', vx)
                    ang, mag = self.vector_finder(vx, vy)
                    print('ang', ang)
                    print('mag', mag)

                    self.visualize_obstacle(yCoord, xCoord, vy, vx, mag)

                #Set last circle to current
                self.lastCircle = self.bestCircle
                #Set last timestamp to current
                self.last_time = self.time_stamp
                cv2.waitKey(5)
                #self.pub.publish(self.cmd_vel)

                self.update = True
                # start out not issuing any motor commands
            r.sleep()


if __name__ == '__main__':
    node = Keeper("/camera/image_raw")
    node.run()
