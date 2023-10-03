#! /usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from myrosject.srv import FindWall, FindWallResponse

class Find_Nearest_Wall():
    def __init__(self):
        # initiate a list of 720 values
        self.laser = [10.0] * 720  
        # start publisher, subscriber and server
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move = Twist()
        self.sub = rospy.Subscriber('/scan', LaserScan, self.sub_callback)
        self.rate = rospy.Rate(1)
        self.my_service = rospy.Service("/find_wall", FindWall, self.server_callback)
        rospy.loginfo("The service: /find_wall is ready")
        
    def sub_callback(self, data):
        # get laser data
        self.laser = data.ranges

    def server_callback(self, request):
        print("server_callback started with {",request,"}")
        while self.laser[0] == 10: # Wait until self.laser gets realistic values
            rospy.loginfo("laser values not ready yet")
            self.rate.sleep()
        #find min laser and direction of rotation
        [min_laser,dir_of_rot] = self.find_min_laser()
        #set timer
        timer = 0
        #rotate
        while self.laser[360] > min_laser + 0.06: # tolerance
            self.move.angular.z = dir_of_rot*0.15
            self.pub.publish(self.move)
            timer = timer + 1
            print("Robot is rotating",timer,"seconds")
            if timer > 60: #timer if it rotates longer, then something is not good
                print("robot could not find wall, will try again")
                #find min laser and direction of rotation again
                [min_laser,dir_of_rot] = self.find_min_laser()
                timer = 0
            self.rate.sleep()
        self.move.angular.z = 0
        self.pub.publish(self.move)
        print("Rotation done, now moving to the wall.")

        # frontbeam is the smallest one and we can move forward to the wall
        while self.laser[360] > 0.3 and self.laser[360] != float('inf'):
            self.move.linear.x = 0.08
            self.pub.publish(self.move)
            self.rate.sleep()
        self.move.linear.x = 0
        self.pub.publish(self.move)
        print("We are at the wall")

        # rotate 90 degree (180rays) to the left be along the wall
        distance_front = self.laser[360]
        print("Current front distance to wall is",distance_front,"and current right laser is",self.laser[180])
        while self.laser[170] < distance_front - 0.025 or self.laser[170] > distance_front + 0.025: #tolerance
            self.move.angular.z = 0.15
            self.pub.publish(self.move)
            self.rate.sleep()
        self.move.angular.z = 0
        self.pub.publish(self.move)
        print("Wall is now to the right side")

        # return message
        my_response = FindWallResponse()
        my_response.wallfound = True
        return my_response

    def find_min_laser(self):
        # init values
        i=0
        min_laser = float('inf')
        min_index = 0
        # get robot standing still
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pub.publish(self.move)
        # loop through all laserbeams to find the smallest
        for i in range(0,len(self.laser)):
            if self.laser[i] < min_laser:
                min_laser = self.laser[i]
                min_index = i
        print("Lowest laser value is",min_laser,"at the Index",min_index,"and front laser has value",self.laser[360])

        # rotate to make frontbeam the smallest one
        # convert angle to radiant
        # 360°/720rays = 0.5°/ray, 1° = pi/180 = 0.01745 rad
        # 0.5/ray * 0.01745 rad = 0.00872 rad/ray
        # Get difference of ray beams to the middle beam
        diff_ray_index = 360 - min_index
        print("Difference of ray beams to rotate",diff_ray_index)
        if diff_ray_index > 0:
            dir_of_rot = -1 #rotate to the right
            print("rotate to the right")
        else:
            dir_of_rot = 1 #rotate to the left
            print("rotate to the left")

        return [min_laser,dir_of_rot]

def main():
    rospy.init_node("find_nearest_wall_service_server_node")
    Find_Nearest_Wall()
    rospy.spin()

if __name__ == '__main__':
    main()