#! /usr/bin/env python

import time
import rospy
import actionlib
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
from myrosject.srv import FindWall, FindWallRequest
from myrosject.msg import OdomRecordAction, OdomRecordGoal


class Follow_Wall():

    def __init__(self):
        # create the connection to the action server
        self.action_client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
        # waits until the action server is up and running
        self.action_client.wait_for_server()
        # creates a goal to send to the action server
        action_goal = OdomRecordGoal()
        # sends the goal to the action server, specifying which feedback function
        # to call when feedback received
        self.action_client.send_goal(action_goal)

        # Wait for the service client /find_wall to be running
        rospy.wait_for_service('/find_wall')
        # Create the connection to the service
        find_wall_service = rospy.ServiceProxy('/find_wall', FindWall)
        print("Connection to service /find_wall done")
        # Create an object of type FindWallRequest
        find_wall_object = FindWallRequest()
        # Send through the connection the object of type FindWallRequest
        result = find_wall_service(find_wall_object)
        # Print the result given by the service called
        print("result is",result)

        # Start publisher and subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move = Twist()
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.rate = rospy.Rate(1)
        self.move.linear.x = 0
        self.move.angular.z = 0

        # initiate a list of 720 values
        self.laser = [10.0] * 720  
        # start wall following
        self.run = True
        self.moveWall()

    def callback(self,data): 
        # Get Laser Data
        self.laser = data.ranges
        time.sleep(0.1)
    
    def moveWall(self):
        while self.run == True:
            # ray_angle = data.angle_min + (0 * data.angle_increment)
            # Navigate along side the wall
            if self.laser[180] > 0.3 and self.laser[360] > 0.5 and self.laser[360] != float('inf') and self.laser[180] != float('inf'):
                # if right laser is bigger than 0.3 turn right closer to the wall
                self.move.angular.z = -0.11
                self.move.linear.x = 0.08
            elif self.laser[180] < 0.2 and self.laser[360] > 0.5 and self.laser[360] != float('inf') or self.laser[180] == float('inf'):
                # if right laser is smaller than 0.2 turn left away from the wall
                self.move.angular.z = 0.2
                self.move.linear.x = 0.08
            elif self.laser[360] < 0.5 and self.laser[360] != float('inf') and self.laser[180] != float('inf'):
                # if front beam is smaller than 0.5 turn fast left
                self.move.angular.z = 0.35
                self.move.linear.x = 0.08
            elif self.laser[360] == float('inf'):
                # Either laservalues are not good or robot is in front of the wall
                print("Front Laser 360 value is infinite!")
                i = 0
                front_free = False
                for i in range(350,370):
                    print("check laser value",i)
                    laser_front_value = self.laser[i]
                    if laser_front_value != float('inf'):
                        print("laser value index",i,"is good")
                        front_free = True
                if front_free == True:
                    print("robot can move forward, slowly")
                    self.move.angular.z = 0.08
                    self.move.linear.x =  0.04
                else:
                    while self.laser[360] == float('inf'):
                        print("robot has either a wall in front or laser values are not good for example sunlight")
                        timer = 0
                        while timer < 4:
                            self.move.angular.z = 0
                            self.move.linear.x =  -0.08
                            self.pub.publish(self.move)
                            time.sleep(1)
                            timer = timer + 1
                            print("go back",timer,"seconds")
                        timer = 0
                        while timer < 8:
                            self.move.angular.z = 0.15
                            self.move.linear.x =  0
                            self.pub.publish(self.move)
                            time.sleep(1)
                            timer = timer + 1
                            print("turn left",timer,"seconds")
                        #stop robot and check again the position
                        self.move.angular.z = 0
                        self.move.linear.x =  0
                        self.pub.publish(self.move)
                        time.sleep(2)                
            else :
                # move just forward
                self.move.angular.z = 0
                self.move.linear.x = 0.08
    
            # Close the node when one lap is finished   
            action_result = self.action_client.get_result()
            if action_result != None:
                self.run = False
                rospy.signal_shutdown("shutdown time")

            print("Right Laser:",self.laser[180],"Front Laser",self.laser[360]) 
            print("linear speed:",self.move.linear.x,"angular speed:",self.move.angular.z)
            self.pub.publish(self.move) 
            time.sleep(0.1)


def main():
    rospy.init_node('follow_wall_node')
    Follow_Wall()
    rospy.spin()

if __name__ == '__main__':
    main()
