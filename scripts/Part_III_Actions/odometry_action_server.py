#! /usr/bin/env python
import math
import rospy
import actionlib
from myrosject.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32

class Odometry_Record():

    def __init__(self):
        #Init Values
        self.odom = Odometry()
        self.rate = rospy.Rate(1)
        self.ctrl_c = False
        #Create Subscriber
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.sub_odom_callback)
        rospy.loginfo("subscriber /odom is ready")
        #Create Action Server
        self._feedback = OdomRecordFeedback()
        self._result = OdomRecordResult()
        #Result is empty at the beginning 
        self._result.list_of_odoms = []
        self._as = actionlib.SimpleActionServer("record_odom", OdomRecordAction, self.action_callback, False)
        self._as.start()
        rospy.loginfo('action server /record_odom is ready')
        
    def sub_odom_callback(self, msg):
        self.odom = msg

    def action_callback(self, goal):
        #Init local variable list of odometry with zeros for distance calculation
        list_of_odoms = [[0.0,0.0,0.0]]
        
        success = True

        i = 0
        distance = 0.0

        while distance < 5.5: # 1 Round is approximately 5.5 Meters

            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                self._as.set_preempted()
                success = False
                break

            #Add the next recording
            pos_x = self.odom.pose.pose.position.x
            pos_y = self.odom.pose.pose.position.y
            ori_z = self.odom.pose.pose.orientation.z
            list_of_odoms.append([pos_x,pos_y,ori_z])

            #Get point32() structure
            odom_record = Point32()
            odom_record.x = pos_x
            odom_record.y = pos_y
            odom_record.z = ori_z
            self._result.list_of_odoms.append(odom_record)
            
            #Calculate distance traveled and give feedback
            i += 1

            if i > 1: 
                delta_distance = math.sqrt((math.pow(list_of_odoms[i][0] -list_of_odoms[i-1][0],2)) +
                    (math.pow(list_of_odoms[i][1] -list_of_odoms[i-1][1],2)))
                distance = distance + delta_distance
                self._feedback.current_total = distance
                self._as.publish_feedback(self._feedback)
                # print("Current distance is "+str(self._feedback.current_total))

            self.rate.sleep()

        if success:
            rospy.loginfo("One lap done successfully, list of odoms is:")
            self._as.set_succeeded(self._result)
            print(self._result.list_of_odoms)

if __name__ == '__main__':
    rospy.init_node('record_odom_node')
    Odometry_Record()
    rospy.spin()
