#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts



class AddIntes:
    def __init__(self):
        rospy.init_node('my_node')
        print('in main')
        rate=rospy.Rate(10)
        rospy.Subscriber('two_ints',TwoInts, self.cb)
        self.pub=rospy.Publisher('sum',Int16,queue_size=10)
        while not rospy.is_shutdown():
            #rospy.spin()
            #self.pub.publish(self.my_sum)
            #print(self.pub)
            rate.sleep()
        
    def cb(self,data):
        self.my_sum=data.a+data.b
        self.pub.publish(self.my_sum)
        print(self.my_sum)
        rospy.sleep(0.1)

if __name__=='__main__':
    print('yaha')
    AddIntes()