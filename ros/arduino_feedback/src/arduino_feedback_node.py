#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
from arduino_feedback.msg import feedback

if __name__ == '__main__':

    try:
        ardS = serial.Serial("/dev/ttyACM0", 9600)
    except:
        rospy.loginfo("Unable to connect to port ttyACM0")
   
#    msg = PidData()
    pub = rospy.Publisher('arduino_feedback', String, queue_size=10)
    rospy.init_node('arduino_feedback_node', anonymous=True)
    rate = rospy.Rate(10) 
    rpm_data1 = []
    msg = feedback()
    while not rospy.is_shutdown():
	try:
	  #  read_serial = str(int(ardS.readline(), 16))
            read_serial = ardS.readline()
#            msg.rpm1 = read_serial
                       
            msg.rpm1 = str(5)
	    msg.rpm2 = 1.234
	    hello_str = "hey"
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()
	except:
	    rospy.loginfo("Unable to readline from arduino")
            print rpm_data1



 




