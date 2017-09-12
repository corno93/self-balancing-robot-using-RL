#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
from arduino_feedback.msg import feedback

if __name__ == '__main__':

    try:
        ardS = serial.Serial("/dev/ttyACM0", 38400)
    except:
        rospy.loginfo("Unable to connect to port ttyACM0")
   
#    msg = PidData()
    pub = rospy.Publisher('arduino_feedback', feedback, queue_size=10)
    rospy.init_node('arduino_feedback_node', anonymous=True)
    rate = rospy.Rate(10) 
    rpm_data1 = []
    msg = feedback()
	
    while not rospy.is_shutdown():

	try:
	  #  read_serial = str(int(ardS.readline(), 16))
            data_recieved = ardS.readline()
#            msg.rpm1 = read_serial
            if data_recieved[:2] == 'R1':
                msg.rpm1 = float(data_recieved[2:])
            elif data_recieved[:2] == 'R2':
                msg.rpm2 = float(data_recieved[2:])
           
            rospy.loginfo(data_recieved)
            pub.publish(msg)
            rate.sleep()
	except:
	    rospy.loginfo("Unable to readline from arduino")



 




