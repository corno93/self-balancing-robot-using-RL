#!/usr/bin/env python

import rospy
from robot_teleop_tuner.msg import pid_values 

if __name__ == '__main__':
    rospy.init_node('pid_tuner')
    pub = rospy.Publisher('pid_tuner',pid_values ,queue_size=10)
    rate = rospy.Rate(10)
    msg = pid_values()

    while not rospy.is_shutdown():
	try:
	    term = raw_input("Enter term id: p, i, d, pv: ")
	    value = raw_input("Enter value: ")
	    term = term.strip()
 	    if term == 'p':
	  	    msg.p = float(value)
	    elif term == 'i':
		    msg.i = float(value)
	    elif term == 'd':
		    msg.d = float(value)
	    elif term == 'pv':
		    msg.pv = float(value)
	    else:
		print "incorrect entered variable.."

	except KeyboardInterrupt:
	   print "for fuck sake mate its not that hard..."
	   break

	print "You have selected %s at %s" % (term, value)
	pub.publish(msg)
	rate.sleep()
