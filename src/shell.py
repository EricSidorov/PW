#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from PW.msg import Status
busy = False
rospy.init_node('PW_shell')
def stat_cb(msg):
	global busy
	if msg.status == "Busy":
		busy = True
		print 'Status: ', msg.status
	if msg.status == "Free":
		busy = False
		print 'Status: ', msg.status, '\n'
		comm = raw_input('enter command: ')
		pub.publish(String(comm))

stat_sub = rospy.Subscriber('/PW/status',Status,stat_cb)
pub = rospy.Publisher('/DW_control',String)

if __name__ == '__main__':
	comm = raw_input('enter command: ')
	pub.publish(String(comm))

	rospy.spin()