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
	if msg.status == "Free":
		busy = False
	print 'Status: ', msg.status	
stat_sub = rospy.Subscriber('/PW/status',Status,stat_cb)
pub = rospy.Publisher('/DW_control',String)
def main():
	global busy
	comm = raw_input('enter command: ')
	if not busy:
		pub.publish(String(comm))
	else:
		print "busy..."
if __name__ == '__main__':
	while True:
		try:
			main()
		except KeyboardInterrupt:
			print 'bye'
			break