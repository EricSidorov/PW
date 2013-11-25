#! /usr/bin/env python
from DogWormVRC3 import *
import rospy
import sys
if __name__=='__main__':
    print len(sys.argv)
    if len(sys.argv)==2:
        if sys.argv[1] == 'bwd' or sys.argv[1] == 'fwd':
            crwl_dir = sys.argv[1]
    else:
        crwl_dir = 'fwd'
    print 'crawling', crwl_dir
    rospy.init_node("DW_test")
    # rospy.sleep(0.5)
    iTF = Interface_tf()
    DW = DW_Controller(iTF)
    DW.Initialize(Terrain = "HILLS") 
    DW.Throttle = {'FWD': 1.0, 'BWD': 2.0, 'FROT': 1., 'SROT': 1.}
    if crwl_dir == 'fwd':
        DW.gait_params = {'Slope':0.0,'LegSpread':0.25,'PelvisHeight':0.25,'FRotKnee':0.7,'MaxRecover':1}
    else:
        if crwl_dir == 'bwd':
            DW.gait_params = {'Slope':0.0,'LegSpread':0.75,'PelvisHeight':0.0,'FRotKnee':0.7,'MaxRecover':1}  
    DW.LoadPoses()
    # rospy.Subscriber('/C25/publish',C25C0_ROP,DW.Odom_cb)
    odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,DW.Odom_cb)

    # Subscribe to topic controller
    rospy.Subscriber('/DW_control',String,DW.Interface_cb)

    rospy.Subscriber('/atlas/atlas_state',AtlasState,DW.RS_cb)
    rospy.sleep(0.1)
    DW.send_pos_traj(DW.RS.GetJointPos(),array(DW.RS.GetJointPos()),0.1,0.005)
    DW.CloseHands()
    rospy.sleep(0.1)

    signal.signal(signal.SIGALRM, DW.Alarm)
    point = [6,6,crwl_dir]
    for i in xrange(1,10):
        DW.Sit(1.5)
        DW.reset_srv()
        rospy.sleep(2)
        DW.GoToPoint(point)