#! /usr/bin/env python

##################################################################
##################################################################
# File modified by: JONATHAN SPITZ
# Last modified on: 28/07/2013
# Modifications:
#  # Added ability to command the robot via topic /DW_control
#  # Created function LoadPoses to allow for quicker development
##################################################################
##################################################################

import roslib
roslib.load_manifest('PW')
#import roslib; roslib.load_manifest('DogWorm')
import math, rospy, os, rosparam
from seq_generator import PW_seq
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from numpy import zeros, array, linspace, arange
import numpy as np
from JointController import JointCommands_msg_handler
from JointController import hand_joint_controller
from robot_state import robot_state
from PW.msg import Status
from atlas_msgs.msg import AtlasState
from math import ceil
import yaml
from copy import copy
from std_srvs.srv import Empty
##nedded for tests
from IMU_monitoring import IMUCh
# from Abstractions.Interface_tf import *
from std_msgs.msg import String


class Interface_tf(object):
    def __init__(self):
        self._TransformListener = tf.TransformListener()
        self._TransformBroadcaster = tf.TransformBroadcaster()
 
    def TransformListener(self):
        return self._TransformListener
    
    def TransformBroadcaster(self):
        return self._TransformBroadcaster


class DW_Controller(object):
    """DW_Controller"""
    def __init__(self,iTf):
        super(DW_Controller, self).__init__()
        self._iTf = iTf

        self.LoadPoses()


    ##################################################################
    ########################## INITIALIZE ############################
    ##################################################################

    def Initialize(self,Terrain):
        self._robot_name = "atlas"
        self._jnt_names = ['back_lbz', 'back_mby', 'back_ubx', 'neck_ay', #3
                           'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', #9
                           'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax', #15
                           'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', #21
                           'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'] #27

        # Initialize joint commands handler
        self.JC = JointCommands_msg_handler(self._robot_name,self._jnt_names)
        self.LHC = hand_joint_controller("left")
        self.RHC = hand_joint_controller("right")
        # Initialize robot state listener
        self.RS = robot_state(self._jnt_names)
        self.IMU_mon = IMUCh()

        print("PW::Initialize")
        self.GlobalPos = 0
        self.GlobalOri = 0
        self.HeadPitch = 0
        self._counter = 0
        self._terrain = Terrain
        self.RotMode = 2
        self._fall_count = 0
        self.FALL_LIMIT = 3
        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)
        self._stat_pub = rospy.Publisher('/PW/status',Status)
        self._rpy_pub = rospy.Publisher('/PW_rpy',Vector3)

        # Commands
        self.Commands = [['sit','Sit down from standing position'],
                         ['fwd [n]','Crawl forward [n] times'],
                         ['bwd [n]','Crawl backwards [n] times'],
                         ['turn [ori]','Turn in place to desired [ori]entation (in radians)'],
                         ['rot type [fast/slow]','Set rotation type to slow or fast'],
                         ['rot [delta]','Rotate in place once with strength [delta] (+/- 1)'],
                         ['recover','Recover from tipping'],
                         ['stand','Stand up from sitting position'],
                         ['reset pose','Resets the robot\'s configuration to the default standing pose'],
                         ['reset','Resets the robot\'s pose and returns it to its starting position'],
                         ['reload','Reloads the sit down, fwd, and bwd sequences parameters from seq_generator.py'],
                         ['seq [fwd/bwd] [step]','Brings the robot to step [step] of the [fwd/bwd] sequence'],
                         ['close hands','Closes the robot\'s hands'],
                         ['head [up/down] [rad]','Tilts the robot\'s head [up/down] by [rad] radians'],
                         ['befb [0/1]','Turns the bearing feedback response on [1] or off [0]'],
                         ['bedes [rad]','Sets the desired bearing for the bearing FB to [rad] radians from the x axis'],
                         ['terrain [MUD/HILLS/OTHER]','Sets the terrain that the robot is moving on'],
                         ['status','Shows the status of all the system\'s flags'],
                         ['commands','Shows the list of all available commands'],
                         ['help [command]','Provides help on using a command'],
                         ['exit','Exit this console']]

        ##################################################################
        ######################## Controller Gains ########################
        ##################################################################

        self.JC.set_gains('l_leg_uhz',1000,0,10)
        self.JC.set_gains('r_leg_uhz',1000,0,10)
        self.JC.set_gains('l_leg_mhx',1000,0,10)
        self.JC.set_gains('r_leg_mhx',1000,0,10)
        # self.JC.set_gains('l_leg_kny',100,0,10)
        # self.JC.set_gains('r_leg_kny',100,0,10)
        # self.JC.set_gains('l_leg_lhy',100,0,10)
        # self.JC.set_gains('r_leg_lhy',100,0,10)
        self.JC.set_gains('back_lbz',5000,0,10)
        self.JC.set_gains('back_ubx',2000,0,10)
        self.JC.set_gains('l_arm_usy',1000,0,10)
        self.JC.set_gains('r_arm_usy',1000,0,10)
        self.JC.set_gains('l_arm_shx',1000,0,10)
        self.JC.set_gains('r_arm_shx',1000,0,10)
        self.JC.set_gains('l_arm_ely',2000,0,10)
        self.JC.set_gains('r_arm_ely',2000,0,10)
        self.JC.set_gains("l_arm_elx",1200,0,5)
        self.JC.set_gains("r_arm_elx",1200,0,5)
        self.JC.set_gains("l_arm_mwx",1200,0,5)
        self.JC.set_gains("r_arm_mwx",1200,0,5)

    ##################################################################
    ########################### FUNCTIONS ############################
    ##################################################################

    def Interface_cb(self,msg):
        CommString = msg.data
        print("COMMAND RECEIVED: %s" % CommString)
        MotionType = 0
        Parameters = []

        Args = []
        while CommString.find(',') != -1:
            Part = CommString.partition(', ')
            Args.append(Part[0])
            CommString = Part[2]
        Args.append(CommString)

        for Command in Args:
            if Command.find(self.Commands[0][0]) == 0: ################ SIT ################
                MotionType = 1
                Parameters.append(MotionType)

            String = self.Commands[1][0].partition("[")[0]
            if Command.find(String) == 0: ################ FWD ################
                MotionType = 2
                Parameters.append(MotionType)
                CommParted = Command.partition(String)
                NumSteps = int(CommParted[2])
                Parameters.append(NumSteps)

            String = self.Commands[2][0].partition("[")[0]
            if Command.find(String) == 0: ################ BWD ################
                MotionType = 3
                Parameters.append(MotionType)
                CommParted = Command.partition(String)
                NumSteps = int(CommParted[2])
                Parameters.append(NumSteps)

            String = self.Commands[3][0].partition("[")[0]
            if Command.find(String) == 0: ################ TURN ################
                MotionType = 4
                Parameters.append(MotionType)
                CommParted = Command.partition(String)
                Bearing = float(CommParted[2])
                Parameters.append(Bearing)

            String = self.Commands[4][0].partition(" ")[0]
            if Command.find(String) == 0: ################ ROT ################
                CommParted = Command.partition(String+" ")
                if CommParted[2].find("type") == 0:
                    CommParted2 = CommParted[2].partition(" ")
                    if CommParted2[2].find("fast") == 0:
                        self.RotMode = 1
                        print "Rotation mode set to fast"
                    if CommParted2[2].find("slow") == 0:
                        self.RotMode = 2
                        print "Rotation mode set to slow"
                    MotionType = -1
                else:
                    MotionType = 5
                    Delta = float(CommParted[2])
                    Parameters.append(MotionType)
                    Parameters.append(Delta)

            String = self.Commands[6][0]
            if Command.find(String) == 0: ################ RECOVER ################
                MotionType = 6
                Parameters.append(MotionType)

            String = self.Commands[7][0]
            if Command.find(String) == 0: ################ STAND ################
                MotionType = 7
                Parameters.append(MotionType)

            if Command.find(self.Commands[9][0]) == 0: ################ RESET ###############
                if Command.find(self.Commands[8][0]) > 0: ########## RESET POSE #############
                    MotionType = 8
                else:
                    MotionType = 9
                Parameters.append(MotionType)

            if Command.find(self.Commands[10][0]) == 0:
                MotionType = 10
                Parameters.append(MotionType)

            String = self.Commands[11][0].partition("[")[0]
            if Command.find(String) == 0: ################ SEQ ################
                MotionType = 11
                Parameters.append(MotionType)
                CommParted = Command.partition(String)
                CommParted2 = CommParted[2].partition(" ")
                Sequence = CommParted2[0]
                SeqStep = int(CommParted2[2])
                Parameters.append(Sequence)
                Parameters.append(SeqStep)

            if Command.find(self.Commands[12][0]) == 0: ########### CLOSE HANDS ###########
                MotionType = 12
                Parameters.append(MotionType)

            String = self.Commands[13][0].partition("[")[0]
            if Command.find(String) == 0: ################ HEAD ################
                MotionType = -1
                CommParted = Command.partition(String)
                CommParted2 = CommParted[2].partition(" ")
                if CommParted2[0].find("up"):
                    self.HeadPitch = self.HeadPitch + float(CommParted2[2])
                if CommParted2[0].find("down"):
                    self.HeadPitch = self.HeadPitch - float(CommParted2[2])
                self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[-1],0.5,0.005)

            String = self.Commands[14][0].partition("[")[0]
            if Command.find(String) == 0: ################ BEFB ################
                MotionType = -1
                CommParted = Command.partition(String)
                self.FollowPath = int(CommParted[2])
                if self.FollowPath == 0:
                    self.AddRotation(0)
                    self.AddBackRotation(0)
                    print "Bearing feedback is now OFF"
                else:
                    print "Bearing feedback is now ON"

            String = self.Commands[15][0].partition("[")[0]
            if Command.find(String) == 0: ################ BEDES ################
                MotionType = -1
                CommParted = Command.partition(String)
                self.DesOri = float(CommParted[2])
                print("Desired bearing set to: %f" % self.DesOri)

            String = self.Commands[16][0].partition("[")[0]
            if Command.find(String) == 0: ################ TERRAIN ################
                MotionType = -1
                CommParted = Command.partition(String)
                self._terrain = CommParted[2]
                print("Terrain type set to: %s" % self._terrain)

            if Command.find(self.Commands[17][0]) == 0: ########### STATUS ###########
                MotionType = -1
                if self.FollowPath == 0:
                    print "Bearing feedback is now OFF"
                else:
                    print "Bearing feedback is now ON"
                    print("Desired bearing set to: %f" % self.DesOri)
                print("Terrain type set to: %s" % self._terrain)
                if self.RotMode == 1:
                    print "Rotation mode set to fast"
                else:
                    print "Rotation mode set to slow"

            if Command.find(self.Commands[18][0]) == 0: ########### COMMANDS ###########
                MotionType = -1
                print "Available commands:"
                com_string = ""
                for com in self.Commands:
                    com_string += com[0]+", "
                print com_string[0:-2]

            String = self.Commands[19][0].partition("[")[0]
            if Command.find(String) == 0: ########### HELP ###########
                MotionType = -1
                CommParted = Command.partition(String)
                for com in self.Commands:
                    if com[0].find(CommParted[2])>=0:
                        print com[0]+" - "+com[1] 
            
        if MotionType == 0:
            print("Got no command param, aborting...")
        elif MotionType < 0:
            print " "
        else:
            self._stat_pub.publish(Status("Busy"))
            self.DoTask(Parameters)


    def DoTask(self, Parameters):
        MotionType = Parameters[0]

        rospy.sleep(0.1)

        if MotionType == 1: ################ SIT ################
            print("Sitting down...")
            self.Sit(1.5)
            print("SUCCESS!!\n")

        elif MotionType == 2: ################ FWD ################
            NumSteps = Parameters[1]
            print("Crawling FWD %d steps..." % NumSteps)
            for x in range(NumSteps):
                self.Crawl()
            print("SUCCESS!!\n")

        elif MotionType == 3: ################ BWD ################
            NumSteps = Parameters[1]
            print("Crawling BWD %d steps..." % NumSteps)
            for x in range(NumSteps):
                self.BackCrawl()
            print("SUCCESS!!\n")

        elif MotionType == 4: ################ TURN ################
            Bearing = Parameters[1]
            print("Rotating to a bearig of %f radians..." % Bearing)
            if self.RotateToOri(Bearing):
                print("SUCCESS!!\n")
            else:
                print("FAIL\n")

        elif MotionType == 5: ########### SINGLE ROTATE ############
            Delta = Parameters[1]
            if self.RotMode == 1:
                print("Rotating fast in place %0.0f%%" % (Delta*100))
                y0,p,r = self.current_ypr()
                self.RotSpotSeq(Delta)
                y,p,r = self.current_ypr()
                print("Rotated %f radians" % (self.DeltaAngle(y,y0)))                
            else:
                print("Rotating slowly in place %0.0f%%" % (Delta*100))
                y0,p,r = self.current_ypr()
                self.RotOnMudSeq(Delta)
                y,p,r = self.current_ypr()
                print("Rotated %f radians" % (self.DeltaAngle(y,y0)))    
            print("SUCCESS!!\n")

        elif MotionType == 6: ################ RECOVER ################
            print("Recovering after tipping...")
            if self.CheckTipping():
                print("SUCCESS!!\n")
            else:
                print("FAIL\n")

        elif MotionType == 7: ################ STANDUP ################
            print("Standing up...")
            self.StandUp()
            print("SUCCESS!!\n")

        elif MotionType == 8: ################ RESET (POSE) ################
            print("Resetting...")
            self.LoadPoses()
            self.JC.reset_gains()
            self.send_pos_traj(self.RS.GetJointPos(),self.BasStndPose,0.5,0.005)
            print("SUCCESS!!\n")

        elif MotionType == 9: ################ RESET (FULL) ################
            print("Resetting...")
            self.LoadPoses()
            self.JC.reset_gains()
            self.send_pos_traj(self.RS.GetJointPos(),self.BasStndPose,0.5,0.005)
            rospy.sleep(0.5)
            self.reset()
            print("SUCCESS!!\n")

        elif MotionType == 10: ################ RELOAD ################
            print("Loading new gaits...")
            self.LoadPoses()
            print("SUCCESS!!\n")

        elif MotionType == 11: ################ SEQUENCE ################
            Sequence = Parameters[1]
            SeqStep = Parameters[2]
            print("Going to %s sequence step %d" % (Sequence, SeqStep))
            if Sequence.find("fwd") == 0:
                self.GoToSeqStep(SeqStep)
            if Sequence.find("bwd") == 0:
                self.GoToBackSeqStep(SeqStep)
            print("SUCCESS!!\n")

        elif MotionType == 12: ############# CLOSE HANDS ##############
            print("Closing hands...")
            self.CloseHands()
            print("SUCCESS!!\n")

        self._stat_pub.publish(Status("Free"))
               
    def send_pos_traj(self,pos0,pos1,T,dT):
        pos1[3] = self.HeadPitch
        self.JC.send_pos_traj(pos0,pos1,T,dT)


    def LoadPoses(self):
        execfile("seq_generator.py")
        seq_file = file('seqs.yaml','r')
        seqs = yaml.load(seq_file)
        self.RotFlag = seqs.RotFlag
        self.BasStndPose = seqs.BasStndPose
        self.SitDwnSeq1 = seqs.SitDwnSeq1
        self.SitDwnSeq2 = seqs.SitDwnSeq2
        self.RobotCnfg = seqs.RobotCnfg
        self.RobotCnfg2 = seqs.RobotCnfg2
        self.BaseHandPose = seqs.BaseHandPose
        self.StepDur = seqs.StepDur
        self.StepDur2 = seqs.StepDur2
        self.Throtle = seqs.Throtle
        self.count_tipping = seqs.count_tipping
        self.count_tottal = seqs.count_tottal
        self.BaseHipZ = seqs.BaseHipZ
        self.CurSeqStep = seqs.CurSeqStep
        self.CurSeqStep2 = seqs.CurSeqStep2
        self.DesOri = seqs.DesOri
        self.FollowPath = seqs.FollowPath


    def RS_cb(self,msg):
        self.RS.UpdateState(msg)
        self.IMU_mon.get_contact(msg)
        self.IMU_mon.imu_manipulate(msg)


    def Odom_cb(self,msg):
        if 1000 <= self._counter: 
            # print ("Odom_cb::", self.GlobalPos)
            self._counter = 0
        self._counter += 1
        # self.GlobalPos = msg.pose.pose.pose.position # from C25_GlobalPosition
        # self.GlobalOri = msg.pose.pose.pose.orientation # from C25_GlobalPosition
        self.GlobalPos = msg.pose.pose.position    # from /ground_truth_odom
        self.GlobalOri = msg.pose.pose.orientation # from /ground_truth_odom

        y,p,r = self.current_ypr()
        self._rpy_pub.publish(Vector3(r,p,y))


    def move_neck(self):
        init_pos = self.RS.GetJointPos()
        self.JC.set_all_pos(init_pos)
        pos1 = self.RS.GetJointPos()[3]
        pos2 = 0.2 # 0.0 # 0.4 # Fire hose 0.7 # 
        dt = 0.05;
        N = 50
        for ratio in linspace(0, 1, N):
            interpCommand = (1-ratio)*pos1 + ratio * pos2
            self.JC.set_pos(3,interpCommand)
            self.JC.send_command()
            rospy.sleep(dt)


    def ResetPose(self):
        self.JC.set_all_pos([0]*28)
        self.JC.send_command()


    def reset(self):
        self.reset_srv()
        rospy.sleep(1)

        while self.GlobalPos.z<0.9 or self.GlobalPos.z>1: #or abs(self.GlobalPos.x)>0.5:
        # while self.GlobalPos.z<0.25 or self.GlobalPos.z>0.4: #or abs(self.GlobalPos.x)>0.5:
            self.reset_srv()
            rospy.sleep(1)


    def current_ypr(self):
        quat = copy(self.GlobalOri)
        (r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return (y,p,r)
        # RPY = self.RS.GetIMU()
        # return (RPY[2], RPY[1], RPY[0])


    def DeltaAngle(self,DesAngle,CurAngle):
        Delta = DesAngle - CurAngle
        if Delta > math.pi:
            Delta-=2*math.pi
        elif Delta < -math.pi:
            Delta+=2*math.pi
        return Delta


    def SeqWithBalance(self,pos1,pos2,T,dt, COMref = 0):
        if len(pos1) == len(pos2) == len(self._jnt_names):
            N = ceil(T/dt)
            pos1 = array(pos1)
            pos2 = array(pos2)

            init_com, rot = self._iTf.TransformListener().lookupTransform('/com','/l_foot',rospy.Time(0))
            if COMref != 0:
                COMref0 = [init_com[0],init_com[1]]
                init_com = [0]*2

            for ratio in linspace(0, 1, N):
                cur_com, rot = self._iTf.TransformListener().lookupTransform('/com','/l_foot',rospy.Time(0))
                interpCommand = (1-ratio)*pos1 + ratio * pos2
  
                if COMref != 0:
                    init_com[0] = (1-ratio)*COMref0[0] + ratio*COMref[0]
                    init_com[1] = (1-ratio)*COMref0[1] + ratio*COMref[1]

                # Balance torso
                back_mby_pos = interpCommand[1]+2*(cur_com[0]-init_com[0])
                back_ubx_pos = interpCommand[2]+2*(cur_com[1]-init_com[1])
  
                self.JC.set_all_pos([ float(x) for x in interpCommand ])
                self.JC.set_pos("back_mby",back_mby_pos)
                self.JC.set_pos("back_ubx",back_ubx_pos)

                self.JC.send_command()
                rospy.sleep(dt)

        else:
            print 'position command legth doest fit'


    def Sit(self,T):
        self.JC.set_gains("l_arm_mwx",20,0,15)
        self.JC.set_gains("r_arm_mwx",20,0,15)
        self.JC.set_gains("l_arm_elx",50,0,15)
        self.JC.set_gains("r_arm_elx",50,0,15)
        pos = copy(self.SitDwnSeq1)
        self.send_pos_traj(self.RS.GetJointPos(),pos,T*0.5,0.005)
        # self.SeqWithBalance(self.RS.GetJointPos(),self.SitDwnSeq1,T*0.3,0.005)
        self.JC.set_pos("l_leg_uay",-0.1)
        self.JC.set_pos("r_leg_uay",-0.1)
        self.JC.set_gains("l_leg_uay",10,0,5,set_default = False)
        self.JC.set_gains("r_leg_uay",10,0,5,set_default = False)
        self.JC.set_gains("l_leg_lax",10,0,5,set_default = False)
        self.JC.set_gains("r_leg_lax",10,0,5,set_default = False)
        self.JC.send_command()
        rospy.sleep(T*0.3)
        self.JC.set_gains("back_mby",4000,0,10)
        self.JC.set_gains("l_arm_elx",200,0,3)
        self.JC.set_gains("r_arm_elx",200,0,3)
        self.JC.set_gains("l_arm_mwx",100,0,0.2)
        self.JC.set_gains("r_arm_mwx",100,0,0.2)
        # self.JC.set_gains("l_leg_uay",50,0,5,set_default = False)
        # self.JC.set_gains("r_leg_uay",50,0,5,set_default = False)
        # self.JC.set_gains("l_leg_lax",50,0,5,set_default = False)
        # self.JC.set_gains("r_leg_lax",50,0,5,set_default = False)
        self.send_pos_traj(self.RS.GetJointPos(),self.SitDwnSeq2,T*0.2,0.005)
        self.JC.set_gains("l_arm_mwx",400,0,10)
        self.JC.set_gains("r_arm_mwx",400,0,10)
        # self.JC.send_command()
        # rospy.sleep(T*0.2)


    def DoPath(self,Path):
        for Point in Path:
            if Point[2] == "fwd":
                self.Throtle=1
            if Point[2] == "bwd":
                self.Throtle=0.6

            self.GoToPoint(Point)
            
    def GoToPoint(self,Point):
        if Point[2] == "fwd":
            self.Throtle=1
        if Point[2] == "bwd":
            self.Throtle=0.6
        self._Point = Point
        # Calculate distance and orientation to target
        while (0 == self.GlobalPos):
            rospy.sleep(1)
            print("Waiting for GlobalPos")
        DeltaPos = [self._Point[0]-self.GlobalPos.x,self._Point[1]-self.GlobalPos.y]
        Distance = math.sqrt(DeltaPos[0]**2+DeltaPos[1]**2)

        # Get current orientation
        y,p,r = self.current_ypr()
        T_ori = math.atan2(DeltaPos[1],DeltaPos[0])
        self.DesOri = T_ori
        if self._Point[2] == "bwd":
            T_ori += math.pi

        # Rotate in place towards target
        if abs(self.DeltaAngle(T_ori,y))>0.1:
            self.RotateToOri(T_ori)


        # Crawl towards target
        if self._Point[2] == "fwd":
            self.Crawl()
        if self._Point[2] == "bwd":
            self.BackCrawl()

        self._stuck_counter = 0


    def PerformStep(self):
        result = False
        self.count_tottal +=1
        print 'Total_count:', self.count_tottal
        # Calculate distance and orientation to target
        DeltaPos = [self._Point[0]-self.GlobalPos.x,self._Point[1]-self.GlobalPos.y]
        Distance = math.sqrt(DeltaPos[0]**2+DeltaPos[1]**2)
        T_ori = math.atan2(DeltaPos[1],DeltaPos[0])
        self.DesOri = T_ori
        if self._Point[2] == "bwd":
            T_ori += math.pi

        if Distance<0.8:
            print "Reached Waypoint"
            result = True
        else:
            y,p,r = self.current_ypr()
            # Check tipping
            self.CheckTipping()

            # Rotate in place towards target
            if self._fall_count < self.FALL_LIMIT:
              # Crawl towards target
                if self._Point[2] == "fwd":
                    self.Crawl()
                if self._Point[2] == "bwd":
                    self.BackCrawl()
                Drift = abs(self.DeltaAngle(T_ori,y))
                if 0.5<Drift<1.4 and Distance>1:
                    self.RotateToOri(T_ori)
                if Drift>1.4:
                    self.RotateToOri(T_ori)

                    self.BackCrawl()
                DeltaPos2 = [self._Point[0]-self.GlobalPos.x,self._Point[1]-self.GlobalPos.y]
                Distance2 = math.sqrt(DeltaPos2[0]**2+DeltaPos2[1]**2)
                if self._terrain !='MUD':
                    if abs(Distance2-Distance)<0.1:
                        self._stuck_counter+=1
                        print 'stuck... D=',(Distance2-Distance)
                        if self._stuck_counter >= 2:
                            #dont follow path
                            self.FollowPath = 0
                            #go oposite dir
                            if self._Point[2] == "bwd":
                                self.Crawl()
                            if self._Point[2] == "fwd":
                                self.BackCrawl()
                            self.RotSpotSeq(2)
                            self.FollowPath = 1
                            self._stuck_counter = 0
                    else:
                        self._stuck_counter = 0

            else:
                self.FollowPath = 0
        return result


    def Crawl(self):
        # self.IMU_mon.turned = 0
        if self.FollowPath == 1:
            # Update sequence to correct orientation
            y,p,r = self.current_ypr()
            print("Bearing: %f" % y)
            Correction = self.DeltaAngle(self.DesOri,y)
            print("Bearing: %f, DesOri: %f, Coorection = %f" % (y,self.DesOri,Correction))

            Delta = Correction/0.2
            if abs(Delta)>1:
                Delta /= abs(Delta)

            self.AddRotation(Delta)

        if self.RotFlag == 1:
            print "Getting ready"
            self.RotFlag = 2
            self.CurSeqStep = 0
            self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],0.5,0.005)

        self.GoToSeqStep(len(self.StepDur))
        rospy.sleep(0.2)
        self.RotFlag = 0


    def BackCrawl(self):
        # self.IMU_mon.turned = 1
        if self.FollowPath == 1:
            # Update sequence to correct orientation
            y,p,r = self.current_ypr()
            Correction = self.DeltaAngle(self.DesOri+math.pi,y)

            Delta = Correction/0.22
            if abs(Delta)>1:
                Delta /= abs(Delta)

            self.AddBackRotation(Delta)

        if self.RotFlag == 1:
            self.RotFlag = 2
            self.GoToBackSeqStep(1)

        self.GoToBackSeqStep(len(self.StepDur2))


    def GoToSeqStep(self,Step):
        if Step == self.CurSeqStep:
            pass
        else:
            if Step > len(self.StepDur)-1:
                Step = 0
                self.DoSeqStep()
            while self.CurSeqStep != Step:
                self.DoSeqStep()


    def GoToBackSeqStep(self,Step):
        if Step == self.CurSeqStep2:
            pass
        else:
            if Step > len(self.StepDur2)-1:
                Step = 0
                self.DoInvSeqStep()
            while self.CurSeqStep2 != Step:
                self.DoInvSeqStep()

    
    def DoSeqStep(self): 
        print 'Doing Step seq #',self.CurSeqStep
        #self.traj_with_impedance(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],self.StepDur[self.CurSeqStep]/self.Throtle,0.005) 
        self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],self.StepDur[self.CurSeqStep]/self.Throtle,0.005) 
        self.JC.reset_gains()
        self.CurSeqStep += 1
        if self.CurSeqStep > len(self.StepDur)-1:
            self.CurSeqStep = 0
    

    def DoInvSeqStep(self):
        if self._terrain =='HILLS':
            if self.CurSeqStep == 2: ###### NEW ######
                self.JC.set_gains('r_arm_ely',600,0,30,set_default= False)
                self.JC.set_gains('l_arm_ely',600,0,30,set_default= False)

            if self.CurSeqStep2 == 3:
             
                if self.IMU_mon.second_contact == 'arm_r':
                    self.JC.set_gains('r_arm_ely',3000,0,10,set_default= False)
                    self.JC.set_gains('l_arm_ely',1000,0,10,set_default= False)

                elif self.IMU_mon.second_contact == 'arm_l':
                    self.JC.set_gains('l_arm_ely',3000,0,10,set_default= False)
                    self.JC.set_gains('r_arm_ely',1000,0,10,set_default= False)

                print 'second_contact:',self.IMU_mon.second_contact   

            print 'Doing inv. Step seq #',self.CurSeqStep2

            if self.CurSeqStep2 == 1 or self.CurSeqStep2 == 2:
                pos = list(self.RS.GetJointPos())
                pos[2] = -1.3*self.IMU_mon.roll 
                self.send_pos_traj(list(self.RS.GetJointPos()),pos,0.2,0.01)
                self.RobotCnfg2[self.CurSeqStep2][2] = pos[2] ###### NEW ######

                # pos = copy(self.RobotCnfg2[1][:])
                # self.TimeVariantIterp(self.RS.GetJointPos(),self.RobotCnfg2[self.CurSeqStep2],self.StepDur2[self.CurSeqStep2]/self.Throtle,0.005,0)
                # self.CurSeqStep2 += 1
                # if self.CurSeqStep2 > 4:
                #     self.CurSeqStep2 = 0
                # return

        self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg2[self.CurSeqStep2],self.StepDur2[self.CurSeqStep2]/self.Throtle,0.005) 
        self.JC.reset_gains()
        self.CurSeqStep2 += 1
        if self.CurSeqStep2 > len(self.StepDur2)-1:
            self.CurSeqStep2 = 0
        # if self.CurSeqStep2 > 4:
        #     self.CurSeqStep2 = 0


    def AddRotation(self,Delta):
        # Delta of 1 gives approx. 0.2 radians turn right
        # Add gait changes to appropriate step
        self.RobotCnfg[2][0] = Delta*0.1

        # Insert those changes in following steps as well
        self.RobotCnfg[3][0] = Delta*0.1
        self.RobotCnfg[4][4] = self.BaseHipZ+Delta*0.1
        self.RobotCnfg[4][4+6] = -self.BaseHipZ+Delta*0.1
        self.RobotCnfg[0][4] = self.BaseHipZ+Delta*0.1
        self.RobotCnfg[0][4+6] = -self.BaseHipZ+Delta*0.1


    def AddBackRotation(self,Delta):
        # Delta of 1 gives approx. 0.22 radians turn left
        if Delta>0:
            dID = 0
        else:
            dID = 6

        if abs(Delta)>1:
            Delta = Delta/abs(Delta)

        # Add gait changes to appropriate step
        self.RobotCnfg2[1][0] = 0.25*Delta
        self.RobotCnfg2[1][2] = 0.15*Delta
        self.RobotCnfg2[3][0] = 0
        self.RobotCnfg2[3][2] = 0

        # Insert those changes in following steps as well
        self.RobotCnfg2[2][0] = 0.25*Delta
        self.RobotCnfg2[2][2] = 0.15*Delta
        self.RobotCnfg2[3][0] = 0.3*Delta
        self.RobotCnfg2[3][2] = 0.15*Delta
        self.RobotCnfg2[4][0] = 0
        self.RobotCnfg2[4][2] = 0
        self.RobotCnfg2[0][0] = 0
        self.RobotCnfg2[0][2] = 0


    def RotateToOri(self,Bearing):
        if self._terrain == "MUD" or self._terrain == "HILLS":
            return self.RotateToOriInMud(Bearing)
        else:
            if self.RotFlag == 2:
                self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[-1][:],1,0.01)
            self.RotFlag = 1

            # Make sure Bearing is from -pi to +pi
            Bearing = Bearing % (2*math.pi)
            if Bearing > math.pi:
                Bearing -= 2*math.pi
            if Bearing < -math.pi:
                Bearing += 2*math.pi

            # Get into "break-dance" configuration
            #self.GoToSeqStep(3)
            pos=copy(self.RobotCnfg[2][:])
            pos[1] = 0.9
            pos[6] = pos[6+6] = -1.7
            pos[8] = pos[8+6] = 0.7
            pos[16] = pos[16+6] = 0.9
            pos[17] = -0.9
            pos[17+6] = 0.9
            pos[18] = pos[18+6] = 2
            pos[19] = 1.0
            pos[19+6] = -1.0
            self.send_pos_traj(self.RS.GetJointPos(),pos,1,0.01) 

            # Get current orientation
            y0,p,r = self.current_ypr()
            Angle=self.DeltaAngle(Bearing,y0)

            icount = 0
            while abs(Angle)>0.3:#0.15: # Error of 9 degrees
                print 'Delta: ',Angle,'Bearing: ',Bearing, 'yaw: ',y0
                Delta = Angle/0.75
                if abs(Delta)>1:
                    Delta/=abs(Delta)
                if 0<Delta<0.35:
                    Delta+=0.25
                if -0.35<Delta<0:
                    Delta-=0.25

                self.RotSpotSeq(Delta)

                # Check tipping
                self.CheckTipping()
                
                # Get current orientation
                y,p,r = self.current_ypr()
                if abs(self.DeltaAngle(y,y0))<0.1:
                    icount = icount+1
                    if icount>2:
                        # Robot isn't turning, try RotateInMud
                        return self.RotateToOriInMud(Bearing)
                    else:
                        print("Rotation failed, attempt %d" % icount)

                y0 = y
                Angle=self.DeltaAngle(Bearing,y0)

            # # Return to original configuration

            # self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[0][:],0.5,0.01) 
            # self.CurSeqStep = 0
            # self.CurSeqStep2 = 0
            #############################################
            # Return to original configuration

            self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[-1][:],1,0.01)
            rospy.sleep(0.5)
            self.CurSeqStep = 0
            self.CurSeqStep2 = 0
            return 1


    def RotateToOriInMud(self,Bearing):
        self.RotFlag = 1

        # Make sure Bearing is from -pi to +pi
        Bearing = Bearing % (2*math.pi)
        if Bearing > math.pi:
            Bearing -= 2*math.pi
        if Bearing < -math.pi:
            Bearing += 2*math.pi

        self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg2[4][:],1.5,0.01)

        # Get current orientation
        y0,p,r = self.current_ypr()
        Angle=self.DeltaAngle(Bearing,y0)
        # print 'Angle: ',Angle
        while abs(Angle)>0.1: # Error of 3 degrees
            print 'MUD Delta: ',Angle,'Bearing: ',Bearing, 'yaw: ',y0
            Delta = Angle/0.45
            if abs(Delta)>1:
                Delta/=abs(Delta)
            # if 0<Delta<0.35:
            #     Delta+=0.25
            # if -0.35<Delta<0:
            #     Delta-=0.25

            self.RotOnMudSeq(Delta)
            # Check tipping
            self.CheckTipping()
            # Get current orientation
            y,p,r = self.current_ypr()
            if abs(self.DeltaAngle(y,y0))<0.1:
                # Robot isn't turning, Give up
                return 0

            y0 = y
            # Angle=self.DeltaAngle(Bearing,y0)
            Angle=self.DeltaAngle(Bearing,y0)

        # # Return to original configuration
        # self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[0][:],0.5,0.01) 
        # self.CurSeqStep = 0
        # self.CurSeqStep2 = 0

        #############################################
            # Return to original configuration

        self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg2[4][:],2,0.01)
        rospy.sleep(1)
        self.CurSeqStep2 = 4
        return 1


    def RotSpotSeq(self,Delta):
        # Delta of 1 gives a left rotation of approx. 0.75 radians
        knee0 = 1.1
        hip0 = self.BaseHipZ/2
        arm0 = 1.0
        d_arm = 0.5

        # Init configuration
        pos=copy(self.RobotCnfg[2][:])
        pos[1] = 0.8
        pos[6] = pos[6+6] = -1.7
        pos[7] = pos[7+6] = knee0
        pos[8] = pos[8+6] = 0.8
        pos[9] = pos[9+6] = 0
        pos[16] = pos[16+6] = 0.9
        pos[17] = -arm0
        pos[17+6] = arm0
        pos[18] = pos[18+6] = 2.3
        pos[19] = 0.6
        pos[19+6] = -0.6

        if self.RotFlag != 1:
            self.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005) 
        self.RotFlag = 1

        T=1.
        if Delta>0:
            sign = 1
            dID = 0
        else:
            sign = -1
            dID = 6
        Delta = abs(Delta)

        # Lift first leg (and arm)
        pos[7+dID] = knee0-1
        pos[17+6-dID] = sign*(arm0-d_arm)
        pos[19+6-dID] = -sign*(0.6+d_arm)
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.12*T,0.005) 
        # Rotate first leg outwards until it touches ground
        pos[4+dID] = hip0*sign+Delta*sign*0.35
        pos[4+6-dID] = -hip0*sign-Delta*sign*0.35
        pos[5+dID] = hip0*sign+Delta*sign*0.2
        pos[5+6-dID] = -hip0*sign-Delta*sign*0.2
        pos[9+dID] = Delta*sign*0.2
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.24*T,0.005) 
        # Lift other leg, now all the weight is on first leg
        pos[7+dID] = knee0
        pos[17+dID] = -sign*(arm0-d_arm)
        pos[19+dID] = sign*(0.6+d_arm)
        pos[7+6-dID] = knee0-1
        pos[17+6-dID] = sign*(arm0)
        pos[19+6-dID] = -sign*(0.6)
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.12*T,0.005) 
        # Return first leg and torso to original configuration
        pos[4+dID] = hip0*sign
        pos[4+6-dID] = -hip0*sign
        pos[5+dID] = hip0*sign
        pos[5+6-dID] = -hip0*sign
        pos[7+dID] = knee0
        pos[9+dID] = 0
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.24*T,0.005) 
        # Return other leg to original configuration
        pos[7+6-dID] = knee0
        pos[17+dID] = -sign*(arm0)
        pos[19+dID] = sign*(0.6)
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.12*T,0.005) 
        rospy.sleep(0.1*T)


    def RotOnMudSeq(self,Delta):
        # Delta of 1 gives a left rotation of approx. 0.35 radians

        knee0 = 1.0
        kneediff = 0.3
        if Delta>0:
            dID = 0
            sign = 1
        else:
            dID = 6
            sign = -1

        self.RotFlag = 1
        T=1

        # Get into starting position
        pos = copy(self.RobotCnfg2[4][:])
        pos[1] = 0.8
        # Get current orientation
        # y0,p,r = self.current_ypr()
        # pos[2] = -0.5*r
        pos[4] = 0.1
        pos[4+6] = -0.1
        pos[5] = 0.1
        pos[5+6] = -0.1
        pos[6] = pos[6+6] = -1.3
        pos[7] = pos[7+6] = knee0
        pos[8] = pos[8+6] = 0.5
        pos[9] = pos[9+6] = 0
        if Delta>0:
            pos[16] = 0.6-0.3*Delta
            pos[16+6] = 0.6
        else:
            pos[16] = 0.6
            pos[16+6] = 0.6+0.3*Delta
        pos[17] = -1.1
        pos[17+6] = 1.1
        pos[18] = pos[18+6] = 2.5
        pos[19] = 0.5
        pos[19+6] = -0.5
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.01)

        # Lift first leg
        # pos[2] = 0
        pos[6+dID] = -1.7
        pos[7+dID] = knee0+kneediff
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.01)

        # Rotate it to the side
        pos[0] = 0.15*Delta
        # pos[2] = 0.2*Delta
        # pos[5+dID] = 0.1+0.4*Delta
        pos[4+dID] = sign*(0.1+0.2*Delta)
        pos[5+dID] = 0.1+0.2*Delta
        pos[7+dID] = knee0
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.4*T,0.01)

        # Lower first leg / Lift second leg
        pos[6+dID] = -1.2
        # pos[8+dID] = 0.2
        pos[6+6-dID] = -1.7
        # pos[7+6-dID] = knee0+kneediff
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.01)

        # Rotate pelvis / Close first leg
        pos[0] = 0
        pos[4] = 0.1
        pos[4+6] = -0.1
        pos[5] = 0.1
        pos[5+6] = -0.1
        # pos[2] = 0
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.4*T,0.01)

        # Return to init
        pos[6] = pos[6+6] = -1.3
        pos[7] = pos[7+6] = knee0
        pos[8] = pos[8+6] = 0.5
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.01)

        # Lift arms
        pos[6] = pos[6+6] = -1.5
        pos[8] = pos[8+6] = 0.7
        pos[17] = -0.5
        pos[17+6] = 0.5
        pos[16] = pos[16+6] = 0.8
        pos[18] = pos[18+6] = 2.5
        pos[19] = 1.1
        pos[19+6] = -1.1

        # pos = copy(self.RobotCnfg2[1][:])
        # # y0,p,r = self.current_ypr()
        # # pos[2] = -0.5*r
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.01)

        # self.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.01)

        # raise KeyboardInterrupt


    def CheckTipping(self):
        result = 0
        while result == 0:
            # Get current orientation
            
            y,p,r = self.current_ypr()
            self._fall_count += 1
            # R,P,Y = self.RS.GetIMU()
            R,P,Y = self.RS._orientation.GetRPY()

            print 'Check Tipping r=',r,"R=",R,"p=",p,"P=",P
            # if abs(p)>0.4*math.pi or abs(r)>0.8*math.pi:
            if  P>=0.8:
                # print "Front recovery"
                result = self.FrontTipRecovery()
            elif abs(p)>0.4*math.pi or abs(r)>0.8*math.pi:
                # Robot tipped backwards
                # print "Back recovery"
                result = self.BackTipRecovery()
                self.CurSeqStep2 = 0########
                self.CurSeqStep = 0###############

            elif r>math.pi/4:
                # Robot tipped to the right
                result = self.TipRecovery("right")
                self.CurSeqStep2 = 0########
                self.CurSeqStep = 0###############
                # print "Right recovery"
            elif r<-math.pi/4:
                # Robot tipped to the left
                result = self.TipRecovery("left")
                self.CurSeqStep2 = 0########
                self.CurSeqStep = 0###############
                # print "Left recovery"
            else:
                result = 1
                # self.FollowPath = 1
                self._fall_count = 0
        self.RotFlag = 1


    def TipRecovery(self,side):
        self.count_tipping += 1
        print 'Tipping:', self.count_tipping

        dID = 0
        sign = 1
        if side == "right":        
            dID = 6
            sign = -1

        # # Simple way of turning fwd
        # pos = copy(self.RobotCnfg[-1][:])
        # pos[17+dID] = sign*1.0
        # self.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01)

        pos = copy(self.RobotCnfg[-1][:])
        pos[18+dID] = 2.4
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)
        rospy.sleep(0.1)

        # Extend leg and bend arm
        pos = copy(self.RobotCnfg[-1][:])
        pos[7+dID] = 0.8
        pos[17+dID] = -sign*1.3
        pos[18+dID] = 2.6
        pos[19+dID] = sign*2.35
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.3,0.01)

        rospy.sleep(0.1)

        # Push body by rotating arm
        pos[18+dID] = 1.2
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        rospy.sleep(0.1)

        # Extend arm back to normal
        pos[17+dID] = -sign*1.1
        pos[19+dID] = sign*0.05
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        pos = copy(self.RobotCnfg[-1][:])
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        rospy.sleep(0.5)
        R,P,Y = self.RS.GetIMU()
          
        if  P>=0.8:
            print "Front recovery"
            result = self.FrontTipRecovery()
            return result

        # Get current orientation
        y,p,r = self.current_ypr()
        if abs(r)<math.pi/4:
            # Success!
            self.CurSeqStep = 0
            self.CurSeqStep2 = 0
            return 1

        else:
            return 0  


    def BackTipRecovery(self):
        # "Flatten" body on ground, bend elbows
        pos = copy(self.RobotCnfg[4][:])
        pos[1] = 0
        pos[6] = pos[6+6] = -0.4
        pos[7] = pos[7+6] = 0.8
        pos[8] = pos[8+6] = 0.3
        pos[16] = pos[16+6] = 0
        pos[17] = 0.4
        pos[17+6] = -0.4
        pos[18] = pos[18+6] = 3.14
        pos[19] = 1.1
        pos[19+6] = -1.1
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.4,0.01)
        rospy.sleep(0.3)

        # Bend arms
        pos[1] = 0.8
        pos[16] = pos[16+6] = 1.4
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        # Prepare to push
        pos[17] = 1.3
        pos[17+6] = -1.3
        pos[18] = pos[18+6] = 2.6
        pos[19] = 2.3
        pos[19+6] = -2.3
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.3,0.01)

        # PUSH!!!
        pos = copy(self.RobotCnfg[-1][:])
        pos[6] = pos[6+6] = -1.3
        pos[17] = -1.1
        pos[17+6] = 1.1
        pos[18] = pos[18+6] = 2.6
        pos[19] = pos[19+6] = 0.05
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.3,0.01)

        self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[-1][:],0.5,0.01)
        rospy.sleep(0.5)

        # Get current orientation
        y,p,r = self.current_ypr()
        if abs(p)<0.4*math.pi:
            # Success!
            self.CurSeqStep = 0
            self.CurSeqStep2 = 0
            return 1
        else:
            return 0


    def FrontTipRecovery(self):

        pos = [0, 0, 0, 0,
            0, 0, -0.02, 0.04, -0.02, 0,
            0, 0, -0.02, 0.04, -0.02, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0]
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.4,0.01)

        pos = [ 0, 0, 0, 0,
            0, 0, 0, 0, -2.1, 0,
            0, 0, 0, 0, -2.1, 0,
            0, 0, 0, 1.5, 0, 0,
            0, 0, 0, -1.5, 0, 0]
        # self.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01)
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.3,0.01)
        rospy.sleep(0.3)

        pos = [0, 0.8, 0, 0,
            0, 0, -2, 2, -2.1, 0,
            0, 0, -2, 2, -2.1, 0,
            -1.7, -0.3, 0, 1.5, 0, 0,
            -1.7, 0.3, 0, -1.5, 0, 0]
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        pos = [0, 1.1, 0, 0,
            0, 0, -2.2, 2.8, -2.6, 0,
            0, 0, -2.2, 2.8, -2.6, 0,
            -1.4, -0.8, 1.5, 0, 0, 1.5,
            -1.4, 0.8, 1.5, 0, 0, -1.5] 
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        pos = [0, 0.8, 0, 0,
            0, 0, -2, 2.8, -2.6, 0,
            0, 0, -2, 2.8, -2.6, 0,
            -1.3, -0.7, 0, 0, 0, 0,
            -1.3, 0.7, 0, 0, 0, 0]
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        rospy.sleep(0.4)

        pos = copy(self.RobotCnfg[-1][:])
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.6,0.01)
        rospy.sleep(0.5)

        # Get current orientation
        y,p,r = self.current_ypr()
        if abs(p)<0.4*math.pi:
            # Success!
            self.CurSeqStep = 0
            self.CurSeqStep2 = 0
            return 1
        else:
            return 0


    def DynStandUp(self):
        # Works only a small percentage of the time
        # SeqWithBalance needs to be updated to include a COM horizontal motion
        # (since the hip begins the motion from behind the feet)
        self.JC.set_gains("l_arm_mwx",1200,0,5)
        self.JC.set_gains("r_arm_mwx",1200,0,5)

        T = 1

        # Go to init pos, supported on hands and feet, with pelvis back between hands
        pos = copy(self.RobotCnfg[0][:])
        pos[1] = 0.3
        pos[4] = pos[4+6] = 0
        pos[5] = pos[5+6] = 0
        pos[7] = pos[7+6] = 1.2
        pos[8] = pos[8+6] = 0.7
        pos[16] = pos[16+6] = 0.5
        pos[19] = 0.8
        pos[19+6] = -0.8
        pos[18] = pos[18+6] = 2.7
        pos[19] = 0.8
        pos[19+6] = -0.8
        pos[20] = pos[20+6] = 0
        pos[21] = pos[21+6] = 0
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.5*T,0.005)
        rospy.sleep(0.5)

        # [Insert description here]
        pos[16] = pos[16+6] = 0.3
        pos[17] = -1.4
        pos[17+6] = 1.4
        pos[19] = pos[19+6] = 0
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.5*T,0.005)

        rospy.sleep(0.2)

        # Tuck legs in and start rotation
        pos[1] = 0.8
        pos[5] = 0.1
        pos[5+6] = -0.1
        pos[6] = pos[6+6] = -1.7
        pos[7] = pos[7+6] = 2.4
        pos[8] = pos[8+6] = -0.7
        pos[9] = -0.1
        pos[9+6] = 0.1
        pos[16] = pos[16+6] = -0.5
        pos[17] = -1.2
        pos[17+6] = 1.2
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.005)

        pos[1] = 0.2
        pos[16] = pos[16+6] = -1.3
        pos[17] = -0.6
        pos[17+6] = 0.6
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.3*T,0.005)
        rospy.sleep(0.2)

        pos[6] = pos[6+6] = 0
        pos[17] = -0.5
        pos[17+6] = 0.5
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.005)
        rospy.sleep(0.3)
        pos[6] = pos[6+6] = -1.7
        pos[17] = -0.9
        pos[17+6] = 0.9
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.1*T,0.005)

        # rospy.sleep(1)
        rospy.sleep(0.5)

        # Use hands to push off and place COM on feet
        self.JC.set_gains('l_leg_uay',900,0,10)
        self.JC.set_gains('r_leg_uay',900,0,10)
        self.JC.set_gains('l_leg_lax',900,0,10)
        self.JC.set_gains('r_leg_lax',900,0,10)
        self.JC.set_gains('l_arm_usy',1000,0,10)
        self.JC.set_gains('r_arm_usy',1000,0,10)
        self.JC.set_gains('l_arm_shx',1000,0,10)
        self.JC.set_gains('r_arm_shx',1000,0,10)
        pos[1] = 0.7
        pos[16] = pos[16+6] = -1.4
        pos[17] = -1.1
        pos[17+6] = 1.1
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.4*T,0.005)
        rospy.sleep(0.2)

        rospy.sleep(0.5)
        pos[17] = -0.9
        pos[17+6] = 0.9
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.005)

        rospy.sleep(2)
        RPY = self.RS.GetIMU()
        print RPY
        if abs(RPY[0])+abs(RPY[1])<0.2:
            self.SeqWithBalance(self.RS.GetJointPos(),self.BasStndPose,3,0.005,[-0.02, 0.175])
            rospy.sleep(0.8)
            self.send_pos_traj(self.RS.GetJointPos(),self.BasStndPose,0.5*T,0.005)


    def StandUp(self):
        RPY = self.RS.GetIMU()
        D, R = self._iTf.TransformListener().lookupTransform('/l_foot','/pelvis',rospy.Time(0))
        while not (abs(RPY[0])<= 0.1 and abs(RPY[1])<=0.1 and D[2] >= 0.8):
            self.CheckTipping()
            self.DynStandUp()
            rospy.sleep(1)
            RPY = self.RS.GetIMU()
            D, R = self._iTf.TransformListener().lookupTransform('/l_foot','/pelvis',rospy.Time(0))
            print 'roll: ',RPY[0],'pitch: ',RPY[1],'D: ',D[2]


    def CloseHands(self):
        self.LHC.set_all_pos(self.BaseHandPose)
        self.RHC.set_all_pos(self.BaseHandPose)
        self.LHC.send_command()
        self.RHC.send_command()


##################################################################
######################### USAGE EXAMPLE ##########################
##################################################################

if __name__=='__main__':
    rospy.init_node("DW_test")
    # rospy.sleep(0.5)
    iTF = Interface_tf()
    DW = DW_Controller(iTF)
    DW.Initialize(Terrain = "HILLS") 
    # rospy.Subscriber('/C25/publish',C25C0_ROP,DW.Odom_cb)
    odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,DW.Odom_cb)

    # Subscribe to topic controller
    rospy.Subscriber('/DW_control',String,DW.Interface_cb)

    rospy.Subscriber('/atlas/atlas_state',AtlasState,DW.RS_cb)
    rospy.sleep(0.1)
    DW.send_pos_traj(DW.RS.GetJointPos(),array(DW.RS.GetJointPos()),0.1,0.005)
    DW.CloseHands()
    rospy.sleep(0.1)

    while True:
        comm = raw_input("Enter command: ")

        if comm == "Exit" or comm == "exit":
            break

        try:
            DW.Interface_cb(String(comm))
        except KeyboardInterrupt:
            print "Interrupting sequence..."
            break

    # rospy.spin()

    # DW.Sit(1.5)       
    # rospy.sleep(0.5)

    # Point1 = [1.15,-10.27,"fwd"] # Point close to right side of gate
    # Point2 = [3.19,-8.5,"fwd"] # Point on the right of first mount
    # Point3 = [1.78,-5.08,"fwd"] # Point before crossing "ukaf"
    # Point4 = [5.46,-4.32,"fwd"] # Point after crossing "ukaf"
    # Point5 = [4.74,-1.55,"fwd"] # Fork point
    # Point6 = [4.29,1.28,"fwd"] # Daring option, point before cross
    # Point7 = [6.76,4.98,"fwd"] # Daring option, point after cross
    # Point8 = [6.08,6.77,"fwd"] # Final gate
    # Path = [Point1,Point2,Point3,Point4,Point5,Point6,Point7,Point8]
    # # rospy.sleep(2)
    # DW.DoPath(Path)
    # # DW.FrontTipRecovery()

    