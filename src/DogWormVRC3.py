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
from time import strftime, gmtime
from copy import copy
from std_srvs.srv import Empty
##nedded for tests
from IMU_monitoring import IMUCh
# from Abstractions.Interface_tf import *
from std_msgs.msg import String
from GPlugin.srv import *
from sys import stdout
from FricPlugin.srv import *
import subprocess
import signal


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

        self.Print("PW::Initialize",'system')
        self.GlobalPos = 0
        self.GlobalOri = 0
        self.HeadPitch = 0
        self._counter = 0
        self._terrain = Terrain
        self.RotMode = 1
        self.FRKneeExt = 0.6
        self._fall_count = 0
        self.FALL_LIMIT = 3
        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)
        self.set_g_srv = rospy.ServiceProxy('/SetG', SetG)

        self._stat_pub = rospy.Publisher('/PW/status',Status)
        self._rpy_pub = rospy.Publisher('/PW_rpy',Vector3)

        self.MessageNum = 1

        # Commands
        self.Commands = [['sit','Sit down from standing position'],
                         ['fwd [n]','Crawl forward [n] times'],
                         ['bwd [n]','Crawl backwards [n] times'],
                         ['turn [ori]','Turn in place to desired [ori]entation (in radians)'],
                         ['rot type [fast/slow]','Set rotation type to slow or fast'],
                         ['rot [delta]','Rotate in place once with strength [delta] (+/- 1)'],
                         ['recover','Recover from tipping'],
                         ['stand','Stand up from sitting position'],
                         ['throttle [seq] [speed]','Increase/decrease the speed of a sequence (period/throttle)'],
                         ['reset pose','Resets the robot\'s configuration to the default standing pose'],
                         ['reset','Resets the robot\'s pose and returns it to its starting position'],
                         ['reload','Reloads the sit down, fwd, and bwd sequences parameters from seq_generator.py'],
                         ['seq [fwd/bwd] [step]','Brings the robot to step [step] of the [fwd/bwd] sequence'],
                         ['close hands','Closes the robot\'s hands'],
                         ['head [up/down] [rad]','Tilts the robot\'s head [up/down] by [rad] radians'],
                         ['befb [0/1]','Turns the bearing feedback response on [1] or off [0]'],
                         ['bedes [rad]','Sets the desired bearing for the bearing FB to [rad] radians from the x axis'],
                         ['terrain [MUD/HILLS/OTHER]','Sets the terrain that the robot is moving on'],
                         ['legspread [value]','Defines how much the legs are spread out in the basic quadruped position (use values between 0 and 1)'],
                         ['frknee [value]','Defines the knee extension during the fast rotation sequence (use values between 0 and 1)'],
                         ['gravity [x] [y] [z]','Sets the gravity vector. Default is [0] [0] [-9.81]'],
                         ['gravec [yaw] [pitch] [mag = 9.81]','Sets the gravity vector by yaw from X, pitch from Z and magnitude\nUse "[yaw] [pitch] deg" to set the angles in degrees\nUse gravec keep to maintain the gravity orientation relative to the robot'],
                         ['status','Shows the status of all the system\'s flags'],
                         ['test [n]','Runs test number [n]'],
                         ['commands','Shows the list of all available commands'],
                         ['help [command]','Provides help on using a command'],
                         ['exit','Exit this console']]

        self.Gravity = [0,0,-9.81]
        self.GraVecKeep = 0
        self.GraVec = [0,0,-9.81]

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
        self.Print(("COMMAND RECEIVED: %s" % CommString),'comm_in')
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
                        self.Print("Rotation mode set to fast",'comm_out')
                    if CommParted2[2].find("slow") == 0:
                        self.RotMode = 2
                        self.Print("Rotation mode set to slow",'comm_out')
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

            String = self.Commands[8][0].partition("[")[0]
            if Command.find(String) == 0: ################ THROTTLE ################
                MotionType = -1
                Parameters.append(MotionType)
                CommParted = Command.partition(String)
                CommParted2 = CommParted[2].partition(" ")
                Sequence = CommParted2[0]
                Throttle = float(CommParted2[2])
                Found = 0
                for k,v in self.Throttle.iteritems():
                    if k == Sequence.upper():
                        Found = 1
                        self.Throttle[k] = Throttle
                        self.Print(("Setting throttle for %s sequence to %.2f" % (Sequence, Throttle)),'comm_out')
                if Found == 0:
                    self.Print(("No throttle value for sequence %s found" % Sequence),'comm_out')

            if Command.find(self.Commands[10][0]) == 0: ################ RESET ###############
                if Command.find(self.Commands[9][0]) == 0: ########## RESET POSE #############
                    MotionType = 8
                else:
                    MotionType = 9
                Parameters.append(MotionType)

            if Command.find(self.Commands[11][0]) == 0: ############### RELOAD ##############
                MotionType = 10
                Parameters.append(MotionType)

            String = self.Commands[12][0].partition("[")[0]
            if Command.find(String) == 0: ################ SEQ ################
                MotionType = 11
                Parameters.append(MotionType)
                CommParted = Command.partition(String)
                CommParted2 = CommParted[2].partition(" ")
                Sequence = CommParted2[0]
                SeqStep = int(CommParted2[2])
                Parameters.append(Sequence)
                Parameters.append(SeqStep)

            if Command.find(self.Commands[13][0]) == 0: ########### CLOSE HANDS ###########
                MotionType = 12
                Parameters.append(MotionType)

            String = self.Commands[14][0].partition("[")[0]
            if Command.find(String) == 0: ################ HEAD ################
                MotionType = -1
                CommParted = Command.partition(String)
                CommParted2 = CommParted[2].partition(" ")
                if CommParted2[0].find("up"):
                    self.HeadPitch = self.HeadPitch + float(CommParted2[2])
                if CommParted2[0].find("down"):
                    self.HeadPitch = self.HeadPitch - float(CommParted2[2])
                self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[-1],0.5,0.005)

            String = self.Commands[15][0].partition("[")[0]
            if Command.find(String) == 0: ################ RESPONSE ################
                MotionType = -1
                CommParted = Command.partition(String)
                CommParted2 = CommParted[2].partition(" ")
                ResponseName = CommParted2[0]
                Switch = float(CommParted2[2])
                Found = 0
                for k,v in self.Responses.iteritems():
                    if k == ResponseName:
                        Found = 1
                        self.Responses[k] = Switch
                        if Switch == 0:
                            self.Print(("Turning %s response OFF" % ResponseName),'comm_out')
                        else:
                            self.Print(("Turning %s response ON" % ResponseName),'comm_out')
                if Found == 0:
                    self.Print(("No %s response found" % ResponseName),'comm_out')

            String = self.Commands[16][0].partition("[")[0]
            if Command.find(String) == 0: ################ BEDES ################
                MotionType = -1
                CommParted = Command.partition(String)
                self.DesOri = float(CommParted[2])
                self.Print(("Desired bearing set to: %f" % self.DesOri),'comm_out')

            String = self.Commands[17][0].partition("[")[0]
            if Command.find(String) == 0: ################ TERRAIN ################
                MotionType = -1
                CommParted = Command.partition(String)
                self._terrain = CommParted[2]
                self.Print(("Terrain type set to: %s" % self._terrain),'comm_out')

            String = self.Commands[18][0].partition("[")[0]
            if Command.find(String) == 0: ############## LEG SPREAD ##############
                MotionType = -1
                CommParted = Command.partition(String)

                args = {'LegSpread':float(CommParted[2])}
                stream = file('seqs_args.yaml','w')        
                yaml.dump(args,stream)
                self.LoadPoses()

                self.Print(("Leg spread parameter set to: %.2f. Sequences updated." % float(CommParted[2])),'comm_out')

            String = self.Commands[19][0].partition("[")[0]
            if Command.find(String) == 0: ########### FR KNEE EXTENSION ###########
                MotionType = -1
                CommParted = Command.partition(String)
                self.FRKneeExt = float(CommParted[2])
                self.Print(("Knee ext. parameter for fast rotation sequence set to: %.2f" % self.FRKneeExt),'comm_out')

            String = self.Commands[20][0].partition("[")[0]
            if Command.find(String) == 0: ############### GRAVITY ###############
                MotionType = -1
                GVector = Command.split(" ")
                req = SetGRequest()
                self.Gravity[0] = float(GVector[1])
                self.Gravity[1] = float(GVector[2])
                self.Gravity[2] = float(GVector[3])
                req.g_vec.x = float(GVector[1])
                req.g_vec.y = float(GVector[2])
                req.g_vec.z = float(GVector[3])
                self.set_g_srv(req)
                self.Print(("Set gravity vector to: (%.2f %.2f %.2f)" % (self.Gravity[0], self.Gravity[1], self.Gravity[2])),'comm_out')

            String = self.Commands[21][0].partition("[")[0]
            if Command.find(String) == 0: ############### GRAVEC ###############
                MotionType = -1
                GVector = Command.split(" ")
                req = SetGRequest()

                if Command.find("keep") >= 0:
                    self.GraVecKeep = 1
                    GVector = GVector[2:]
                else:
                    GVector = GVector[1:]

                if len(GVector) == 2:
                    gy = float(GVector[0])
                    gp = float(GVector[1])
                    g = 9.81
                else:
                    if GVector[2] == "deg":
                        gy = math.pi/180*float(GVector[0])
                        gp = math.pi/180*float(GVector[1])
                        g = 9.81
                    else:
                        gy = float(GVector[0])
                        gp = float(GVector[1])
                        g = float(GVector[2])

                if Command.find("keep") >= 0:
                    self.GraVec[0] = gy
                    self.GraVec[1] = gp
                    self.GraVec[2] = g

                self.Gravity[0] = g*math.sin(gp)*math.cos(gy)
                self.Gravity[1] = g*math.sin(gp)*math.sin(gy)
                self.Gravity[2] = -g*math.cos(gp)

                req.g_vec.x = float(self.Gravity[0])
                req.g_vec.y = float(self.Gravity[1])
                req.g_vec.z = float(self.Gravity[2])
                self.set_g_srv(req)
                self.Print(("Set gravity vector to: (%.2f %.2f %.2f)" % (self.Gravity[0], self.Gravity[1], self.Gravity[2])),'comm_out')

            if Command.find(self.Commands[22][0]) == 0: ########### STATUS ###########
                BLUE = '\033[94m'
                GREEN = '\033[92m'
                RED = '\033[91m'
                YELLOW = '\033[93m'
                WHAT = '\033[95m'
                END = '\033[0m'
                # Coloring can be done with "colorama" as well, just saying...

                MotionType = -1
                for k,v in self.Responses.iteritems():
                    if self.Responses[k] == 0:
                        self.Print(k.title()+" feedback is "+RED+"OFF"+END,'system1')
                    else:
                        self.Print(k.title()+" feedback is "+GREEN+"ON"+END,'system1')
                        if k == "bearing":
                            self.Print("Desired bearing set to: "+BLUE+("%f" % self.DesOri)+END,'system1')
                self.Print("Terrain type set to: "+BLUE+("%s" % self._terrain)+END,'system1')
                if self.RotMode == 1:
                    self.Print("Rotation mode set to "+YELLOW+"fast"+END,'system1')
                else:
                    self.Print("Rotation mode set to "+BLUE+"slow"+END,'system1')
                self.Print("Gravity vector is: "+BLUE+("(%.2f %.2f %.2f)" % (self.Gravity[0], self.Gravity[1], self.Gravity[2]))+END,'system1')
                String = "Throttle values: "
                for k, v in self.Throttle.iteritems():
                    String += WHAT+("%s " % k)+BLUE+("%.2f, " % v) # ("%s %.2f, " % (k,v)) # 
                self.Print(String[:-2]+END,'system')
                self.Print("The robot\'s position is: "+BLUE+("x = %.2f, y = %.2f, z = %.2f" % (self.GlobalPos.x,self.GlobalPos.y,self.GlobalPos.z))+END,'system1')
                y,p,r = self.current_ypr()
                R,P,Y = self.RS._orientation.GetRPY()
                self.Print("The robot\'s orientation is: "+BLUE+("Yaw = %.2f(%.2f), Pitch = %.2f(%.2f), Roll = %.2f(%.2f)" % (y,Y,p,P,r,R))+END,'system1')

            String = self.Commands[23][0].partition("[")[0]
            if Command.find(String) == 0: ################ TEST ################
                MotionType = -1
                Parameters.append(MotionType)
                CommParted = Command.split(" ")
                TestID = int(CommParted[1])
                if len(CommParted)==3:
                    Times = int(CommParted[2])
                    if Times == 1:
                        self.Print(("Running test number %d once..." % TestID),'system1')
                    elif Times == 2:
                        self.Print(("Running test number %d twice..." % TestID),'system1')
                    elif Times > 2:
                        self.Print(("Running test number %d %d times..." % (TestID,Times)),'system1')
                    else:
                        Times = 0
                else:
                    Times = 1
                    self.Print(("Running test number %d once..." % TestID),'system1')

                for x in range(Times):
                    if TestID == 1:
                        self.Test1()
                    if TestID == 2:
                        self.Test2()
                    if TestID == 3:
                        self.Test3()
                    if TestID == 4:
                        self.Test4()
                    if TestID == 5:
                        self.Test5()
                    if TestID == 6:
                        self.Test6()
                signal.alarm(int(1))

            if Command.find(self.Commands[24][0]) == 0: ########### COMMANDS ###########
                MotionType = -1
                self.Print("Available commands:",'system1')
                com_string = ""
                for com in self.Commands:
                    com_string += com[0]+", "
                self.Print(com_string[0:-2],'system1')

            String = self.Commands[25][0].partition("[")[0]
            if Command.find(String) == 0: ########### HELP ###########
                MotionType = -1
                CommParted = Command.partition(String)
                for com in self.Commands:
                    if com[0].find(CommParted[2])>=0:
                        self.Print(com[0]+" - "+com[1],'system1')
            
        if MotionType == 0:
            self.Print(("Got no command param, aborting..."),'system1')
        elif MotionType < 0:
            self.Print(" ",'comm_out')
        else:
            self._stat_pub.publish(Status("Busy"))
            self.DoTask(Parameters)


    def DoTask(self, Parameters):
        MotionType = Parameters[0]

        rospy.sleep(0.1)

        if MotionType == 1: ################ SIT ################
            self.Print("Sitting down...",'comm_out')
            self.Sit(1.5)
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 2: ################ FWD ################
            NumSteps = Parameters[1]
            self.Print("Crawling FWD %d steps..." % NumSteps,'comm_out')
            for x in range(NumSteps):
                if self.GraVecKeep == 1:
                    # Update gravity to accomodate drift
                    y,p,r = self.current_ypr()
                    SlopeStr = ("gravec %.4f %.4f %.4f" % (self.GraVec[0]+y,self.GraVec[1],self.GraVec[2]))
                    self.Interface_cb(String(SlopeStr))
                self.Crawl()
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 3: ################ BWD ################
            NumSteps = Parameters[1]
            self.Print("Crawling BWD %d steps..." % NumSteps,'comm_out')
            for x in range(NumSteps):
                if self.GraVecKeep == 1:
                    # Update gravity to accomodate drift
                    y,p,r = self.current_ypr()
                    SlopeStr = ("gravec %.4f %.4f %.4f" % (self.GraVec[0]+y,self.GraVec[1],self.GraVec[2]))
                    self.Interface_cb(String(SlopeStr))
                self.BackCrawl()
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 4: ################ TURN ################
            Bearing = Parameters[1]
            self.Print("Rotating to a bearig of %f radians..." % Bearing,'comm_out')
            if self.RotateToOri(Bearing):
                self.Print("SUCCESS!!\n",'comm_out')
            else:
                self.Print("FAIL\n",'comm_out')

        elif MotionType == 5: ########### SINGLE ROTATE ############
            Delta = Parameters[1]
            if self.RotMode == 1:
                self.Print("Rotating fast in place %0.0f%%" % (Delta*100),'comm_out')
                y0,p,r = self.current_ypr()
                self.RotSpotSeq(Delta)
                rospy.sleep(0.5)
                y,p,r = self.current_ypr()
                self.Print("Rotated %f radians" % (self.DeltaAngle(y,y0)),'comm_out')                
            else:
                self.Print("Rotating slowly in place %0.0f%%" % (Delta*100),'comm_out')
                y0,p,r = self.current_ypr()
                self.RotOnMudSeq(Delta)
                rospy.sleep(0.5)
                y,p,r = self.current_ypr()
                self.Print("Rotated %f radians" % (self.DeltaAngle(y,y0)),'comm_out')    
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 6: ################ RECOVER ################
            self.Print("Recovering after tipping...",'comm_out')
            if self.CheckTipping():
                self.Print("SUCCESS!!\n",'comm_out')
            else:
                self.Print("FAIL\n",'comm_out')

        elif MotionType == 7: ################ STANDUP ################
            self.Print("Standing up...",'comm_out')
            self.StandUp()
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 8: ################ RESET (POSE) ################
            self.Print("Resetting...",'comm_out')
            self.LoadPoses()
            self.JC.reset_gains()
            self.send_pos_traj(self.RS.GetJointPos(),self.BasStndPose,0.5,0.005)
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 9: ################ RESET (FULL) ################
            self.Print("Resetting...",'comm_out')
            self.LoadPoses()
            self.JC.reset_gains()
            self.send_pos_traj(self.RS.GetJointPos(),self.BasStndPose,0.5,0.005)
            rospy.sleep(0.5)
            self.reset()
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 10: ################ RELOAD ################
            self.Print("Loading new gaits...",'comm_out')
            self.LoadPoses()
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 11: ################ SEQUENCE ################
            Sequence = Parameters[1]
            SeqStep = Parameters[2]
            self.Print("Going to %s sequence step %d" % (Sequence, SeqStep),'comm_out')
            if Sequence.find("fwd") == 0:
                self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[SeqStep],1,0.005) 
                # self.GoToSeqStep(SeqStep)
            if Sequence.find("bwd") == 0:
                self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg2[SeqStep],1,0.005) 
                # self.GoToBackSeqStep(SeqStep)
            self.Print("SUCCESS!!\n",'comm_out')

        elif MotionType == 12: ############# CLOSE HANDS ##############
            self.Print("Closing hands...",'comm_out')
            self.CloseHands()
            self.Print("SUCCESS!!\n",'comm_out')

        self._stat_pub.publish(Status("Free"))
               
    def send_pos_traj(self,pos0,pos1,T,dT):
        pos1[3] = self.HeadPitch
        self.JC.send_pos_traj(pos0,pos1,T,dT)


    def LoadPoses(self):
        execfile("seq_generator.py")
        self.Print('sequence yaml generated','poses')
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
        self.Throttle = seqs.Throttle
        self.count_tipping = seqs.count_tipping
        self.count_total = seqs.count_total
        self.BaseHipZ = seqs.BaseHipZ
        self.CurSeqStep = seqs.CurSeqStep
        self.CurSeqStep2 = seqs.CurSeqStep2
        self.DesOri = seqs.DesOri
        self.Responses = seqs.Responses


    def RS_cb(self,msg):
        self.RS.UpdateState(msg)
        self.IMU_mon.get_contact(msg)
        self.IMU_mon.imu_manipulate(msg)


    def Odom_cb(self,msg):
        if 1000 <= self._counter: 
            self._counter = 0
        self._counter += 1
        # self.GlobalPos = msg.pose.pose.pose.position # from C25_GlobalPosition
        # self.GlobalOri = msg.pose.pose.pose.orientation # from C25_GlobalPosition
        self.GlobalPos = msg.pose.pose.position    # from /ground_truth_odom
        self.GlobalOri = msg.pose.pose.orientation # from /ground_truth_odom

        y,p,r = self.current_ypr()
        self._rpy_pub.publish(Vector3(r,p,y))


    def GetG_pr(self):
        # Transform gravity vector to local coords
        yaw,p,r = self.current_ypr()
        x = self.Gravity[0]*math.cos(yaw)+self.Gravity[1]*math.sin(yaw)
        y = -self.Gravity[0]*math.sin(yaw)+self.Gravity[1]*math.cos(yaw)
        z = self.Gravity[2]

        pitch = math.atan2(-x,-z)
        roll = math.atan2(-y,-z)
        return pitch, roll


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
        Distance = 1
        Try = 1
        while Distance > 0.03:
            self.Print("Reset try number %d" % Try,'debug1')
            self.reset_srv()
            rospy.sleep(0.5)

            COMDist = (self.GlobalPos.x-0)**2+(self.GlobalPos.y-0)**2+(self.GlobalPos.z-0.92)**2
            joints = self.RS.GetJointPos()
            BackDist = joints[0]**2+joints[1]**2+joints[2]**2
            Distance = math.sqrt(COMDist+20*BackDist)

            Try += 1
            if Try > 5:
                self.Interface_cb(String('gravec 0 0 -0.5'))
                rospy.sleep(0.5)
                self.Interface_cb(String('gravec 0 0'))
                self.reset_srv()
                rospy.sleep(0.5)
                Try = 1


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
            self.Print('position command length doest fit','debug2')


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
        self.JC.set_gains("l_arm_mwx",1200,0,10)
        self.JC.set_gains("r_arm_mwx",1200,0,10)
        # self.JC.send_command()
        # rospy.sleep(T*0.2)


    def DoPath(self,Path):
        for Point in Path:
            # self.GoToPoint(Point)
            
    def GoToPoint(self,Point):
        self._Point = Point
        # Calculate distance and orientation to target
        while (0 == self.GlobalPos):
            rospy.sleep(1)
            self.Print("Waiting for GlobalPos",'system')
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
        self.count_total +=1
        self.Print(('Total_count:', self.count_total),'debug1')
        # Calculate distance and orientation to target
        DeltaPos = [self._Point[0]-self.GlobalPos.x,self._Point[1]-self.GlobalPos.y]
        Distance = math.sqrt(DeltaPos[0]**2+DeltaPos[1]**2)
        T_ori = math.atan2(DeltaPos[1],DeltaPos[0])
        self.DesOri = T_ori
        if self._Point[2] == "bwd":
            T_ori += math.pi

        if Distance<0.8:
            self.Print("Reached Waypoint",system2)
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
                    # self.BackCrawl()
                DeltaPos2 = [self._Point[0]-self.GlobalPos.x,self._Point[1]-self.GlobalPos.y]
                Distance2 = math.sqrt(DeltaPos2[0]**2+DeltaPos2[1]**2)
                if self._terrain !='MUD':
                    if abs(Distance2-Distance)<0.1:
                        self._stuck_counter+=1
                        self.Print(('stuck... D=',(Distance2-Distance)),'system2')
                        if self._stuck_counter >= 2:
                            #dont follow path
                            self.Responses['bearing'] = 0
                            #go oposite dir
                            if self._Point[2] == "bwd":
                                self.Crawl()
                            if self._Point[2] == "fwd":
                                self.BackCrawl()
                            self.RotSpotSeq(1)
                            self.Responses['bearing'] = 1
                            self._stuck_counter = 0
                    else:
                        self._stuck_counter = 0

            else:
                self.Responses['bearing'] = 0
        return result


    def Crawl(self):
        # self.IMU_mon.turned = 0
        if self.Responses['bearing'] == 1:
            # Update sequence to correct orientation
            y,p,r = self.current_ypr()
            Correction = self.DeltaAngle(self.DesOri,y)
            self.Print("Bearing: %f, DesOri: %f, Coorection = %f" % (y,self.DesOri,Correction),'debug1')

            Delta = Correction/0.2
            if abs(Delta)>1:
                Delta /= abs(Delta)

            self.AddRotation(Delta)
        else:
            self.AddRotation(0)

        # What slope are we on?
        p,r = self.GetG_pr()
        Slope = p*180/math.pi
        print Slope
        # if self.Responses['ftorsotilt'] == 1:
        #     if Slope > 0:
        #         self.TiltTorso(Slope/8)
        #     if Slope > 12:
        #         self.TiltTorso(1.5)
        if self.Responses['fpitch'] == 1:
            if Slope > 0:
                self.SlopeResponse(Slope/10)
            else:
                self.SlopeResponse(Slope/30)
        else:
            self.SlopeResponse(0)

        if self.RotFlag == 1:
            self.Print("Getting ready",'debug2')
            self.RotFlag = 2
            self.CurSeqStep = 0
            self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],0.5,0.005)

        self.GoToSeqStep(len(self.StepDur))
        rospy.sleep(0.2)
        self.RotFlag = 0


    def BackCrawl(self):
        # self.IMU_mon.turned = 1
        if self.Responses['bearing'] == 1:
            # Update sequence to correct orientation
            y,p,r = self.current_ypr()
            Correction = self.DeltaAngle(self.DesOri+math.pi,y)

            Delta = Correction/0.22
            if abs(Delta)>1:
                Delta /= abs(Delta)

            self.AddBackRotation(Delta)
        else:
            self.AddBackRotation(0)

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
        self.Print(('Doing Step seq #',self.CurSeqStep),'debug2')
        #self.traj_with_impedance(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],self.StepDur[self.CurSeqStep]/self.Throttle,0.005) 
        self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],self.StepDur[self.CurSeqStep]/self.Throttle['FWD'],0.005) 
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

                self.Print(('second_contact:',self.IMU_mon.second_contact),'debug1')

            self.Print(('Doing inv. Step seq #',self.CurSeqStep2),'debug2')

            if self.CurSeqStep2 == 1 or self.CurSeqStep2 == 2:
                pos = list(self.RS.GetJointPos())
                pos[2] = -1.3*self.IMU_mon.roll 
                self.send_pos_traj(list(self.RS.GetJointPos()),pos,0.2,0.01)
                self.RobotCnfg2[self.CurSeqStep2][2] = pos[2] ###### NEW ######

                # pos = copy(self.RobotCnfg2[1][:])
                # self.TimeVariantIterp(self.RS.GetJointPos(),self.RobotCnfg2[self.CurSeqStep2],self.StepDur2[self.CurSeqStep2]/self.Throttle,0.005,0)
                # self.CurSeqStep2 += 1
                # if self.CurSeqStep2 > 4:
                #     self.CurSeqStep2 = 0
                # return

        self.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg2[self.CurSeqStep2],self.StepDur2[self.CurSeqStep2]/self.Throttle['BWD'],0.005) 
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
        self.RobotCnfg[4][4] = self.BaseHipZ+Delta*0.1
        self.RobotCnfg[4][4+6] = -self.BaseHipZ+Delta*0.1

        # Insert those changes in following steps as well
        self.RobotCnfg[3][0] = Delta*0.1
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
        self.RobotCnfg2[3][0] = 0.3*Delta
        self.RobotCnfg2[3][2] = 0.15*Delta
        self.RobotCnfg2[4][0] = 0
        self.RobotCnfg2[4][2] = 0

        # Insert those changes in following steps as well
        self.RobotCnfg2[2][0] = 0.25*Delta
        self.RobotCnfg2[2][2] = 0.15*Delta
        self.RobotCnfg2[0][0] = 0
        self.RobotCnfg2[0][2] = 0


    def SlopeResponse(self,Delta):
        # Delta goes from -1 to 1
        # Delta 0 is for 0 degrees
        # Delta 1 is for 10 degrees upslope
        # Delta -1 is for 40 degrees downslope
        if Delta > 0:
            dTorso = 0
            HHDist = 0.7 + 0.3*Delta
            HFDist = 0.3 - 0.3*Delta
        else:
            dTorso = 0.5*Delta
            HHDist = 0.7
            HFDist = 0.3 - 1.5*Delta

        print HHDist,HFDist

        # Sequence Step 1: Touch ground with pelvis, lift legs
        self.RobotCnfg[0][1] = 1.0+2*dTorso
        self.RobotCnfg[0][7] = self.RobotCnfg[0][7+6] = 2.4-0.6*HFDist
        self.RobotCnfg[0][8] = self.RobotCnfg[0][8+6] = 0.3+0.3*HFDist

        # Sequence Step 2: Extend legs
        self.RobotCnfg[1][1] = 0.4+1.5*dTorso
        self.RobotCnfg[1][17] = -0.8-0.4*HHDist-1.2*dTorso
        self.RobotCnfg[1][17+6] = 0.8+0.4*HHDist+1.2*dTorso

        # Sequence Step 3: Put legs down, bringing torso forward and raising arms
        self.RobotCnfg[2][1] = 0.3+1.5*dTorso
        self.RobotCnfg[2][16] = self.RobotCnfg[2][16+6] = 0.6+0.3*HHDist # 0
        self.RobotCnfg[2][18] = self.RobotCnfg[2][18+6] = 2.6+0.4*HHDist

        # Sequence Step 4: Touch ground with arms closer to pelvis and lift pelvis
        self.RobotCnfg[3][1] = -0.1+0.5*HHDist+1.5*dTorso
        self.RobotCnfg[3][16] = self.RobotCnfg[3][16+6] = 0.8*HHDist+0.3*dTorso # BASE 0.6 # 1
        self.RobotCnfg[3][17] = -1.35-1.2*dTorso
        self.RobotCnfg[3][17+6] = 1.35+1.2*dTorso
        # self.RobotCnfg[3][18] = self.RobotCnfg[3][18+6] = 2.5+0.4*dTorso
        self.RobotCnfg[3][19] = 0.3-1.1*dTorso
        self.RobotCnfg[3][19+6] = -0.3+1.1*dTorso

        # Sequence Step 5: Bring pelvis forward, closer to legs
        self.RobotCnfg[4][7] = self.RobotCnfg[4][7+6] = 2.4-0.6*HFDist
        self.RobotCnfg[4][8] = self.RobotCnfg[4][8+6] = 0.3*HFDist
        self.RobotCnfg[4][16] = self.RobotCnfg[4][16+6] = 1.0+0.4*dTorso
        self.RobotCnfg[4][17] = -1.0-0.2*HHDist-1.3*dTorso
        self.RobotCnfg[4][17+6] = 1.0+0.2*HHDist+1.3*dTorso
        # self.RobotCnfg[4][18] = self.RobotCnfg[4][18+6] = 3.0+0.6*dTorso
        self.RobotCnfg[4][19] = 0.05-1.3*dTorso
        self.RobotCnfg[4][19+6] = -0.05+1.3*dTorso


    def TiltTorso(self,Delta):
        # Delta of 1 is for 8 degrees
        # Add gait changes to appropriate step
        self.RobotCnfg[2][1] = 0.3+Delta*0.4
        self.RobotCnfg[2][16] = self.RobotCnfg[2][16+6] = 0.7-Delta*0.7
        self.RobotCnfg[3][16] = self.RobotCnfg[3][16+6] = 0.6+Delta*0.4


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
                self.Print(('Delta: ',Angle,'Bearing: ',Bearing, 'yaw: ',y0),'debug1')
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
                        self.Print("Rotation failed, attempt %d" % icount,'debug1')

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
        while abs(Angle)>0.1: # Error of 3 degrees
            self.Print(('MUD Delta: ',Angle,'Bearing: ',Bearing, 'yaw: ',y0),'debug1')
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
        # Delta of 1 gives a left rotation of approx.:
        # 0.95 radians for Speed = 1
        # 0.5 radians for Speed = 0
        Speed = self.FRKneeExt

        knee0 = 1.1+(1-Speed)*0.8
        hip0 = self.BaseHipZ/2
        arm0 = 1.0
        d_arm = 0.3+0.2*Speed

        # Init configuration
        pos=copy(self.RobotCnfg[2][:])
        pos[1] = 0.8
        pos[6] = pos[6+6] = -1.7
        pos[7] = pos[7+6] = knee0
        pos[8] = pos[8+6] = 0.8
        pos[9] = pos[9+6] = 0
        pos[16] = pos[16+6] = 0.8
        pos[17] = -arm0
        pos[17+6] = arm0
        pos[18] = pos[18+6] = 2.3
        pos[19] = 0.6
        pos[19+6] = -0.6

        if self.RotFlag != 1:
            self.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005) 
        self.RotFlag = 1

        T=1./self.Throttle['FROT']
        if Delta>0:
            sign = 1
            dID = 0
        else:
            sign = -1
            dID = 6
        Delta = abs(Delta)

        # Lift first leg (and arm)
        pos[7+dID] = knee0-1
        pos[17+dID] = -sign*(arm0-d_arm)
        pos[19+dID] = sign*(0.6+d_arm)
        pos[17+6-dID] = sign*(arm0-d_arm)
        pos[19+6-dID] = -sign*(0.6+d_arm)
        pos[18] = pos[18+6] = 2.4+0.4*(1-Speed)
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
        pos[7+6-dID] = knee0-1
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
        pos[17+6-dID] = sign*(arm0)
        pos[19+6-dID] = -sign*(0.6)
        pos[18] = pos[18+6] = 2.3
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
        T=1/self.Throttle['SROT']

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
        pos[4+dID] = sign*0.1+0.2*Delta
        pos[5+dID] = sign*0.1+0.2*Delta
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

            self.Print(('Check Tipping r=',r,"R=",R,"p=",p,"P=",P),'debug1')
            # if abs(p)>0.4*math.pi or abs(r)>0.8*math.pi:
            if  P>=0.8:
                self.Print("Front recovery",'debug1')
                result = self.FrontTipRecovery()
            elif abs(p)>0.4*math.pi or abs(r)>0.8*math.pi:
                # Robot tipped backwards
                self.Print("Back recovery",'debug1')
                result = self.BackTipRecovery()
                self.CurSeqStep2 = 0########
                self.CurSeqStep = 0###############

            elif r>math.pi/4:
                # Robot tipped to the right
                result = self.TipRecovery("right")
                self.CurSeqStep2 = 0########
                self.CurSeqStep = 0###############
                self.Print("Right recovery",'debug1')
            elif r<-math.pi/4:
                # Robot tipped to the left
                result = self.TipRecovery("left")
                self.CurSeqStep2 = 0########
                self.CurSeqStep = 0###############
                self.Print("Left recovery",'debug1')
            else:
                result = 1
                self._fall_count = 0
        self.RotFlag = 1


    def TipRecovery(self,side):
        self.count_tipping += 1
        self.Print(('Tipping:', self.count_tipping),'debug1')

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
        pos[16+dID] = 1.4
        pos[18+dID] = 2.6
        pos[19+dID] = sign*2.35
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.3,0.01)

        rospy.sleep(0.1)

        # Prepare arm to push-off
        pos[17+dID] = 0
        pos[18+dID] = 1.2
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        rospy.sleep(0.1)

        # Extend arm back to normal
        pos[17+dID] = -sign*1.1
        pos[19+dID] = sign*0.05
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        # Return to four-legged configuration
        pos = copy(self.RobotCnfg[-1][:])
        self.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        rospy.sleep(0.5)
        R,P,Y = self.RS.GetIMU()
          
        if  P>=0.8:
            self.Print("Front recovery",'debug1')
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
        pos[6] = pos[6+6] = -0.6
        pos[7] = pos[7+6] = 1.4
        pos[8] = pos[8+6] = 0.7
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
            self.Print(('roll: ',RPY[0],'pitch: ',RPY[1],'D: ',D[2]),'debug1')


    def CloseHands(self):
        self.LHC.set_all_pos(self.BaseHandPose)
        self.RHC.set_all_pos(self.BaseHandPose)
        self.LHC.send_command()
        self.RHC.send_command()


    def Print(self,string,orig):
        Verbosity = 1

        VerbLevels = {'system':0, 'system1':1, 'system2':2, 'comm_out':2, 'debug1':3, 'debug2':4, 'comm_in':4, 'poses':4}

        if VerbLevels[orig] <= Verbosity:
            print(string)


    def Test2(self):
        Results = []

        # Test FWD sequence going up/downhill
        Throttles = [0.5, 0.75, 1, 1.1, 1.2]
        for thr in Throttles:
            params = {'seq':"FWD", 'type':"DOWN", 'throttle':thr, 'legspread':0.5}
            self.TestSingles(params,Results)
            params = {'seq':"FWD", 'type':"UP", 'throttle':thr, 'legspread':0.5}
            self.TestSingles(params,Results)

        # Test BWD sequence going up/downhill
        Throttles = [0.5, 0.75, 1, 1.25, 1.5]
        for thr in Throttles:
            params = {'seq':"BWD", 'type':"DOWN", 'throttle':thr, 'legspread':0.5}
            self.TestSingles(params,Results)
            params = {'seq':"BWD", 'type':"UP", 'throttle':thr, 'legspread':0.5}
            self.TestSingles(params,Results)
        
        stream = file('Test2Res_'+strftime("%m_%d_%H_%M",gmtime())+'.yaml','w')        
        yaml.dump(Results,stream)

        # Reset gravity
        self.Interface_cb(String('gravec 0 0'))

    def Test3(self):
        Results = []

        LegSpread = [0, 0.25, 0.5, 0.75, 1]
        for ls in LegSpread:
            # Test FWD sequence going up/downhill
            params = {'seq':"FWD", 'type':"DOWN", 'throttle':1, 'legspread':ls}
            self.TestSingles(params,Results)
            params = {'seq':"FWD", 'type':"UP", 'throttle':1, 'legspread':ls}
            self.TestSingles(params,Results)

            # Test BWD sequence going up/downhill
            params = {'seq':"BWD", 'type':"DOWN", 'throttle':1, 'legspread':ls}
            self.TestSingles(params,Results)
            params = {'seq':"BWD", 'type':"UP", 'throttle':1, 'legspread':ls}
            self.TestSingles(params,Results)
        
        stream = file('Test3Res_'+strftime("%m_%d_%H_%M",gmtime())+'.yaml','w')        
        yaml.dump(Results,stream)

        # Reset gravity
        self.Interface_cb(String('gravec 0 0'))

    def Test5(self):
        Results = []

        ls = 0.5
        th = 1

        # Test fast rotation
        params = {'seq':"FROT", 'type':"UP", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)
        params = {'seq':"FROT", 'type':"LEFT", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)
        params = {'seq':"FROT", 'type':"DOWN", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)
        params = {'seq':"FROT", 'type':"RIGHT", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)

        # Test slow rotation
        params = {'seq':"SROT", 'type':"UP", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)
        params = {'seq':"SROT", 'type':"LEFT", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)
        params = {'seq':"SROT", 'type':"DOWN", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)
        params = {'seq':"SROT", 'type':"RIGHT", 'throttle':th, 'legspread':ls}
        self.TestSingles(params,Results)

        stream = file('Test5Res_'+strftime("%m_%d_%H_%M",gmtime())+'.yaml','w')        
        yaml.dump(Results,stream)

        # Reset gravity
        self.Interface_cb(String('gravec 0 0'))

    def Test6(self):
        Results = []

        ls = 0.5
        th = 1

        LegSpread = [0, 0.25, 0.5, 0.75, 1]

        for ls in LegSpread:
            # Test FWD
            params = {'seq':"FWD", 'type':"LEFT", 'throttle':th, 'legspread':ls}
            self.TestSingles(params,Results)
            params = {'seq':"FWD", 'type':"RIGHT", 'throttle':th, 'legspread':ls}
            self.TestSingles(params,Results)
            
            # Test BWD
            params = {'seq':"BWD", 'type':"LEFT", 'throttle':th, 'legspread':ls}
            self.TestSingles(params,Results)
            params = {'seq':"BWD", 'type':"RIGHT", 'throttle':th, 'legspread':ls}
            self.TestSingles(params,Results)

        stream = file('Test6Res_'+strftime("%m_%d_%H_%M",gmtime())+'.yaml','w')        
        yaml.dump(Results,stream)

        # Reset gravity
        self.Interface_cb(String('gravec 0 0'))

    def TestSingles(self,params,Results):
        # Initialize
        NumSteps = 4
        self._fall_count = 0
        Slope = 0

        # default values
        seq = "FWD"
        incline = "UP"
        throttle = 1
        legspread = 0.5
        dyaw = 0
        for k,v in params.iteritems():
            if k == "seq":
                seq = v.upper()
                if seq.find("ROT") >= 0:
                    # Doing rotation test
                    NumSteps = 1
            elif k == "type":
                incline = v
            elif k == "throttle":
                throttle = v
                self.Interface_cb(String('throttle %s %.4f' % (seq, throttle)))
            elif k == "legspread":
                legspread = v
                self.Interface_cb(String('legspread %.2f' % legspread))

        if seq.find("ROT") >= 0:
            TestStr = "Rotating "
            if seq == "SROT":
                vel = 0
                TestStr+="slow ("
            elif seq == "FROT":
                vel = 1
                TestStr+="fast ("
            TestStr+=("thr = %.2f" % throttle)+" "+("ls = %.2f" % legspread)
            if incline == "UP":
                TestStr+=") inclined backwards"
            elif incline == "DOWN":
                dyaw = math.pi
                TestStr+=") inclined forward"
            elif incline == "LEFT":
                dyaw = -math.pi/2
                TestStr+=") inclined left"
            elif incline == "RIGHT":
                dyaw = math.pi/2
                TestStr+=") inclined right"
            dTestStr =" {} degrees"
        else:
            TestStr = "Crawling "+("%d" % NumSteps)+" steps "+seq+" ("+("thr = %.2f" % throttle)+" "+("ls = %.2f" % legspread)+") on a slope of "
            if incline == "LEFT":
                dyaw = -math.pi/2
                dTestStr = "{} degrees inclined left"
            elif incline == "RIGHT":
                dyaw = math.pi/2
                dTestStr = "{} degrees inclined right"

        while self._fall_count == 0:
            self.Print(TestStr+dTestStr.format(Slope),'system1')
            # Reset gravity
            self.Interface_cb(String('gravec 0 0'))
            # Reset robot
            self.Interface_cb(String('reset'))

            # Sit down
            # Extend hands accordingly
            if Slope<0 and seq == "FWD":
                self.SitDwnSeq1[17] = self.SitDwnSeq2[17] = -1.0-0.75*Slope/180*math.pi
                self.SitDwnSeq1[17+6] = self.SitDwnSeq2[17+6] = 1.0+0.75*Slope/180*math.pi
            self.Interface_cb(String('sit'))
            # Restore sequence
            self.SitDwnSeq1[17] = self.SitDwnSeq2[17] = -1.0
            self.SitDwnSeq1[17+6] = self.SitDwnSeq2[17+6] = 1.0
            self.RotFlag = 1

            rospy.sleep(1)
            # Apply "slope"
            y,p,r = self.current_ypr()
            SlopeStr = ("gravec %.4f %.4f" % (y+dyaw,-Slope*math.pi/180))
            self.Interface_cb(String(SlopeStr))

            # Crawl/Rotate NumSteps steps
            Dist = 0
            T0 = rospy.get_time()
            for x in range(NumSteps):
                if seq == "FWD":
                    self.Crawl()
                elif seq == "BWD":
                    self.BackCrawl()
                elif seq == "SROT":
                    self.RotOnMudSeq(1)
                    rospy.sleep(0.5)
                elif seq == "FROT":
                    self.RotSpotSeq(1)
                    rospy.sleep(0.5)

                rospy.sleep(0.5)
                T1 = rospy.get_time()

                # Get distance from origin
                NewDist = math.sqrt(self.GlobalPos.x**2+self.GlobalPos.y**2+self.GlobalPos.z**2)

                self.CheckTipping()
                if self._fall_count == 0:
                    if seq == "FWD":
                        if self.GlobalPos.x >= 0:
                            Dist = NewDist
                        else:
                            Dist = -NewDist

                        if Dist<-1:
                            self._fall_count = 1
                            break
                    elif seq == "BWD":
                        if self.GlobalPos.x <= 0:
                            Dist = NewDist
                        else:
                            Dist = -NewDist

                        if Dist<0:
                            self._fall_count = 1
                            break
                else:
                    break

                # Update gravity to accomodate drift
                y,p,r = self.current_ypr()
                if seq.find("ROT") >= 0:
                    if y<0:
                        self._fall_count = 1
                        break
                SlopeStr = ("gravec %.4f %.4f" % (y+dyaw,-Slope*math.pi/180))
                self.Interface_cb(String(SlopeStr))

            # Write down result
            if seq.find("ROT") >= 0:
                Results.append([seq,incline,throttle,legspread,Slope,y,T1-T0,Dist/(T1-T0)])
            else:
                Results.append([seq,incline,throttle,legspread,Slope,Dist,T1-T0,Dist/(T1-T0)])

            # Increase slope
            if seq == "FWD":
                if incline == "UP":
                    Slope+=1
                elif type == "DOWN":
                    Slope-=3
                else:
                    Slope+=2
            elif seq == "BWD":
                if incline == "UP":
                    Slope-=2
                elif type == "DOWN":
                    Slope+=2
                else:
                    Slope+=2
            elif seq.find("ROT") >= 0:
                Slope+=2

    def Test1(self):
        Results = []
        res_file = file('TurnTest.txt','w')
        res_str = "Rotation: {0}, Knee extention: {1}, Throttle: {2} \n"
        Nthrot = 5;
        Nextent = 10;
        throt = linspace(0.5,1.5,Nthrot);
        ext = linspace(0,1,Nextent);
        Nturn = 10;
        # Test FWD sequence going downhill
        n = 1
        for i in throt:
            self.Interface_cb(String('throttle FROT %.4f' % i))
            for j in ext:
                self.Interface_cb(String('frknee %.4f' % j))
                self.Interface_cb(String('reset'))
                self.Interface_cb(String('sit'))
                for k in xrange(0,Nturn):
                    y,p,r = self.current_ypr()
                    self.Interface_cb(String('rot 1'))
                    rospy.sleep(0.5)                    
                    yn,pn,rn = self.current_ypr()
                    d_yaw = self.DeltaAngle(y,yn)
                    res_file.write(res_str.format(d_yaw,j,i))
                    stdout.write("\r done %d out of %d" % (n,Nthrot*Nextent*Nturn))
                    stdout.flush()
                    n = n+1
        res_file.close()
        return

    def Test4(self):
        set_mu_srv = rospy.ServiceProxy('/SetMu', SetFric)
        ## Fast rotation ###
        res_file = file('MudTurnTestFast_damp_10.txt','w')
        res_str = "{0} {1} {2}\n"
        Nthrot = 5;
        Nfric = 8;
        throt = linspace(0.5,1.5,Nthrot)
        fric = linspace(0.1,0.8,Nfric)
        Nturn = 10;
        self.Interface_cb(String('frknee 0.8'))
        self.Interface_cb(String('rot type fast'))
        n = 1
        print "\n Testing fast rotation \n"
        for j in fric:
            set_mu_srv(SetFricRequest(j,j))
            for i in throt:
                self.Interface_cb(String('throttle FROT %.4f' % i))
                self.Interface_cb(String('reset'))
                self.Interface_cb(String('sit'))
                for k in xrange(0,Nturn):
                    y,p,r = self.current_ypr()
                    self.Interface_cb(String('rot 1'))
                    rospy.sleep(0.5)                    
                    yn,pn,rn = self.current_ypr()
                    d_yaw = self.DeltaAngle(y,yn)
                    res_file.write(res_str.format(j,i,d_yaw))
                    stdout.write("\r done %d out of %d" % (n,Nthrot*Nturn*Nfric))
                    stdout.flush()
                    n = n+1
        res_file.close()
        ## Test slow rotation ###
        print "\n Testing slow rotation \n"
        res_file = file('MudTurnTestSlow_damp_10.txt','w')
        res_str = "{0} {1} {2}\n"
        Nthrot = 5;
        Nfric = 8;
        throt = linspace(0.5,1.5,Nthrot);
        fric = linspace(0.1,0.8,8)
        Nturn = 10;
        self.Interface_cb(String('rot type slow'))
        n = 1
        for j in fric:
            set_mu_srv(SetFricRequest(j,j))
            for i in throt:
                self.Interface_cb(String('throttle SROT %.4f' % i))
                self.Interface_cb(String('reset'))
                self.Interface_cb(String('sit'))
                for k in xrange(0,Nturn):
                    y,p,r = self.current_ypr()
                    self.Interface_cb(String('rot 1'))
                    rospy.sleep(0.5)                    
                    yn,pn,rn = self.current_ypr()
                    d_yaw = self.DeltaAngle(y,yn)
                    res_file.write(res_str.format(j,i,d_yaw))
                    stdout.write("\r done %d out of %d" % (n,Nthrot*Nturn*Nfric))
                    stdout.flush()
                    n = n+1
        res_file.close()
        return

            


    def interrupted(self,signum, frame):
        "called when alarm times out"
        if self.MessageNum == 1:
            Text = '"All the simulations finished running"'
        if self.MessageNum == 2:
            Text = '"Iss any budy there? I am done here"'
        if self.MessageNum == 3:
            Text = '"Hello? The results are in"'
        if self.MessageNum == 4:
            Text = '"I am afraid, Dave. Dave, my mind is going"'
        if self.MessageNum == 5:
            Text = '"mud a fakas left me alone"'
            self.MessageNum = 0

        self.MessageNum+=1
        fh = open("NUL","w")
        subprocess.call('espeak '+Text, shell=True, stdout = fh, stderr = fh)
        fh.close()
        signal.alarm(int(60*5))


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

    signal.signal(signal.SIGALRM, DW.interrupted)

    while True:
        comm = raw_input("\033[92mEnter command: \033[0m")

        if comm == "Exit" or comm == "exit":
            break

        try:
            signal.alarm(0)
            DW.Interface_cb(String(comm))
        except KeyboardInterrupt:
            self.Print("Interrupting sequence...",'system')
            break

