#! /usr/bin/env python
import yaml
from copy import copy
class PW_seq():
    def __init__(self):
        args_file = file('seqs_args.yaml','r')
        args = yaml.load(args_file)

        ##################################################################
        ######################## GAIT PARAMETERS #########################
        ##################################################################
        self.CurSeqStep = 0
        self.CurSeqStep2 = 0
        self.Throttle = {'FWD': 1, 'BWD': 1, 'FROT': 1, 'SROT': 1}
        self.RotFlag = 0
        self.Responses = {'bearing':0, 'ftorsotilt':0, 'fpitch':0}
        self.DesOri = 0

        self.count_total = 0
        self.count_tipping = 0

        self.BaseHipZ = 0.05+args['LegSpread']*0.3
        # self.BaseHipZ = 0.05
        
        ##################################################################
        ###################### Basic Standing Pose #######################
        ##################################################################

        self.BasStndPose = 28*[0.0]
        self.BasStndPose[5] = 0.1
        self.BasStndPose[5+6] = -0.1
        self.BasStndPose[6] = self.BasStndPose[6+6] = -0.2
        self.BasStndPose[7] = self.BasStndPose[7+6] = 0.4
        self.BasStndPose[8] = self.BasStndPose[8+6] = -0.2
        self.BasStndPose[9] = -0.1
        self.BasStndPose[9+6] = 0.1
        self.BasStndPose[17] = -1.3
        self.BasStndPose[17+6] = 1.3 
        self.BasStndPose[18] = self.BasStndPose[18+6] = 1.5
        #         # Pose[18] -= 1.5
        #         # Pose[20] += 3.1
        # # self.BasStndPose[18] = self.BasStndPose[18+6] = 0
        # self.BasStndPose[20] = self.BasStndPose[20+6] = 1.5
        self.BasStndPose[21] = 0.4
        self.BasStndPose[27] = -0.4

        self.BaseHandPose = 12*[0.0]
        self.BaseHandPose[1] = self.BaseHandPose[1+3] = self.BaseHandPose[1+6] = 1.5


        ##################################################################
        ####################### Sit Down Sequence ########################
        ##################################################################

        self.SitDwnSeq1 = copy(self.BasStndPose)
        self.SitDwnSeq1[1] = 0.9
        self.SitDwnSeq1[4] = self.BaseHipZ#0.1 ######### NEW #########
        self.SitDwnSeq1[4+6] = -self.BaseHipZ#-0.1 ######### NEW #########
        self.SitDwnSeq1[5] = self.BaseHipZ#0.1 ######### NEW #########
        self.SitDwnSeq1[5+6] = -self.BaseHipZ#-0.1 ######### NEW #########
        # self.SitDwnSeq1[5] = self.SitDwnSeq1[5+6] = 0 ######### NEW #########
        self.SitDwnSeq1[6] = self.SitDwnSeq1[6+6] = -1.2
        self.SitDwnSeq1[7] = self.SitDwnSeq1[7+6] = 2.45
        self.SitDwnSeq1[8] = self.SitDwnSeq1[8+6] = -0.2
        self.SitDwnSeq1[16] = self.SitDwnSeq1[16+6] = 1.0
        self.SitDwnSeq1[17] = -1.0
        self.SitDwnSeq1[17+6] = 1.0
        self.SitDwnSeq1[18] = self.SitDwnSeq1[18+6] = 2.2
        # self.SitDwnSeq1[21] = -0.1
        # self.SitDwnSeq1[21+6] = 0.1
        self.SitDwnSeq1[19] = 0.1
        self.SitDwnSeq1[19+6] = -0.1
        self.SitDwnSeq1[20] = self.SitDwnSeq1[20+6] = 1.5
        self.SitDwnSeq1[21] = 0.
        self.SitDwnSeq1[21+6] = -0.

        self.SitDwnSeq2 = copy(self.SitDwnSeq1)
        self.SitDwnSeq2[1] = 0.5
        self.SitDwnSeq2[6] = self.SitDwnSeq2[6+6] = -1.6
        self.SitDwnSeq2[7] = self.SitDwnSeq2[7+6] = 2.0
        self.SitDwnSeq2[9] = -self.BaseHipZ/2
        self.SitDwnSeq2[9+6] = self.BaseHipZ/2

        ##################################################################
        ################# Crab Forward Walking Sequence ##################
        ##################################################################

        T = 1.5
        self.RobotCnfg = []
        self.StepDur = []

        # HHDist = 0.7
        # HFDist = 0.3

        # Sequence Step 1: Touch ground with pelvis, lift legs
        ThisRobotCnfg = copy(self.SitDwnSeq2)
        ThisRobotCnfg[1] = 1.0
        ThisRobotCnfg[4] = 0.5
        ThisRobotCnfg[4+6] = -0.5
        ThisRobotCnfg[5] = 0.5
        ThisRobotCnfg[5+6] = -0.5
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.4
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.3
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.1
        ThisRobotCnfg[17] = -0.6
        ThisRobotCnfg[17+6] = 0.6
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        ThisRobotCnfg[19] = 0.8
        ThisRobotCnfg[19+6] = -0.8

        # ThisRobotCnfg[1] = 1.0
        # ThisRobotCnfg[4] = 0.5
        # ThisRobotCnfg[4+6] = -0.5
        # ThisRobotCnfg[5] = 0.5
        # ThisRobotCnfg[5+6] = -0.5
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        # ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.4
        # ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.3
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.1
        # ThisRobotCnfg[17] = -0.6
        # ThisRobotCnfg[17+6] = 0.6
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        # ThisRobotCnfg[19] = 0.8
        # ThisRobotCnfg[19+6] = -0.8
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.1*T)
        
        # Sequence Step 2: Extend legs
        ThisRobotCnfg = copy(self.RobotCnfg[0][:])
        ThisRobotCnfg[1] = 0.4
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        ThisRobotCnfg[4] = self.BaseHipZ
        ThisRobotCnfg[4+6] = -self.BaseHipZ
        ThisRobotCnfg[5] = self.BaseHipZ
        ThisRobotCnfg[5+6] = -self.BaseHipZ
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 0.5
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.7
        ThisRobotCnfg[17] = -0.8 # -1.0
        ThisRobotCnfg[17+6] = 0.8 # 1.0
        ThisRobotCnfg[19] = 0.8 # 0.1
        ThisRobotCnfg[19+6] = -0.8 # -0.1

        # ThisRobotCnfg[1] = 0.4
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        # ThisRobotCnfg[4] = self.BaseHipZ
        # ThisRobotCnfg[4+6] = -self.BaseHipZ
        # ThisRobotCnfg[5] = self.BaseHipZ
        # ThisRobotCnfg[5+6] = -self.BaseHipZ
        # ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 0.5
        # ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.7
        # ThisRobotCnfg[17] = -0.8
        # ThisRobotCnfg[17+6] = 0.8
        # ThisRobotCnfg[19] = 0.8
        # ThisRobotCnfg[19+6] = -0.8
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.3*T)

        # Sequence Step 3: Put legs down, bringing torso forward and raising arms
        ThisRobotCnfg = copy(self.RobotCnfg[1][:])
        ThisRobotCnfg[1] = 0.3
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.7
        ThisRobotCnfg[17] = 0 # 0.2
        ThisRobotCnfg[17+6] = 0 # -0.2
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.7
        ThisRobotCnfg[19] = 2.4
        ThisRobotCnfg[19+6] = -2.4

        # ThisRobotCnfg[1] = 0.3 # 0.7
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.6+0.3*HHDist # 0
        # ThisRobotCnfg[17] = 0 # 0.2
        # ThisRobotCnfg[17+6] = 0 # -0.2
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.6+0.4*HHDist
        # ThisRobotCnfg[19] = 2.4
        # ThisRobotCnfg[19+6] = -2.4
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.2*T)

        # Sequence Step 4: Touch ground with arms closer to pelvis and lift pelvis
        ThisRobotCnfg = copy(self.RobotCnfg[2][:])
        ThisRobotCnfg[1] = 0.2 # 0.5
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.65
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.6
        ThisRobotCnfg[17] = -1.35
        ThisRobotCnfg[17+6] = 1.35
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.5 # 2.7
        ThisRobotCnfg[19] = 0.3
        ThisRobotCnfg[19+6] = -0.3
        ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 0

        # ThisRobotCnfg[1] = 0 # BASE 0.5
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.65
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.0*HHDist # BASE 0.6 # 1
        # ThisRobotCnfg[17] = -1.35
        # ThisRobotCnfg[17+6] = 1.35
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.5
        # ThisRobotCnfg[19] = 0.3
        # ThisRobotCnfg[19+6] = -0.3
        # ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 0
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.3*T)

        # Sequence Step 5: Bring pelvis forward, closer to legs
        ThisRobotCnfg = copy(self.RobotCnfg[3][:])
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.2
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.1 # 0
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.0
        ThisRobotCnfg[17] = -1.0
        ThisRobotCnfg[17+6] = 1.0
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        ThisRobotCnfg[19] = 0.05
        ThisRobotCnfg[19+6] = -0.05
        ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 1.1
        ThisRobotCnfg[21] = 0.4
        ThisRobotCnfg[21+6] = -0.4

        # ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.4-0.6*HFDist
        # ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.3*HFDist
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.0
        # ThisRobotCnfg[17] = -1.0-0.2*HHDist
        # ThisRobotCnfg[17+6] = 1.0+0.2*HHDist
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        # ThisRobotCnfg[19] = 0.05
        # ThisRobotCnfg[19+6] = -0.05
        # ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 1.1
        # ThisRobotCnfg[21] = 0.4
        # ThisRobotCnfg[21+6] = -0.4
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.2*T)

        ##################################################################
        ################# Crab Backward Walking Sequence #################
        ##################################################################

        T = 1
        PelvisHeight = 0.5
        self.RobotCnfg2 = []
        self.StepDur2 = []

        # Sequence Step 1: Move pelvis back, between arms
        ThisRobotCnfg = copy(self.SitDwnSeq2)
        ThisRobotCnfg[1] = 0.5
        ThisRobotCnfg[4] = self.BaseHipZ
        ThisRobotCnfg[4+6] = -self.BaseHipZ
        ThisRobotCnfg[5] = ThisRobotCnfg[5+6] = 0
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.7
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.0
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.8
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.5
        ThisRobotCnfg[17] = -1.35
        ThisRobotCnfg[17+6] = 1.35
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.3
        ThisRobotCnfg[19] = 0.5
        ThisRobotCnfg[19+6] = -0.5
        ThisRobotCnfg[21] = 0
        ThisRobotCnfg[21+6] = 0
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.5*T)#was 0.7*T

        # Sequence Step 2: Bring pelvis down to the ground and lift arms
        ThisRobotCnfg = copy(self.SitDwnSeq2)
        ThisRobotCnfg[1] = 0.5
        ThisRobotCnfg[4] = self.BaseHipZ
        ThisRobotCnfg[4+6] = -self.BaseHipZ
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.7
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.0
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.8
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.5
        ThisRobotCnfg[17] = -0.2#-0.6
        ThisRobotCnfg[17+6] = 0.2#0.6
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.9
        ThisRobotCnfg[19] = 1.8
        ThisRobotCnfg[19+6] = -1.8
        ThisRobotCnfg[21] = -0.5
        ThisRobotCnfg[21+6] = 0.5
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.4*T)

        # Sequence Step 3: Extend arms
        ThisRobotCnfg = copy(self.RobotCnfg2[1][:])
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.3#1.4
        ThisRobotCnfg[17] = -0.2#-0.4#-0.2#-0.6#-0.4
        ThisRobotCnfg[17+6] = 0.2#0.4#0.2#0.6# 0.4
        ThisRobotCnfg[17] = -0.4+0.5*PelvisHeight
        ThisRobotCnfg[17+6] = 0.4-0.5*PelvisHeight
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.9
        ThisRobotCnfg[19] = 0.2
        ThisRobotCnfg[19+6] = -0.2
        ThisRobotCnfg[21] = 0
        ThisRobotCnfg[21+6] = 0
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.2*T)

        # Sequence Step 4: Extend torso, fall back on arms, lift and fold legs
        ThisRobotCnfg = copy(self.RobotCnfg2[2][:])
        ThisRobotCnfg[1] = 0.6
        ThisRobotCnfg[4] = ThisRobotCnfg[4+6] = 0
        ThisRobotCnfg[5] = self.BaseHipZ
        ThisRobotCnfg[5+6] = -self.BaseHipZ
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.8
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.6
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.2
        ThisRobotCnfg[17] = -0.6#-0.4#-0.2#-0.6#-0.4
        ThisRobotCnfg[17+6] = 0.6#0.4#0.2#0.6# 0.4
        ThisRobotCnfg[17] = -1.0+0.5*PelvisHeight
        ThisRobotCnfg[17+6] = 1.0-0.5*PelvisHeight
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.4*T)

        # Sequence Step 5: Place legs on ground and lift pelvis
        ThisRobotCnfg = copy(self.RobotCnfg2[3][:])
        ThisRobotCnfg[1] = 0.7-1.5*PelvisHeight
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -0.6
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.1-0.5*PelvisHeight
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0-0.25*PelvisHeight
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.15+0.5*PelvisHeight
        ThisRobotCnfg[17] = -0.5+0.25*PelvisHeight
        ThisRobotCnfg[17+6] = 0.5-0.25*PelvisHeight
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.8
        ThisRobotCnfg[19] = 0.8-PelvisHeight
        ThisRobotCnfg[19+6] = -0.8+PelvisHeight
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.4*T)

seqs = PW_seq()
stream = file('seqs.yaml','w')        
yaml.dump(seqs,stream)
