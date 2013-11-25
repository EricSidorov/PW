#! /usr/bin/env python
import yaml
from copy import copy
class PW_seq():
    def __init__(self,args):
        # args_file = file('seqs_args.yaml','r')
        # args = yaml.load(args_file)
        

        ##################################################################
        ######################## GAIT PARAMETERS #########################
        ##################################################################

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
        self.SitDwnSeq1[6] = self.SitDwnSeq1[6+6] = -1.5
        self.SitDwnSeq1[7] = self.SitDwnSeq1[7+6] = 2.0
        self.SitDwnSeq1[8] = self.SitDwnSeq1[8+6] = -0.4
        self.SitDwnSeq1[16] = self.SitDwnSeq1[16+6] = 1.05
        self.SitDwnSeq1[17] = -0.6
        self.SitDwnSeq1[17+6] = 0.6
        self.SitDwnSeq1[18] = self.SitDwnSeq1[18+6] = 2.5
        # self.SitDwnSeq1[21] = -0.1
        # self.SitDwnSeq1[21+6] = 0.1
        self.SitDwnSeq1[19] = 0.0
        self.SitDwnSeq1[19+6] = -0.0
        self.SitDwnSeq1[20] = self.SitDwnSeq1[20+6] = 1.5
        self.SitDwnSeq1[21] = 0.
        self.SitDwnSeq1[21+6] = -0.

        self.SitDwnSeq2 = copy(self.SitDwnSeq1)
        self.SitDwnSeq2[1] = 0.5
        self.SitDwnSeq2[6] = self.SitDwnSeq2[6+6] = -1.6
        self.SitDwnSeq2[7] = self.SitDwnSeq2[7+6] = 2.0
        self.SitDwnSeq2[8] = self.SitDwnSeq2[8+6] = -0.2
        self.SitDwnSeq2[9] = -self.BaseHipZ/2
        self.SitDwnSeq2[9+6] = self.BaseHipZ/2
        self.SitDwnSeq2[16] = self.SitDwnSeq2[16+6] = 1.0
        self.SitDwnSeq2[17] = -1.0
        self.SitDwnSeq2[17+6] = 1.
        self.SitDwnSeq2[18] = self.SitDwnSeq2[18+6] = 2.2
        self.SitDwnSeq2[19] = 0.1
        self.SitDwnSeq2[19+6] = -0.1

        ##################################################################
        ################# Crab Forward Walking Sequence ##################
        ##################################################################

        T = 1.5
        self.RobotCnfg = []
        self.StepDur = []

        # Slope should be in deg and non-positive
        nSlope = args['Slope']/50.0 # Normalized slope 0 -> -1
        if nSlope>0:
            nSlope = 0
        elif nSlope<-1:
            nSlope = -1

        # Sequence Step 1: Touch ground with pelvis, lift legs
        ThisRobotCnfg = copy(self.SitDwnSeq2)
        ThisRobotCnfg[1] = 1.0 + 0.55*nSlope # 1.0 -> 0.45
        ThisRobotCnfg[4] = 0.5
        ThisRobotCnfg[4+6] = -0.5
        ThisRobotCnfg[5] = 0.5
        ThisRobotCnfg[5+6] = -0.5
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75 - 0.6*nSlope # -1.75 -> -1.15
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.4 + 1.0*nSlope # 2.4 -> 1.4
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.3 - 0.2*nSlope # 0.3 -> 0.5
        ThisRobotCnfg[9] = -self.BaseHipZ/2 * (1+nSlope) # -self.BaseHipZ/2 -> 0
        ThisRobotCnfg[9+6] = self.BaseHipZ/2 * (1+nSlope) # -self.BaseHipZ/2 -> 0
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.1 - 0.4*nSlope # 1.1 -> 1.5
        ThisRobotCnfg[17] = -0.8 - 1.6*nSlope # -0.8 -> 0.8
        ThisRobotCnfg[17+6] = 0.8 + 1.6*nSlope # 0.8 -> -0.8
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0 + 1.0*nSlope # 3.0 -> 2.0
        ThisRobotCnfg[19] = 0.8 - 0.9*nSlope # 0.8 -> 1.7
        ThisRobotCnfg[19+6] = -0.8 + 0.9*nSlope # -0.8 -> -1.7
        ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 1.5 + 2.7*nSlope # 1.5 -> -1.2
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.1*T)

        # Sequence Step 2: Extend legs
        ThisRobotCnfg = copy(self.RobotCnfg[0][:])
        ThisRobotCnfg[1] = 0.4 - 0.05*nSlope # 0.45
        ThisRobotCnfg[4] = self.BaseHipZ+(self.BaseHipZ-0.2)*nSlope # 0.2
        ThisRobotCnfg[4+6] = -self.BaseHipZ-(self.BaseHipZ-0.2)*nSlope # -0.2
        ThisRobotCnfg[5] = self.BaseHipZ+(self.BaseHipZ-0.3)*nSlope # 0.3
        ThisRobotCnfg[5+6] = -self.BaseHipZ-(self.BaseHipZ-0.3)*nSlope # -0.3
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75 - 1.05*nSlope # -0.7
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 0.5 - 0.1*nSlope # 0.6
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.7 + 0.2*nSlope # 0.5
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.3*T)

        # Sequence Step 3: Put legs down, bringing torso forward and raising arms
        ThisRobotCnfg = copy(self.RobotCnfg[1][:])
        ThisRobotCnfg[1] = 0.3 - 0.15*nSlope # 0.45
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75 - 1.35*nSlope # -0.4
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.7 - 0.1*nSlope # 0.8
        ThisRobotCnfg[17] = 0 - 0.8*nSlope # 0.8
        ThisRobotCnfg[17+6] = 0 + 0.8*nSlope # -0.8
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.7 + 0.5*nSlope # 2.2
        ThisRobotCnfg[19] = 2.4 + 0.3*nSlope # 2.1
        ThisRobotCnfg[19+6] = -2.4 -0.3*nSlope # -2.1
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.2*T)

        # Sequence Step 4: Touch ground with arms closer to pelvis and lift pelvis
        ThisRobotCnfg = copy(self.RobotCnfg[2][:])
        ThisRobotCnfg[1] = 0.2
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.65 - 1.45*nSlope # -0.2
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.6 - 0.2*nSlope # 0.8
        ThisRobotCnfg[17] = -1.35 - 2.05*nSlope # 0.7
        ThisRobotCnfg[17+6] = 1.35 + 2.05*nSlope # -0.7
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.5 - 0.3*nSlope # 2.8
        ThisRobotCnfg[19] = 0.3 - 1.3*nSlope # 1.6
        ThisRobotCnfg[19+6] = -0.3 + 1.3*nSlope # -1.6
        ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 0 +1.2*nSlope # -1.2
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append((0.3-nSlope/3)*T) # 1.3*T

        # Sequence Step 5: Bring pelvis forward, closer to legs
        ThisRobotCnfg = copy(self.RobotCnfg[3][:]) 
        ThisRobotCnfg[5] = self.BaseHipZ + (self.BaseHipZ-0.5)*nSlope # 0.5
        ThisRobotCnfg[5+6] = -self.BaseHipZ - (self.BaseHipZ-0.5)*nSlope # -0.5
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.65 - 1.05*nSlope # -0.6
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.2 + 0.8*nSlope # 1.4
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.1 - 0.6*nSlope # 0.7
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.0 + 0.1*nSlope # 0.9
        ThisRobotCnfg[17] = -1.0 - 2.2*nSlope # 1.2
        ThisRobotCnfg[17+6] = 1.0 + 2.2*nSlope # -1.2
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        ThisRobotCnfg[19] = 0.05 - 1.65*nSlope # 1.7
        ThisRobotCnfg[19+6] = -0.05 + 1.65*nSlope # -1.7
        ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 1.1 + 2.6*nSlope # -1.5
        ThisRobotCnfg[21] = 0.4 # ?
        ThisRobotCnfg[21+6] = -0.4 # ?
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.2*T)

        # # Sequence Step 1: Touch ground with pelvis, lift legs
        # ThisRobotCnfg = copy(self.SitDwnSeq2)
        # ThisRobotCnfg[1] = 1.0
        # ThisRobotCnfg[4] = 0.5
        # ThisRobotCnfg[4+6] = -0.5
        # ThisRobotCnfg[5] = 0.5
        # ThisRobotCnfg[5+6] = -0.5
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        # ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.4
        # ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.3
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.1
        # ThisRobotCnfg[17] = -0.8 # -0.6
        # ThisRobotCnfg[17+6] = 0.8 # 0.6
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        # ThisRobotCnfg[19] = 0.8
        # ThisRobotCnfg[19+6] = -0.8
        # self.RobotCnfg.append(ThisRobotCnfg)
        # self.StepDur.append(0.1*T)
        
        # # Sequence Step 2: Extend legs
        # ThisRobotCnfg = copy(self.RobotCnfg[0][:])
        # ThisRobotCnfg[1] = 0.4
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        # ThisRobotCnfg[4] = self.BaseHipZ
        # ThisRobotCnfg[4+6] = -self.BaseHipZ
        # ThisRobotCnfg[5] = self.BaseHipZ
        # ThisRobotCnfg[5+6] = -self.BaseHipZ
        # ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 0.5
        # ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.7
        # ThisRobotCnfg[17] = -0.8 # -1.0
        # ThisRobotCnfg[17+6] = 0.8 # 1.0
        # ThisRobotCnfg[19] = 0.8 # 0.1
        # ThisRobotCnfg[19+6] = -0.8 # -0.1
        # self.RobotCnfg.append(ThisRobotCnfg)
        # self.StepDur.append(0.3*T)

        # # Sequence Step 3: Put legs down, bringing torso forward and raising arms
        # ThisRobotCnfg = copy(self.RobotCnfg[1][:])
        # ThisRobotCnfg[1] = 0.3
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.75
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.7
        # ThisRobotCnfg[17] = 0 # 0.2
        # ThisRobotCnfg[17+6] = 0 # -0.2
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.7
        # ThisRobotCnfg[19] = 2.4
        # ThisRobotCnfg[19+6] = -2.4
        # self.RobotCnfg.append(ThisRobotCnfg)
        # self.StepDur.append(0.2*T)

        # # Sequence Step 4: Touch ground with arms closer to pelvis and lift pelvis
        # ThisRobotCnfg = copy(self.RobotCnfg[2][:])
        # ThisRobotCnfg[1] = 0.2 # 0.5
        # ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.65
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.6
        # ThisRobotCnfg[17] = -1.35
        # ThisRobotCnfg[17+6] = 1.35
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.5 # 2.7
        # ThisRobotCnfg[19] = 0.3
        # ThisRobotCnfg[19+6] = -0.3
        # ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 0
        # self.RobotCnfg.append(ThisRobotCnfg)
        # self.StepDur.append(0.3*T)

        # # Sequence Step 5: Bring pelvis forward, closer to legs
        # ThisRobotCnfg = copy(self.RobotCnfg[3][:])
        # ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.2
        # ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.1 # 0
        # ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.0
        # ThisRobotCnfg[17] = -1.0
        # ThisRobotCnfg[17+6] = 1.0
        # ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        # ThisRobotCnfg[19] = 0.05
        # ThisRobotCnfg[19+6] = -0.05
        # ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = 1.1
        # ThisRobotCnfg[21] = 0.4
        # ThisRobotCnfg[21+6] = -0.4
        # self.RobotCnfg.append(ThisRobotCnfg)
        # self.StepDur.append(0.2*T)

        ##################################################################
        ################# Crab Backward Walking Sequence #################
        ##################################################################

        T = 1
        PelvisHeight = args['PelvisHeight']
        self.RobotCnfg2 = []
        self.StepDur2 = []

        # Sequence Step 1: Move pelvis back, between arms
        ThisRobotCnfg = copy(self.SitDwnSeq2)
        ThisRobotCnfg[1] = 0.5
        ThisRobotCnfg[4] = self.BaseHipZ
        ThisRobotCnfg[4+6] = -self.BaseHipZ
        ThisRobotCnfg[5] = self.BaseHipZ
        ThisRobotCnfg[5+6] = -self.BaseHipZ
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
        ThisRobotCnfg[17] = 0.1#-0.4#-0.2#-0.6#-0.4
        ThisRobotCnfg[17+6] = -0.1#0.4#0.2#0.6# 0.4
        # ThisRobotCnfg[17] = -0.4+0.5*PelvisHeight
        # ThisRobotCnfg[17+6] = 0.4-0.5*PelvisHeight
        # print ThisRobotCnfg[17], ThisRobotCnfg[17+6]
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
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.5
        ThisRobotCnfg[17] = -0.6#-0.4#-0.2#-0.6#-0.4
        ThisRobotCnfg[17+6] = 0.6#0.4#0.2#0.6# 0.4
        # ThisRobotCnfg[17] = -1.0+0.5*PelvisHeight
        # ThisRobotCnfg[17+6] = 1.0-0.5*PelvisHeight
        # print ThisRobotCnfg[17], ThisRobotCnfg[17+6]
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

        ##################################################################
        #### Crab Forward Walking Sequence on steep downhill inclines ####
        ##################################################################

        T = 1.5
        self.RobotCnfg3 = []
        self.StepDur3 = []
        
        # Sequence Step 1: Touch ground with pelvis, lift legs
        ThisRobotCnfg = copy(self.SitDwnSeq2)
        ThisRobotCnfg[1] = 0.45
        ThisRobotCnfg[4] = 0.5
        ThisRobotCnfg[4+6] = -0.5
        ThisRobotCnfg[5] = 0.5
        ThisRobotCnfg[5+6] = -0.5
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.15
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.4
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.5
        ThisRobotCnfg[9] = 0.
        ThisRobotCnfg[9+6] = -0.
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.5
        ThisRobotCnfg[17] = 0.8 # -0.6
        ThisRobotCnfg[17+6] = -0.8 # 0.6
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.0
        ThisRobotCnfg[19] = 1.7
        ThisRobotCnfg[19+6] = -1.7
        ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = -1.2
        self.RobotCnfg3.append(ThisRobotCnfg)
        self.StepDur3.append(0.1*T)
        
        # Sequence Step 2: Extend legs
        ThisRobotCnfg = copy(self.RobotCnfg3[0][:])
        ThisRobotCnfg[4] = 0.2
        ThisRobotCnfg[4+6] = -0.2
        ThisRobotCnfg[5] = 0.3
        ThisRobotCnfg[5+6] = -0.3
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -0.7
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 0.6
        self.RobotCnfg3.append(ThisRobotCnfg)
        self.StepDur3.append(0.3*T)

        # Sequence Step 3: Put legs down, bringing torso forward and raising arms
        ThisRobotCnfg = copy(self.RobotCnfg3[1][:])
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -0.4
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.8
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.2
        ThisRobotCnfg[19] = 2.1
        ThisRobotCnfg[19+6] = -2.1
        self.RobotCnfg3.append(ThisRobotCnfg)
        self.StepDur3.append(0.2*T)

        # Sequence Step 4: Touch ground with arms closer to pelvis and lift pelvis
        ThisRobotCnfg = copy(self.RobotCnfg3[2][:])
        ThisRobotCnfg[1] = 0.2
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -0.2
        ThisRobotCnfg[17] = 0.7 # -0.6
        ThisRobotCnfg[17+6] = -0.7 # 0.6
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.8
        ThisRobotCnfg[19] = 1.6
        ThisRobotCnfg[19+6] = -1.6
        self.RobotCnfg3.append(ThisRobotCnfg)
        self.StepDur3.append(0.3*T)

        # Sequence Step 5: Bring pelvis forward, closer to legs
        ThisRobotCnfg = copy(self.RobotCnfg3[3][:])        
        ThisRobotCnfg[1] = 0.2
        ThisRobotCnfg[5] = 0.5
        ThisRobotCnfg[5+6] = -0.5
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -0.6
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.4
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.7
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.9
        ThisRobotCnfg[17] = 1.2 # -0.6
        ThisRobotCnfg[17+6] = -1.2 # 0.6
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 3.0
        ThisRobotCnfg[19] = 1.7
        ThisRobotCnfg[19+6] = -1.7
        ThisRobotCnfg[20] = ThisRobotCnfg[20+6] = -1.5
        self.RobotCnfg3.append(ThisRobotCnfg)
        self.StepDur3.append(0.2*T)

# seqs = PW_seq()
# stream = file('seqs.yaml','w')        
# yaml.dump(seqs,stream)
