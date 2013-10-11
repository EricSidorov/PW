#! /usr/bin/env python
import roslib
roslib.load_manifest('PW')
# roslib.load_manifest('C42_Leg_IK')
#import roslib; roslib.load_manifest('RobotController')
import rospy
import tf
#import PyKDL
import control_primitives
#import contact_reflex
from atlas_msgs.msg import AtlasState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Float32
import numpy as np
class orientation(object):
    def __init__(self):
        self._quaternion = [0,0,0,1]
        self._RPY = [0,0,0]
    def SetQuaternion(self,quaternion):
        self._quaternion = quaternion
        self._RPY = tf.transformations.euler_from_quaternion(quaternion)
    def GetRPY(self):
        return self._RPY

class cb_joints(object):
    def __init__(self,args):
        self.cbJointNames = args
        self._jnt = [0.0 for k in self.cbJointNames]

    def SetAll(self,jnt):
        self._jnt = jnt

    def Get(self,name):
        return self._jnt[self.cbJointNames.index(name)]

    def GetAll(self):
        return self._jnt

class Contact(object):
    def __init__(self,treshold,buf_len,avg_window = 45):
        self._treshold = treshold
        self._contact = False
        self._buffer = [False]*buf_len
        self._avg_buffer = [0.0]*avg_window
        self._avg = 0
    def update(self,force):
        cur = np.linalg.norm(force)
        self._avg_buffer.append(cur)
        first = self._avg_buffer.pop(0)
        self._avg += (cur-first)/len(self._avg_buffer)
        if self._avg>=self._treshold:
            self._buffer.append(True)
        else:
            self._buffer.append(False)
        self._buffer.pop(0)
        if all(self._buffer):
            self._contact = True
        if all([not i for i in self._buffer]):
            self._contact = False
    def GetState(self):
        return self._contact
    def GetAvg(self):
        return self._avg


class ForceTorque(object):
    def __init__(self):
        self.force = [0,0,0]
        self.torque = [0,0,0]
    def update(self,msg):
        self.force = [msg.force.x, msg.force.y, msg.force.z]
        self.torque = [msg.torque.x, msg.torque.y, msg.torque.z]



class robot_state(object):
    def __init__(self,robot_joints):
        self._time = rospy.Time(0)
        self._JointPos = cb_joints(robot_joints)
        self._JointVel = cb_joints(robot_joints)
        self._JointEff = cb_joints(robot_joints)
        self._orientation = orientation()
        self._angular_vel = [0,0,0]
        a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
        b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
        self._vel_x_filter = control_primitives.filter(b,a)
        self._vel_y_filter = control_primitives.filter(b,a)
        self._vel_z_filter = control_primitives.filter(b,a)
        self.force_torque = {'l_hand':ForceTorque(),'r_hand':ForceTorque(),'l_foot':ForceTorque(),'r_foot':ForceTorque()}

    def UpdateState(self,StateMessage):
        self._time = StateMessage.header.stamp
        self._JointPos.SetAll(StateMessage.position)
        self._JointVel.SetAll(StateMessage.velocity)
        self._JointEff.SetAll(StateMessage.effort)
        quaternion = [StateMessage.orientation.x, StateMessage.orientation.y, StateMessage.orientation.z, StateMessage.orientation.w]
        self._orientation.SetQuaternion(quaternion)
        self._angular_vel[0] = (self._vel_x_filter.update(StateMessage.angular_velocity.x))
        self._angular_vel[1] = (self._vel_y_filter.update(StateMessage.angular_velocity.y))
        self._angular_vel[2] = (self._vel_z_filter.update(StateMessage.angular_velocity.z))
        self.force_torque['l_hand'].update(StateMessage.l_hand)
        self.force_torque['r_hand'].update(StateMessage.r_hand)
        self.force_torque['l_foot'].update(StateMessage.l_foot)
        self.force_torque['r_foot'].update(StateMessage.r_foot)
        #self._l_foot_force = self._l_contact_filter.update(StateMessage.l_foot.force.z)
        #self._r_foot_force = self._r_contact_filter.update(StateMessage.r_foot.force.z)
        #self._r_reflex['GC'],self._l_reflex['GC'] = self._GC.update(self._r_foot_force,self._l_foot_force)
        pass
    def GetIMU(self):
        return self._orientation.GetRPY()
    def GetJointPos(self,joint='None'):
        if joint == 'None':
           return self._JointPos.GetAll()
        return self._JointPos.Get(joint)
    def GetForce(self,sensor):
        return self.force_torque[sensor].force
        
    #def GetContactForce(self,leg):
    #    if leg == 'l':
    #        return self._l_foot_force
    #    if leg == 'r':
    #        return self._r_foot_force
    #    return ValueError
    def GetJointVel(self,joint):
        return self._JointVel.Get(joint)
    def GetJointEff(self,joint):
        return self._JointEff.Get(joint)
    def GetAngVel(self):
        return self._angular_vel
    #def GetEvents(self,leg):
    #    if leg == 'l':
    #        return self._l_reflex
    #    if leg == 'r':
    #        return self._r_reflex
    #    return ValueError
############################################################################################################################################
#                                                            TEST                                                                          #
############################################################################################################################################
if __name__ == '__main__':
    jnt_names = ['back_lbz', 'back_mby', 'back_ubx', 'neck_ay', #3
                   'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', #9
                   'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax', #15
                   'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', #21
                   'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'] #27
    class test(object):
        def __init__(self):
            self.RS = robot_state(jnt_names)
            self.sub = rospy.Subscriber('/atlas/atlas_state',AtlasState,self._cb)
            self.force_pub = rospy.Publisher('/RS_test/force',Float32)
        def _cb(self,msg):
            self.RS.UpdateState(msg)
            self.force_pub.publish((force))
    rospy.init_node('TestAtlasState')
    T = test()
    rospy.spin() 



