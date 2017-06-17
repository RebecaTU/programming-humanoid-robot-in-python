'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys

import numpy as np

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, math

from angle_interpolation import AngleInterpolationAgent
from keyframes import *




class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        #name of each joint in each chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
                       # YOUR CODE HERE
                       }
        #To know the lenght of each joint in the robot
        #doc.aldebaran.com/2-1/family/robots/links_robot.html


        self.joint_length = {'HeadYaw': (0.0, 0.0, 126.50), 'HeadPitch': (0.0, 0.0, 0.0), #
                             'LShoulderPitch': (0.0, 98.0, 100.0), 'LShoulderRoll': (0.0, 0.0 , 0.0), 'LElbowYaw': (105.0,  15.0, 0.0), 'LElbowRoll': (0.0, 0.0, 0.0), 'LWristYaw': (55.95, 0.0, 0.0), #
                             'RShoulderPitch': (0.0, -98.0, 100.0), 'RShoulderRoll': (0.0, 0.0, 0.0), 'RElbowYaw': (105.0, - 15.0, 100.0), 'RElbowRoll': (0.0, 0.0, 0.0), 'RWristYaw': (55.95, 0.0, 0.0), #
                             'LHipYawPitch': (0.0, 50.0, -85.0), 'LHipRoll': (0.0, 0.0, 0.0), 'LHipPitch': (0.0, 0.0, 0.0), 'LKneePitch': (0.0, 0.0, - 100.0), 'LAnklePitch': (0.0, 0.0,  - 102.90), 'LAnkleRoll': (0.0, 0.0, 0.0), #
                             'RHipYawPitch': (0.0, -50.0, -85.0), 'RHipRoll': (0.0, 0.0, 0.0),'RHipPitch': (0.0, 0.0, 0.0), 'RKneePitch': (0.0, 0.0, - 100.0), 'RAnklePitch': (0.0, 0.0,  - 102.90), 'RAnkleRoll': (0.0, 0.0, 0.0), #
                        }


    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''

        T = identity(4)
        T1 = identity(4)
        # YOUR CODE HERE

        #We declare the seno and coseno
        s_angle = math.sin(joint_angle)
        c_angle = math.cos(joint_angle)
        pi = np.pi



        #The point here is the next one: The transformation matrix depends on the rotation of the joint:
        #See  http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html  for more information


        #Rotation in X axis
        if 'Roll' in joint_name:
            T1 = matrix([[1, 0, 0, 0],
                       [0, c_angle, -s_angle, 0],
                       [0, s_angle, c_angle, 0],
                       [0, 0, 0, 1]
                       ])
            T = np.dot(T, T1)

        #Rotation in Y axis
        if 'Pitch' in joint_name:
            T1 = matrix([[c_angle, 0, s_angle, 0],
                        [0, 1, 0, 0],
                        [-s_angle, 0, c_angle, 0],
                        [0, 0, 0, 1]
                        ])
            T = np.dot(T, T1)

        #Rotation in Z axis
        if 'Yaw' in joint_name:
            T1 = matrix([[c_angle, s_angle, 0, 0],
                        [-s_angle, c_angle, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                        ])
            T = np.dot(T, T1)
        if joint_name == 'LHipYawPitch' or joint_name == 'RHipYawPitch':
            T1 = matrix([[1, 0, 0, 0],
                         [0, math.cos(-pi/4), - math.sin(-pi/4), 0],
                         [0, math.sin(-pi/4), math.cos(-pi/4), 0],
                         [0, 0, 0, 1]
                         ])
            T = np.dot(T, T1)



        else:

            print "Error in joint name"

        # now we have the transformation matrix with the rotation, but we need also the translation that depends on the lenght of the joint
        # For the x translation, y translation and z.
        # x = T[0][3]
        # y = T[1][3]
        # z = T[2][3]
        T[0, 3] = self.joint_length[joint_name][0]
        T[1, 3] = self.joint_length[joint_name][1]
        T[2, 3] = self.joint_length[joint_name][2]

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():

            T = identity(4)
            for joint in chain_joints:

                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T,Tl)



                self.transforms[joint] = T


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
