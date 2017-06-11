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
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


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
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll','LWristYaw', 'LHand'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']

                       # YOUR CODE HERE
                       }
        #To know the lenght of each joint in the robot

        #????? WE need the lenght of the RHand and LHand??
        self.joint_length = {'HeadYaw': (0.0, 0.0, 126.50), 'HeadPitch': (0.0, 0.0, 0.0), #
                             'LShoulderPitch': (0.0, 98.0, 100.0), 'LShoulderRoll': (0.0, 98.0, 100.0), 'LElbowYaw': (105.0, 98.0 + 15.0, 100.0), 'LElbowRoll': (105.0, 98.0 + 15.0, 100.0), 'LWristYaw': (105.0 + 55.95, 98.0 + 15.0, 100.0), 'LHand': (105.0 + 55.95 + 57.75, 98.0 + 15.0, 100.0 + 12.31), #
                             'RShoulderPitch': (0.0, -98.0, 100.0), 'RShoulderRoll': (0.0, -98.0, 100.0), 'RElbowYaw': (105.0, -98.0 - 15.0, 100.0), 'RElbowRoll': (105.0, -98.0 - 15.0, 100.0), 'RWristYaw': (105.0 + 55.95, -98.0 - 15.0, 100.0), 'RHand': (105.0 + 55.95 + 57.75, -98.0 - 15.0, 100.0 + 12.31), #
                             'LHipYawPitch': (0.0, 50.0, -85.0), 'LHipRoll': (0.0, 50.0, -85.0), 'LHipPitch': (0.0, 50.0, -85.0), 'LKneePitch': (0.0, 50.0, -85.0 - 100.0), 'LAnklePitch': (0.0, 50.0, -85.0 - 100.0 - 102.90), 'LAnkleRoll': (0.0, 50.0, -85.0 - 100.0 - 102.9), #
                             'RHipYawPitch': (0.0, -50.0, -85.0), 'RHipRoll': (0.0, -50.0, -85.0),'RHipPitch': (0.0, -50.0, -85.0), 'RKneePitch': (0.0, -50.0, -85.0 - 100.0), 'RAnklePitch': (0.0, -50.0, -85.0 - 100.0 - 102.90), 'RAnkleRoll': (0.0, -50.0, -85.0 - 100.0 - 102.9), #
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
        T = matrix()
        # YOUR CODE HERE

        #We declara the seno and coseno
        s_angle = math.sin(joint_angle)
        c_angle = math.cos(joint_angle)



        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = local_trans(joint, angle)
                # YOUR CODE HERE

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
