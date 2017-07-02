'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''

from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from math import atan2, sqrt



class InverseKinematicsAgent(ForwardKinematicsAgent):
    ## FOLLOWING THE NUMERICAL SOLUTION ROBOT_ARM_2D

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []

        # YOUR CODE HERE
        #FOLLOWING THE EXAMPLE OF 2D arm with a numerical solution

        Epsilon = 0.001

        # at first we need to know our current effector joint position

        current_joint = {}
        for name in self.chains[effector_name]:
            current_joint[name] = self.perception.joint[name]

        # the target effector position
        target_joint = self.chains[effector_name][-1]
        target_transform = self.from_trans(transform)
        length = len(self.chains[effector_name])

        # We will do this loop until the error almost 0, then we will be in the target position
        while True:

            self.forward_kinematics(current_joint)

            # We create a new variable T with 0 and the length of self.chains

            T = [0] * length
            for i, name in enumerate(self.chains[effector_name]):
                # Transformation matrix for each joint (effector!)
                T[i] = self.transforms[name]

            Te = np.array([self.from_trans(T[-1])])

            # we calculate the error now between the target transformation and the
            # .... transformation matrix of our current joint position

            error = target_transform - Te

            T = np.array([self.from_trans(i) for i in T[0:length]])
            J = Te - T

            # Traspose Jacobian
            J_t = J.T
            J_t[-1, :] = 1

            JJT = np.dot(J, J.T)

            d_theta = Epsilon * np.dot(np.dot(J.T, np.linalg.pinv(JJT)), error.T)
            print d_theta

            for i, name in enumerate(self.chains[effector_name]):
                current_joint[name] = current_joint[name] + np.asarray(d_theta.T)[0][1]

            if np.linalg.norm(d_theta) < Epsilon:
                break
            return current_joint

        joint_angles = current_joint

        return joint_angles

    def from_trans(self, T):
        # return x,y,z
        x, y, z = T[0, 3], T[1, 3], T[2, 3]

        theta_x = 0
        theta_y = 0
        theta_z = 0

        # How to decompose a 3x3 rotation matrix: http://nghiaho.com/?page_id=846

        variable = sqrt(T[2,1]**2 + T[2, 2]**2)



        theta_x = atan2(T[2,1], T[2, 2])
        theta_y = atan2(-T[2,0], variable)
        theta_z = atan2(T[1,0], T[0,0])



        return np.array([x, y, z, theta_x, theta_y, theta_z])



    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)

        #times =
        #names = self.chains[effector_name]
        #keys = []
        # I don't have idea how to change the time in the keyframes
        #current_joint = self.perception.joint

        #for i, name in enumerate(names):
         #   keys.insert(i, [current_joint[name]], [joint_angles[name]])


        #self.keyframes = (names, times , keys)  # the result joint angles have to fill in
        self.target_joints.update(joint_angles)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
