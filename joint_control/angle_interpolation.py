'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

        #when we start the simulation
        self.start_Time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE


        if (self.keyframes == ([], [], [])):

            return target_joints

        # At the beginning
        if (self.start_Time == -1):
            self.start_Time = perception.time

        # and then we compute dif_Time
        dif_Time = perception.time - self.start_Time

        (names, times, keys) = keyframes

        # iterate over all joints in the keyframes
        skippedJoints = 0
        for (i, name) in enumerate(names):

            time_0 = 0  # represents the lower threshold of the keyframe
            time_1 = 0  # represents the upper threshold of the keyframe
            index = 0  # the upper index of the keyframe which has to be used for interpolation
            Time_joints = times[i]
            lenTimes_joints = len(Time_joints)

            # interpolation is finished for this joint, dont do any more steps
            if (dif_Time > Time_joints[-1]):
                skippedJoints += 1
                # reset timer and keyframes
                if (skippedJoints == len(names)):
                    self.startTime = -1
                    self.keyframes = ([], [], [])
                continue

            # iterate over all times of the current joint to find the right time
            for j in xrange(lenTimes_joints):
                time_1 = Time_joints[j]

                # we found the right interval -> break
                if ((dif_Time >= time_0 and dif_Time <= time_1)):
                    index = j
                    break
                time_0 = time_1

            # calculate t-value
            t = (dif_Time - time_0) / (time_1 - time_0)

            # set p-values
            #Bezier interpolation is divided into two equation with different parameters.
            #It's depends on the part of the interpolation function where we are. At the beginning index =0

            # if index == 0  -> no values for p0 and p1

            if index == 0:
                p3 = keys[i][index][0]
                p2 = p3 + keys[i][index][1][2]
                p0 = 0
                p1 = 0
            else:
                p0 = keys[i][index - 1][0]
                p3 = keys[i][index][0]
                p1 = p0 + keys[i][index - 1][2][2]
                p2 = p3 + keys[i][index][1][2]

            # calculate joint angl
            angle = ((1 - t) ** 3) * p0 + 3 * t * ((1 - t) ** 2) * p1 + 3 * (t ** 2) * (1 - t) * p2 + (t ** 3) * p3

            target_joints[name] = angle

            #without this doesn't work.
            #IT is said in : http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
            # "LHipYawPitch and RHipYawPitch share the same motor so they move simultaneously and symmetrically.
            # In case of conflicting orders, LHipYawPitch always takes the priority.

            #Because of that, we have to add also the angles in RHipYawPitch when LHipYawPitch is called
            if (name == "LHipYawPitch"):
                target_joints["RHipYawPitch"] = angle
                # print degrees(angle)

        return target_joints

   
        

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    #agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = leftBackToStand()
    #agent.keyframes = rightBellyToStand()
    #agent.keyframes = rightBackToStand()
    #agent.keyframes = rightBackToStand()
    agent.run()
