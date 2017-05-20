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

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {} #Final function for splines interpolation
        time_diff = {} #2D array with differences between target position and sensor times, for each joint
        delta = {} #2D array with joint position differences between target and sensor
        names, times, keys = keyframes #three separate arrays to store returned values from keyframes

        #Calculate the time differences and joint position differences between the target and sensor
        for i in range(len(names)):
            timeTemp = []
            deltaTemp = []
            for j in range(len(times)):
                timeTemp[j] = times[i][j] - perception.time[names[i]] #Target time (see imported) - the time from sensor data
            for k in range(len(keys)):
                deltaTemp[k] = keys[i][k][0] - perception.joint[names[i]]
            time_diff.append(timeTemp)
            delta.append(deltaTemp)

        #Set parameters needed for Splines interpolation
        a0 = perception.joint
        a1 = delta/time_diff #TODO: calculate and avoid error
        a2 = 0
        a3 = 0

        #Calculate splines interpolation
        target_temp = []
        for i in range(len(delta)):
            for j in range(len(time_diff[0])):
                target_temp = a0[i][j] + a1[i][j] + a2[i][j] * time_diff[i][j] * pow(time_diff[i][j], 2) + a3 * time_diff[i][j] * pow(time_diff[i][j], 3)
            target_joints.append(target_temp)
        # t =  - keyframes.times[self.sensor.joints]
        # target_joints = hello()
        # t = keys[0][0][0] for the first splines value
        return target_joints


'''
* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interpolation or Bezier interpolation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interpolation,
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

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    # agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = leftBackToStand()
agent.run()

############################
      #  delta = self.target.joints - self.sensor.joint

        ##aceleration cero at beginning and at the end

        #values of the parameters of the spline. Spline interpolation
       # a0 = self.sensor.joints

        #a1 = delta/t ;

        ##i don't need these two because keyfram use brezier and only need the first and the second parameter
        #a2 = 0
     	#a3 = 0


        #function of the spline interpolation
        #target_joints = a0 +a1*t +a2 *pow(t,2.0)+a3*pow(t,3.0)
