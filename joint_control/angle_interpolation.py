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
from keyframes import hello  # leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.start = 0.0
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        # time differences between the target time and the sensor times for each joint.

        currenttime = self.perception.time

        time_diff = currenttime - self.start

        # to access them

        (names, times, keys) = keyframes

        # Calculate the time differences and joint position differences between the target and sensor
        for i in range(len(names)):
            # name of the joint
            joint_name = names[i]
            # the name that we use are only the one that are known
            if joint_name in self.joint_names:
                # different time
                # we are going to have two different equation that depend on the time of the Bezier interpolation
                for j in range(len(times[i]) - 1):
                    # for the first point in the Bezier interpolation
                    if time_diff < times[i][0]:
                        target_joints[joint_name] = self.first_bezier(times, keys, i, joint_name, time_diff)
                    # if we are between point in the middle of the interpolation and until the last point
                    elif (times[i][j] < time_diff < times[i][j + 1]) and ((j + 1) < len(times[i])):
                        target_joints[joint_name] = self.rest_bezier(times, keys, i, j, joint_name, time_diff)

        return target_joints

        # Calculate Bezier interpolation (first one)

    def first_bezier(self, times, keys, index_j, joint_name, time_diff):

        # We compute the start time and the end one

        t_end = times[index_j][0]

        # the parameters of the spline
        p_0 = self.perception.joint[joint_name]
        p_1 = keys[index_j][0][1][2] + p_0
        p_3 = keys[index_j][0][0]
        p_2 = keys[index_j][0][2][2] + p_3

        # the time of the spline
        t = time_diff / t_end

        return self.calculate.bezier(p_0, p_1, p_2, p_3, t)

    def rest_bezier(self, times, keys, index_j, t_index, joint_name, time_diff):
        # We compute the start time and the end one
        t_end = times[index_j][0]

        # the parameters of the spline
        p_0 = self.perception.joint[joint_name]
        p_1 = keys[index_j][0][1][2] + p_0
        p_3 = keys[index_j][0][0]
        p_2 = keys[index_j][0][2][2] + p_3

        # the time of the spline
        t = (time_diff - self.start) / (t_end - self.start)

        return self.calculate.bezier(p_0, p_1, p_2, p_3, t)

    # Calculation bezier interpolation.
    def calculate_bezier(p_0, p_1, p_2, p_3, t):
        # to make it easier
        c_0 = pow(1 - t, 3)
        c_1 = 3 * pow(1 - t, 2)
        c_2 = 3 * (1 - t)
        return c_0 * p_0 + c_1 * p_1 * t + c_2 * p_2 * pow(t, 2) + p_3 * pow(t, 3)


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    # agent.keyframes = leftBackToStand()
    agent.run()
