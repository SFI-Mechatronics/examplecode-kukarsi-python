from kukakinematics import kr120 as kin
import sympy
from sympy.abc import x, t, s
from numpy import deg2rad as d2r
import numpy
import time
from path_variable import path_variable
na = numpy.array

class time_keeper:
    def __init__(self):
        self.reset()

    def reset(self):
        self.initial_timestamp = None

    def get(self, tiden):
        if self.initial_timestamp is None:
            self.initial_timestamp = tiden
        return tiden - self.initial_timestamp

def gracious_stop(t):
    ''' This function streches and in the end stops time.'''
    def f(t):
        if t <= 0.0:
            return 0.0
        return numpy.exp(-1.0/t)

    def g(t):
        return f(t) / (f(t) + f(1.0-t))

    return 2 * g(0.5 * t + 0.5) - 1

q_home = [0.0, d2r(-90), d2r(90), 0.0, d2r(90), 0.0]
home = kin.fk(q_home)

height = 0.1

class ptp:
    def __init__(self, T, pt_start, pt_stop):
        self.T = T
        self.set_start(pt_start)
        self.set_stop(pt_stop)

    def set_start(self, start):
        self.pt_start = numpy.array(start)

    def set_stop(self, stop):
        self.pt_stop = numpy.array(stop)
        
    def path_variable(self, t):
        return path_variable(t, self.T, 1.0)

    def get_position(self, t):
        p = self.path_variable(t)
        dst = self.pt_start + (self.pt_stop - self.pt_start) * p
        return dst.ravel().tolist()

class trajectory:
    def __init__(self, segments):
        self.segments = segments
        self.time_keeper = time_keeper()        
        self.reset()

    def reset(self):
        self.time_keeper.reset()
        self.stop_signal_given = False
        self.stop_signal_timestamp = None
        self.robot_is_completely_stopped = False
        self.T = 0.0
        for s in self.segments:
            self.T += s.T

    def get_current_trajectory_segment(self, t):
        T_sum = 0.0
        for s in self.segments:
            if t < T_sum + s.T:
                return s, t - T_sum
            T_sum += s.T
        self.robot_is_completely_stopped = True
        return self.segments[-1], T_sum

    def get_position(self, t):
        tiden = self.time_keeper.get(t)
        st = self.time_scaling(tiden)
        segment, lt = self.get_current_trajectory_segment(st)
        xyz = segment.get_position(lt)
        return xyz

    def stop(self):
        self.stop_signal_given = True

    def time_scaling(self, timestamp):
        if self.stop_signal_given:
            if self.stop_signal_timestamp is None:
                self.stop_signal_timestamp = timestamp

            time_since_stop_signal = timestamp - self.stop_signal_timestamp

            if time_since_stop_signal > 1.5:
                self.robot_is_completely_stopped = True

            return self.stop_signal_timestamp + gracious_stop(time_since_stop_signal * 0.5)

        return timestamp


current_xyz = [0.0, 0.0, 0.0]
current_trajectory = None
move_up_trajectory = trajectory([ptp(5.0, current_xyz, (0.0, 0.0, height))])
move_down_trajectory = trajectory([ptp(5.0, current_xyz, (0.0, 0.0, 0.0))])

def move_up():
    global current_trajectory, current_xyz, move_up_trajectory
    if current_trajectory is None or current_trajectory.robot_is_completely_stopped:
        move_up_trajectory.segments[0].set_start(current_xyz)
        move_up_trajectory.reset()
        current_trajectory = move_up_trajectory

def move_down():
    global current_trajectory, current_xyz, move_down_trajectory
    if current_trajectory is None or current_trajectory.robot_is_completely_stopped:
        move_down_trajectory.segments[0].set_start(current_xyz)
        move_down_trajectory.reset()
        current_trajectory = move_down_trajectory

def stop():
    if current_trajectory is not None:
        current_trajectory.stop()

def get_desired_pose(t):
    global current_xyz
    if current_trajectory is not None:
        current_xyz = current_trajectory.get_position(t)
    res = numpy.array(home)
    res[0, -1] += current_xyz[0]
    res[1, -1] += current_xyz[1]
    res[2, -1] += current_xyz[2]
    return res


prev_joint_command = list(q_home)

def get_joint_command(t):
    global prev_joint_command
    desired_pose = get_desired_pose(t)
    jc = kin.ik(desired_pose, prev_joint_command)
    prev_joint_command = jc
    return numpy.rad2deg(jc - numpy.array(q_home)).tolist()

if __name__ == '__main__':
    traj = trajectory([ptp(5.0, (0.0, 0.0, 0.0), (0.0, 0.0, height)),
                       ptp(5.0, (0.0, 0.0, height), (0.0, 0.0, 0))])
    for t in numpy.linspace(0, traj.T, 30):
        q = get_joint_command(t)
        print("t: ", t, " q: ", q)
