#!/usr/bin/env python
#
# Software License Agreement (BSD License)
# Copyright (c) 2017-2018, Aubo Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the Southwest Research Institute, nor the names
#    of its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from motion_controller_simulator import MotionControllerSimulator
import rospy
import copy
import threading
# Publish
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback
from std_msgs.msg import Int32MultiArray
# Subscribe
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# Services
from industrial_msgs.srv import GetRobotInfo, GetRobotInfoResponse
# Reference
from industrial_msgs.msg import TriState, RobotMode, ServiceReturnCode, DeviceInfo
from trajectory_speed import scale_trajectory_speed
import Queue
import math

"""
AuboRobotSimulator

This class simulates an Aubo robot controller.  The simulator
adheres to the ROS-Industrial robot driver specification:

http://www.ros.org/wiki/Industrial/Industrial_Robot_Driver_Spec

TODO: Currently the simulator only supports the bare minimum motion interface.

TODO: Interfaces to add:
Joint streaming
All services
"""


class AuboRobotSimulatorNode:
    """
    Constructor of aubo robot simulator
    """

    def __init__(self):
        rospy.init_node('aubo_robot_simulator')
        self.seq_number = 1.0
        self.traj_list = []
        self.splitNum = 5
        self.last_trajectory = JointTrajectory()
        self.last_tfs = rospy.rostime.Duration(0)

        self.joint_vel_lim = [2.0] * 6
        self.joint_acc_lim = [1.7, 1.7, 10.0, 10.0, 10.0, 10.0]

        # Class lock for only 1 thread to use it at same time
        self.lock = threading.Lock()

        # Publish rate (hz)
        self.pub_rate = rospy.get_param('pub_rate', 50.0)
        rospy.loginfo("Setting publish rate (hz) based on parameter: %f", self.pub_rate)

        # Joint names
        def_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.joint_names = rospy.get_param('controller_joint_names', def_joint_names)
        if len(self.joint_names) == 0:
            rospy.logwarn("Joint list is empty, did you set controller_joint_name?")
        rospy.loginfo("Simulating manipulator with %d joints: %s", len(self.joint_names), ", ".join(self.joint_names))

        # Setup initial joint positions
        num_joints = len(self.joint_names)

        # retrieve update rate
        motion_update_rate = rospy.get_param('motion_update_rate', 200.)  # set param to 0 to ignore interpolated motion
        self.motion_ctrl = MotionControllerSimulator(num_joints, update_rate=motion_update_rate)

        self.velocity_scale_factor = rospy.get_param('/aubo_controller/velocity_scale_factor', 1.0)
        rospy.loginfo("The velocity scale factor of the trajectory is: %f", self.velocity_scale_factor)

        # Published to joint states
        rospy.logdebug("Creating joint state publisher")
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=100)

        # Published to joint feedback
        rospy.logdebug("Creating joint feedback publisher")
        self.joint_feedback_pub = rospy.Publisher('feedback_states', FollowJointTrajectoryFeedback, queue_size=100)

        # Subscribe to a joint trajectory
        rospy.loginfo("Creating joint trajectory subscriber")
        self.joint_path_sub = rospy.Subscriber('joint_path_command', JointTrajectory, self.trajectory_received)

        # Subscribe to a joint trajectory
        rospy.loginfo("Enable Switch")
        self.plan_type_sub = rospy.Subscriber('/aubo_driver/rib_status', Int32MultiArray, self.rib_status_callback)

        # JointStates timed task (started automatically)
        # period = rospy.Duration(1.0/self.pub_rate)
        # rospy.logdebug('Setting up publish worker with period (sec): %s', str(period.to_sec()))
        # rospy.Timer(period, self.publish_worker)

        self.EnableFlag = 1

        # GetRobotInfo service server and pre-cooked svc response
        self.get_robot_info_response = self._init_robot_info_response()
        self.svc_get_robot_info = rospy.Service('get_robot_info', GetRobotInfo, self.cb_svc_get_robot_info)

        rospy.loginfo("Clean up init")
        rospy.on_shutdown(self.motion_ctrl.shutdown)

        self.exec_thread = threading.Thread(target=self.exec_loop())
        self.exec_thread.daemon = True
        self.exec_thread.start()

    """
    Service callback for GetRobotInfo() service. Returns fake information.
    """

    def cb_svc_get_robot_info(self, req):
        # return cached response instance
        return self.get_robot_info_response

    """
    The publish worker is executed at a fixed rate.  This publishes the various
    state and status information to the action controller.
    """

    def publish_worker(self, event):
        pass
        # self.joint_state_publisher()
        # self.robot_status_publisher()         # robot_status message is published by aubo_driver

    """
    The joint state publisher publishes the current joint state and the current
    feedback state (as these are closely related)
    """

    def joint_state_publisher(self):
        if self.EnableFlag == 1 and self.motion_ctrl.positionUpdatedFlag == '1':
            try:
                joint_state_msg = JointState()
                joint_fb_msg = FollowJointTrajectoryFeedback()
                time = rospy.Time.now()

                with self.lock:
                    # Joint states
                    joint_state_msg.header.stamp = time
                    joint_state_msg.name = self.joint_names
                    joint_state_msg.position = self.motion_ctrl.get_joint_positions()
                    # self.joint_state_pub.publish(joint_state_msg)

                    # Joint feedback
                    joint_fb_msg.header.stamp = time
                    joint_fb_msg.joint_names = self.joint_names
                    joint_fb_msg.actual.positions = self.motion_ctrl.get_joint_positions()

                    # self.joint_feedback_pub.publish(joint_fb_msg)

            except Exception as e:
                rospy.loginfo('Unexpected exception in joint state publisher: %s', e)

    """
    The robot status publisher publishes the current simulated robot status.

    The following values are hard coded:
     - robot always in AUTO mode
     - drives always powered
     - motion always possible
     - robot never E-stopped
     - no error code
     - robot never in error

    The value of 'in_motion' is derived from the state of the MotionControllerSimulator.
    """

    def rib_status_callback(self, data):
        try:
            if data.data[1] == 1:
                #                self.EnableFlag = 1
                rospy.logdebug('True True %d', self.EnableFlag)
            else:
                #                self.EnableFlag = 0
                rospy.logdebug('False False %d', self.EnableFlag)
            self.motion_ctrl.ribBufferSize = data.data[0]
            self.motion_ctrl.ControllerConnectedFlag = data.data[2]
            # rospy.loginfo('mode %d', data.data[1])

        except Exception as e:
            rospy.logerr('Unexpected exception1: %s', e)

    """
    Trajectory subscription callback (gets called whenever a "joint_path_command" message is received).
    @param msg_in: joint trajectory message
    @type  msg_in: JointTrajectory
    """

    # prints
    def print_point(self, p):
        acc = p.accelerations
        vel = p.velocities
        pos = p.positions
        rospy.logwarn("time_from_start = %f", p.time_from_start.to_sec())
        rospy.logwarn("pos=[%f, %f, %f, %f, %f, %f]", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
        rospy.logwarn("vel=[%f, %f, %f, %f, %f, %f]", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5])
        rospy.logwarn("acc=[%f, %f, %f, %f, %f, %f]\n", acc[0], acc[1], acc[2], acc[3], acc[4], acc[5])

    def print_trajectory(self, trj):
        c = 0
        # rospy.loginfo("header.stamp = %f", trj.header.stamp.to_sec())
        # rospy.loginfo("header.seq = %d", trj.header.seq)
        for p in trj.points:
            rospy.loginfo("point #%d", c)
            self.print_point(p)
            c += 1

    def print_trajectory_times(self, trj):
        c = 0
        rospy.loginfo("header.stamp = %f", trj.header.stamp.to_sec())
        rospy.loginfo("header.seq = %f", trj.header.seq)
        for p in trj.points:
            rospy.logwarn("[%d] time_from_start = %f", c, p.time_from_start.to_sec())
            c += 1

    def print_trajectories(self, trj1, trj2):
        c = 0
        rospy.loginfo("Len t1=%d, t2=%d", len(trj1.points), len(trj2.points))
        for i in range(min(len(trj1.points), len(trj2.points))):
            rospy.loginfo("point #%d BEFORE", c)
            self.print_point(trj1.points[i])
            rospy.loginfo("point #%d AFTER", c)
            self.print_point(trj2.points[i])
            rospy.loginfo("")
            c += 1

    # Smoothing velocities
    def smooth_vel(self, trj):
        for j in range(6):
            res = []
            for p in trj.points:
                res.append(p.velocities[j])
            a = self.linear_smooth_array(res)
            for i in range(len(trj.points)):
                v = list(trj.points[i].velocities)
                v[j] = a[i]
                trj.points[i].velocities = tuple(v)
        return trj

    # Smoothing accelerations
    def smooth_acc(self, trj):
        for j in range(6):
            res = []
            for p in trj.points:
                res.append(p.accelerations[j])
            a = self.linear_smooth_array(res)
            for i in range(len(trj.points)):
                v = list(trj.points[i].accelerations)
                v[j] = a[i]
                trj.points[i].accelerations = tuple(v)
        return trj

    # applying linear smooth (?) to arrays
    def linear_smooth_array(self, arr):
        res = copy.deepcopy(arr)
        step = (arr[-1] - arr[0]) / (len(arr) - 1)
        for i in range(0, len(arr)):
            res[i] = arr[0] + i * step
        return res

    # recalculates time + vel + acc
    def recalculate_time(self, trj):
        if len(trj.points) < 2:
            rospy.logerr("len(trj.points) < 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return trj

        for i in range(1, len(trj.points)):
            delta_t = -1.0

            # get max time
            for j in range(6):
                p0 = copy.deepcopy(trj.points[i-1].positions[j])
                p1 = copy.deepcopy(trj.points[i].positions[j])
                v0 = copy.deepcopy(trj.points[i-1].velocities[j])
                v1 = copy.deepcopy(trj.points[i].velocities[j])
                delta_t = max(abs(2 * (p1 - p0) / (v0 + v1)), delta_t)

            if delta_t != 0:
                vel = [0.0] * 6
                for j in range(6):
                    p0 = copy.deepcopy(trj.points[i-1].positions[j])
                    p1 = copy.deepcopy(trj.points[i].positions[j])
                    vel[j] = (p1-p0)/delta_t
                trj.points[i].velocities = tuple(vel)

                acc = [0.0] * 6
                for j in range(6):
                    v0 = copy.deepcopy(trj.points[i - 1].velocities[j])
                    v1 = copy.deepcopy(trj.points[i].velocities[j])
                    acc[j] = (v1-v0)/delta_t
                trj.points[i].accelerations = tuple(acc)
            else:
                rospy.logerr("Delta t = 0 !!!")

            # rospy.loginfo("Old Time = %f", trj.points[i].time_from_start.to_sec())
            trj.points[i].time_from_start = rospy.rostime.Duration(trj.points[i-1].time_from_start.to_sec() + delta_t)
            # rospy.loginfo("New Time = %f\n", trj.points[i].time_from_start.to_sec())
        return trj

    # dont use, unexpected behavior
    def sync_time(self, trj):
        if len(self.last_trajectory.points) > 0 and len(self.traj_list) > 1:
            rospy.loginfo("sync_time running")
            t = self.last_trajectory.points[-1].time_from_start - trj.points[0].time_from_start
            for i in range(len(trj.points)):
                trj.points[i].time_from_start = trj.points[i].time_from_start + t
        return trj

    def merge_trajectories(self, trj1, trj2):
        res = copy.deepcopy(trj1)
        for i in range(1, len(trj2.points)):
            res.points.append(trj2.points[i])
        return res

    # Smoothing 2 trajectories to remove speed fall
    def smooth_trajectory_transition(self, trj1, trj2):
        curr_time = rospy.Time.now()
        fin_trj = self.merge_trajectories(trj1, trj2)
        # fin_trj = self.smooth_acc(fin_trj)
        # fin_trj = self.smooth_vel(fin_trj)
        # fin_trj = self.recalculate_time(fin_trj)
        rospy.logerr("Smoothing took %f seconds!", (rospy.Time.now() - curr_time).to_sec())
        return fin_trj

    def get_max_joint_dist(self, point0, point1):
        res = 0.0
        max_index = 0
        for j in range(6):
            tmp = abs(point1.positions[j] - point0.positions[j])
            if tmp > res:
                res = copy.deepcopy(tmp)
                max_index = j
        return res, max_index

    def check_trj_constraints(self, trj):
        for p in trj.points:
            for j in range(6):
                if abs(p.velocities[j]) >= self.joint_vel_lim[j]: # or abs(p.accelerations[j]) >= 1.7:
                    rospy.logerr("[%d] vel[j] %f >= %f ", j, abs(p.velocities[j]), self.joint_vel_lim[j])
                    rospy.logerr("[%d] acc[j] %f >= %f ", j, abs(p.accelerations[j]), 1.7)
                    return False
        return True

    def scale_trj(self, trj, time):
        rospy.logwarn("Tring to scale with time = %f", time)
        trajectory = copy.deepcopy(trj)
        joint_dist = [0.0]
        joint_dist_max_index = [0]

        for i in range(1, len(trajectory.points)):
            dist, idx = self.get_max_joint_dist(trajectory.points[i-1], trajectory.points[i])
            joint_dist.append(dist)
            joint_dist_max_index.append(idx)

        total = sum(joint_dist)
        # rospy.logwarn("Total = %f", total)

        for i in range(1, len(trajectory.points)):
            delta_t = joint_dist[i] / total * time

            # rospy.logwarn("before time_from_start = %f", trajectory.points[i].time_from_start.to_sec())
            trajectory.points[i].time_from_start = rospy.rostime.Duration(trajectory.points[i-1].time_from_start.to_sec() + delta_t)
            # rospy.logwarn("after time_from_start = %f\n", trajectory.points[i].time_from_start.to_sec())

            vel = [0.0] * 6
            for j in range(6):
                p0 = copy.deepcopy(trajectory.points[i-1].positions[j])
                p1 = copy.deepcopy(trajectory.points[i].positions[j])
                vel[j] = (p1-p0) / delta_t
            trajectory.points[i].velocities = tuple(vel)

            acc = [0.0] * 6
            for j in range(6):
                v0 = copy.deepcopy(trajectory.points[i-1].velocities[j])
                v1 = copy.deepcopy(trajectory.points[i].velocities[j])
                acc[j] = (v1-v0) / delta_t
            trajectory.points[i].accelerations = tuple(acc)

        return trajectory, self.check_trj_constraints(trajectory)

    def speed_up_traj(self, trj):
        # 10 000 seconds
        first = 0.0
        last = 10000.0
        mid = -1.0
        tmp_trj = JointTrajectory()
        while last - first > 0.001:
            mid = (first+last)/2
            tmp_trj, cons = self.scale_trj(trj, mid)
            if cons:
                last = mid
            else:
                first = mid

        rospy.logwarn("Time: old=%f, new=%f", trj.points[-1].time_from_start.to_sec(), mid)
        return tmp_trj

    def new_traj_algo(self, trj):
        joint_dist = [0.0]
        joint_dist_max_index = [0]

        for i in range(len(trj.points)-1):
            dist, idx = self.get_max_joint_dist(trj.points[i], trj.points[i+1])
            joint_dist.append(dist)
            joint_dist_max_index.append(idx)

        for i in range(1, len(joint_dist)):
            rospy.logwarn("[%d] dist=%f (idx=%d)", i, joint_dist[i], joint_dist_max_index[i])

        for i in range(1, len(trj.points)):
            if trj.points[i].velocities[joint_dist_max_index[i]] != 0:
                scale_v = self.joint_vel_lim[joint_dist_max_index[i]]/trj.points[i].velocities[joint_dist_max_index[i]]/5
                scale_a = self.joint_vel_lim[joint_dist_max_index[i]]/trj.points[i].accelerations[joint_dist_max_index[i]]/5

                vel = [0.0] * 6
                for j in range(6):
                    vel[j] = trj.points[i].velocities[j] * scale_v
                trj.points[i].velocities = tuple(vel)

                acc = [0.0] * 6
                for j in range(6):
                    acc[j] = trj.points[i].accelerations[j] * scale_a
                trj.points[i].accelerations = tuple(acc)

                # trj.points[i].time_from_start = rospy.rostime.Duration(trj.points[i].time_from_start.to_sec()*scale_v)


        # return res

    # point = JointTrajectoryPoint()
    # point.velocities
    # point.accelerations
    # point.time_from_start
    # point.positions
    #
    # trajectory = JointTrajectory()
    # trajectory.points
    # trajectory.joint_names
    # trajectory.header

    # splits trajectory into chucks of N='self.splitNum' points, if possible
    def trajectory_received(self, msg_trj):
        rospy.loginfo("Trajectory Received [size = %d]", len(msg_trj.points))
        msg_trj = self.sync_time(msg_trj)
        self.print_trajectory(msg_trj)
        # self.new_traj_algo(trj)
        trj = copy.deepcopy(self.speed_up_traj(msg_trj))
        self.print_trajectories(msg_trj, trj)

        # rospy.logerr("Continue?")
        # input()

        if len(trj.points) != 0:
            slices = int(math.ceil(len(trj.points) / self.splitNum)) + 1
            for i in range(slices):
                t = copy.deepcopy(trj)
                t.points = t.points[i*self.splitNum: i*self.splitNum + self.splitNum]

                if (i == 0) and (len(self.traj_list) > 0):
                    last_traj = self.traj_list.pop()
                    smoothed_trajectory = self.smooth_trajectory_transition(last_traj, t)
                    self.traj_list.append(copy.deepcopy(smoothed_trajectory))
                    self.last_trajectory = copy.deepcopy(smoothed_trajectory)
                elif len(t.points) > 0:
                    self.last_trajectory = copy.deepcopy(t)
                    self.traj_list.append(t)

    def exec_loop(self):
        while not self.motion_ctrl.sig_shutdown:
            # if self.motion_ctrl.is_in_motion() or self.EnableFlag == 0 or len(self.traj_list) == 0 or self.motion_ctrl.points_left() >= self.splitNum:
            if self.EnableFlag == 0 or len(self.traj_list) == 0 or self.motion_ctrl.points_left() >= self.splitNum / 2:
                continue

            rospy.loginfo('[aubo_robot_simulator_node/exec_loop] Executing...')
            with self.lock:
                msg_in = self.traj_list.pop(0)

                if len(msg_in.points) > 0:
                    # rospy.loginfo('[aubo_robot_simulator_node/exec_loop] msg_in.len=%d', len(msg_in.points))
                    self.velocity_scale_factor = rospy.get_param('/aubo_controller/velocity_scale_factor', 1.0)
                    # rospy.loginfo('[aubo_robot_simulator_node/exec_loop] The velocity scale factor is: %s', str(self.velocity_scale_factor))
                    curr_traj = scale_trajectory_speed(msg_in, self.velocity_scale_factor)
                    rospy.loginfo('Sending trj point by point:')

                    # first_point_tfs = curr_traj.points[0].time_from_start
                    # if curr_traj.points[0].time_from_start < self.last_tfs:
                    #     for p in curr_traj.points:
                    #         p.time_from_start -= first_point_tfs
                    # self.print_trajectory(curr_traj)

                    for point in curr_traj.points:
                        point = self._to_controller_order(msg_in.joint_names, point)
                        self.motion_ctrl.add_motion_waypoint(point)

                    # self.last_tfs = curr_traj.points[-1].time_from_start
                else:
                    rospy.logerr('[aubo_robot_simulator_node/exec_loop] len(msg_in.points) == 0')

        rospy.logerr('[aubo_robot_simulator_node/exec_loop] Exiting (sig_shutdown)')

    """
    Remaps point to controller joint order

    @param point:  joint trajectory point
    @type  point:  JointTrajectoryPoint
    @return point: reorder point
    """

    def _to_controller_order(self, keys, point):
        pt_rtn = copy.deepcopy(point)
        pt_rtn.positions = self._remap_order(self.joint_names, keys, point.positions)
        pt_rtn.velocities = self._remap_order(self.joint_names, keys, point.velocities)
        pt_rtn.accelerations = self._remap_order(self.joint_names, keys, point.accelerations)
        return pt_rtn

    def _remap_order(self, ordered_keys, value_keys, values):
        # rospy.loginfo('remap order, ordered_keys: %s, value_keys: %s, values: %s', str(ordered_keys), str(value_keys), str(values))
        ordered_values = []

        ordered_values = [0] * len(ordered_keys)
        mapping = dict(zip(value_keys, values))
        # rospy.loginfo('maping: %s', str(mapping))

        for i in range(len(ordered_keys)):
            ordered_values[i] = mapping[ordered_keys[i]]
            pass

        return ordered_values

    """
    Constructs a GetRobotInfoResponse instance with either default data.
    """

    def _init_robot_info_response(self):
        if not rospy.has_param('~robot_info'):
            # if user did not provide data, we generate some
            import rospkg
            rp = rospkg.RosPack()
            irs_version = rp.get_manifest('industrial_robot_simulator').version
            robot_info = dict(
                controller=dict(
                    model='Aubo Robot Simulator Controller',
                    serial_number='0123456789',
                    sw_version=irs_version),
                robots=[
                    dict(
                        model='Aubo Robot Simulator Manipulator',
                        serial_number='9876543210',
                        sw_version=irs_version)
                ])
        else:
            # otherwise use only the data user has provided (and nothing more)
            robot_info = rospy.get_param('~robot_info')

        resp = GetRobotInfoResponse()
        resp.controller = DeviceInfo(**robot_info['controller'])

        # add info on controlled robot / motion group
        if len(robot_info['robots']) > 0:
            robot = robot_info['robots'][0]
            resp.robots.append(DeviceInfo(**robot))

        if len(robot_info['robots']) > 1:
            # simulator simulates a single robot / motion group
            rospy.logwarn("Multiple robots / motion groups defined in "
                          "'robot_info' parameter, ignoring all but first element")

        # always successfull
        resp.code.val = ServiceReturnCode.SUCCESS
        return resp
