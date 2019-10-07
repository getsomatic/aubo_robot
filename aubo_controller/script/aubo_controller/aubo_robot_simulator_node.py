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
        self.traj_list = []
        self.splitNum = 5

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
            rospy.logerr('Unexpected exception: %s', e)

    """
    Trajectory subscription callback (gets called whenever a "joint_path_command" message is received).
    @param msg_in: joint trajectory message
    @type  msg_in: JointTrajectory
    """

    def print_point(self, p):
        acc = p.accelerations
        vel = p.velocities
        pos = p.positions
        rospy.logerr("time_from_start = %f", p.time_from_start.to_sec())
        rospy.logerr("pos=[%f, %f, %f, %f, %f, %f]", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
        rospy.logerr("vel=[%f, %f, %f, %f, %f, %f]", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5])
        rospy.logerr("acc=[%f, %f, %f, %f, %f, %f]", acc[0], acc[1], acc[2], acc[3], acc[4], acc[5])

    def print_trajectory(self, trj):
        c = 0
        for p in trj.points:
            rospy.loginfo("point #%d", c)
            self.print_point(p)
            c += 1

    def print_trajectories(self, trj1, trj2):
        c = 0
        for i in range(len(trj2.points)):
            rospy.loginfo("point #%d BEFORE", c)
            self.print_point(trj1.points[i])
            rospy.loginfo("point #%d AFTER", c)
            self.print_point(trj2.points[i])
            rospy.loginfo("")
            c += 1

    # getting values for each joint
    def get_vel_for_joint(self, trj, num):
        res = []
        for p in trj.points:
            res.append(p.velocities[num])
        return res

    # assigning values back to joints
    def set_vel_for_joint(self, trj, arr, num):
        for i in range(len(trj.points)):
            v = list(trj.points[i].velocities)
            v[num] = arr[i]
            trj.points[i].velocities = tuple(v)
        return trj

    # getting values for each joint
    def get_acc_for_joint(self, trj, num):
        res = []
        for p in trj.points:
            res.append(p.accelerations[num])
        return res

    # assigning values back to joints
    def set_acc_for_joint(self, trj, arr, num):
        for i in range(len(trj.points)):
            v = list(trj.points[i].accelerations)
            v[num] = arr[i]
            trj.points[i].accelerations = tuple(v)
        return trj

    # Smoothing velocities
    def smooth_vel(self, trj):
        for i in range(6):
            v = self.line_smooth_array(self.get_vel_for_joint(trj, i))
            trj = self.set_vel_for_joint(trj, v, i)
        return trj

    # Smoothing accelerations
    def smooth_acc(self, trj):
        for i in range(6):
            a = self.line_smooth_array(self.get_acc_for_joint(trj, i))
            trj = self.set_acc_for_joint(trj, a, i)
        return trj

    # get median of array
    def get_median(self, arr):
        n = len(arr)
        s = sorted(arr)
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None

    # applying median smooth to arrays
    def median_smooth_arrays(self, arr1, arr2):
        l1 = len(arr1)
        arr = arr1 + arr2
        med = self.get_median(arr)
        # rospy.logerr(med)
        for i in range(len(arr)):
            arr[i] = (arr[i] + med) / 2.0
        return arr[0:l1], arr[l1:]

    # applying linear smooth (?) to arrays
    def line_smooth_array(self, arr):
        step = (arr[-1] - arr[0]) / (len(arr) - 1)
        for i in range(0, len(arr)):
            arr[i] = arr[0] + i * step
        return arr

    # Smoothing 2 trajectories to remove speed fall
    def smooth_trajectory_transition(self, trj1, trj2):
        curr_time = rospy.Time.now()
        fin_trj = copy.deepcopy(trj1)
        for p in trj2.points:
            fin_trj.points.append(p)

        fin_trj = self.smooth_vel(fin_trj)
        fin_trj = self.smooth_acc(fin_trj)
        fin_trj = self.recalculate_time(fin_trj)

        rospy.logerr("Smoothing took %f seconds!", (rospy.Time.now() - curr_time).to_sec())
        return fin_trj

    def recalculate_time(self, trj):
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

        if len(trj.points) < 2:
            rospy.logerr("len(trj.points) < 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return trj

        for i in range(1, len(trj.points)):
            dt = 0.0

            # get max time
            for j in range(6):
                p0 = copy.deepcopy(trj.points[i - 1].positions[j])
                p1 = copy.deepcopy(trj.points[i].positions[j])
                v0 = copy.deepcopy(trj.points[i - 1].velocities[j])
                v1 = copy.deepcopy(trj.points[i].velocities[j])
                rospy.loginfo("curr=%f new=%f", dt, 2 * (p1 - p0) / (v0 + v1))
                dt = max(2 * (p1 - p0) / (v0 + v1), dt)

            rospy.loginfo("final dt=%f\n", dt)
            # recalculate velocities
            vel = [0.0] * 6
            for j in range(6):
                p0 = copy.deepcopy(trj.points[i - 1].positions[j])
                p1 = copy.deepcopy(trj.points[i].positions[j])
                v0 = copy.deepcopy(trj.points[i - 1].velocities[j])
                vel[j] = 2 * (p1 - p0) / dt - v0
            trj.points[i].velocities = tuple(vel)

            # recalculate accelerations
            acc = [0.0] * 6
            for j in range(6):
                v0 = copy.deepcopy(trj.points[i - 1].velocities[j])
                v1 = copy.deepcopy(trj.points[i].velocities[j])
                acc[j] = (v1 - v0) / dt
            trj.points[i].accelerations = tuple(acc)

            rospy.loginfo("Old Time = %f", trj.points[i].time_from_start.to_sec())
            trj.points[i].time_from_start = rospy.rostime.Duration(trj.points[i - 1].time_from_start.to_sec() + dt)
            rospy.loginfo("New Time = %f", trj.points[i].time_from_start.to_sec())

        return trj

    # dont use, unexpected behavior
    def recalc_time(self, trj):
        t = copy.deepcopy(trj.points[0].time_from_start)
        for i in range(len(trj.points)):
            trj.points[i].time_from_start = trj.points[i].time_from_start - t
        return trj

    # splits trajectory into chucks of N='self.splitNum' points, if possible
    def trajectory_received(self, trj):
        if len(trj.points) != 0:
            for i in range(int(math.ceil(len(trj.points) / self.splitNum)) + 1):
                t = copy.deepcopy(trj)

                t.points = trj.points[i * self.splitNum:i * self.splitNum + self.splitNum]
                # if (i+2)*self.splitNum >= len(trj.points):
                #     t.points = trj.points[i*self.splitNum:]
                # else:
                #     t.points = trj.points[i*self.splitNum:i*self.splitNum+self.splitNum]

                # merge with other trajectory + smooth
                if (i == 0) and len(self.traj_list) > 1:
                    # self.traj_list[-1], t = self.smooth_trajectory_transition(self.traj_list[-1], t)
                    t0before = copy.deepcopy(self.traj_list[-1])
                    t1before = copy.deepcopy(t)

                    self.traj_list[-1] = self.smooth_trajectory_transition(self.traj_list[-1], t)

                    for p in t1before.points:
                        t0before.points.append(p)

                    rospy.loginfo("self.traj_list[-1]:")
                    self.print_trajectories(t0before, self.traj_list[-1])

                elif len(t.points) > 0:
                    self.traj_list.append(t)

    def exec_loop(self):
        rospy.logerr('[aubo_robot_simulator_node/exec_loop] Started!')

        while not self.motion_ctrl.sig_shutdown:
            try:
                if self.motion_ctrl.is_in_motion() or self.EnableFlag == 0 or len(
                        self.traj_list) == 0 or self.motion_ctrl.points_left() >= self.splitNum:
                    # if self.motion_ctrl.is_in_motion():
                    # pass
                    # rospy.loginfo('[aubo_robot_simulator_node/exec_loop] In motion, continue...')

                    if self.EnableFlag == 0:
                        rospy.loginfo('[aubo_robot_simulator_node/exec_loop] self.EnableFlag == 0, continue...')

                    if len(self.traj_list) == 0:
                        pass
                        # rospy.loginfo('[aubo_robot_simulator_node/exec_loop] len(self.traj_list) == 0, continue...')
                    continue

                rospy.logerr('[aubo_robot_simulator_node/exec_loop] Executing...')
                msg_in = self.traj_list.pop(0)

                if len(msg_in.points) != 0:
                    rospy.logerr('[aubo_robot_simulator_node/exec_loop] msg_in.len=%d', len(msg_in.points))

                    # if len(msg_in.points) != 0:
                    self.velocity_scale_factor = rospy.get_param('/aubo_controller/velocity_scale_factor', 1.0)
                    rospy.loginfo('[aubo_robot_simulator_node/exec_loop] The velocity scale factor is: %s',
                                  str(self.velocity_scale_factor))
                    curr_traj = scale_trajectory_speed(msg_in, self.velocity_scale_factor)

                    # for point in curr_traj.points:
                    for point in curr_traj.points:
                        # first remaps point to controller joint order, the add the point to the controller.
                        point = self._to_controller_order(msg_in.joint_names, point)
                        self.motion_ctrl.add_motion_waypoint(point)


                else:
                    rospy.logerr('[aubo_robot_simulator_node/exec_loop] len(msg_in.points) == 0')

            except Exception as e:
                rospy.logerr('[aubo_robot_simulator_node/exec_loop] Exception: %s', e)

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
