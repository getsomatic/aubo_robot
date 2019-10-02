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

import rospy
import copy
import threading
import Queue
# Publish
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8
# Subscribe
from trajectory_msgs.msg import JointTrajectoryPoint

"""
MotionController
This class simulates the motion controller for an Aubo robot.
This class IS threadsafe
"""


class MotionControllerSimulator:
    """
    Constructor of motion controller simulator
    """

    def __init__(self, num_joints, update_rate=200, buffer_size=0):
        # Class lock for only 1 thread to use it at same time
        self.lock = threading.Lock()

        # Sets to 1 if 'stop' message received
        self.cancle_trajectory = 0

        # Motion loop update rate (higher update rates result in smoother simulated motion)
        self.update_rate = update_rate
        rospy.loginfo("Setting motion update rate (hz): %f", self.update_rate)

        # Initialize motion buffer (contains joint position lists)
        self.motion_buffer = Queue.Queue(buffer_size)
        rospy.logdebug("Setting motion buffer size: %i", buffer_size)

        def_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.joint_names = rospy.get_param('controller_joint_names', def_joint_names)

        initial_joint_state = rospy.get_param('initial_joint_state', [0] * num_joints)

        # same_len = len(initial_joint_state) == len(self.joint_names)
        # same_len = 6
        initial_joint_state = [0] * num_joints
        # all_num  = all(type(x) is int or type(x) is float for x in initial_joint_state)
        # if not same_len or not all_num:
        #     initial_joint_state = [0]*num_joints
        #     rospy.logwarn("Invalid initial_joint_state parameter, defaulting to all-zeros "
        #                   "(len: %s, types: %s).", same_len, all_num)
        rospy.loginfo("Using initial joint state: %s", str(initial_joint_state))

        # Initialize joint position
        self.joint_positions = initial_joint_state
        rospy.logdebug("Setting initial joint state: %s", str(initial_joint_state))
        self.joint_velocities = [0] * num_joints
        self.joint_accelerations = [0] * num_joints

        self.ribBufferSize = 0
        self.ControllerConnectedFlag = 1
        self.positionUpdatedFlag = '0'

        rospy.logdebug("Creating joint state publisher")
        self.moveit_joint_state_pub = rospy.Publisher('moveItController_cmd', JointTrajectoryPoint, queue_size=2000)

        rospy.logdebug("Creating joint state subscriber!")
        self.update_joint_state_subs = rospy.Subscriber('/aubo_driver/real_pose', Float32MultiArray,
                                                        self.update_joint_state_callback)
        self.Is_cancel_trajectory_subs = rospy.Subscriber('aubo_driver/cancel_trajectory', UInt8,
                                                          self.Is_cancel_trajectory_callback)

        self.MinimumBufferSize = 800
        # Shutdown signal
        self.sig_shutdown = False

        # Stop signal
        self.sig_stop = False

        # Motion thread
        self.motion_thread = threading.Thread(target=self._motion_worker)
        self.motion_thread.daemon = True
        self.motion_thread.start()

    """
    Update the joint position from extern controller.
    """

    def update_joint_state_callback(self, msg_in):
        if not self.is_in_motion():
            self.joint_positions = msg_in.data
            # rospy.loginfo('update joints, %s' , str(self.joint_positions))

    # The function will recieve info about cancel trajectory because user want the robot stop when the robot running
    def Is_cancel_trajectory_callback(self, msg_in):
        self.cancle_trajectory = msg_in.data
        rospy.loginfo('is cancel trajectory status %d', self.cancle_trajectory)

    def add_motion_waypoint(self, point):
        # When add new trajectory into the buffer, here need to handle the accelerations!!
        self.motion_buffer.put(point)

    def get_joint_positions(self):
        with self.lock:
            return self.joint_positions[:]

    """
    return the status of the controller.
    """

    def is_in_motion(self):
        return not self.motion_buffer.empty()

    def points_left(self):
        return self.motion_buffer.qsize()

    def shutdown(self):
        self.sig_shutdown = True
        # change to logdebug later?
        rospy.loginfo('Motion_Controller shutdown signaled')

    def stop(self):
        # change to logdebug later?
        rospy.loginfo('Motion_Controller stop signaled')
        with self.lock:
            self._clear_buffer()
            self.sig_stop = True

    def interpolate(self, last_pt, current_pt, alpha):
        intermediate_pt = JointTrajectoryPoint()
        for last_joint, current_joint in zip(last_pt.positions, current_pt.positions):
            intermediate_pt.positions.append(last_joint + alpha * (current_joint - last_joint))
        intermediate_pt.time_from_start = last_pt.time_from_start + rospy.Duration(
            alpha * (current_pt.time_from_start.to_sec() - last_pt.time_from_start.to_sec()))
        return intermediate_pt

    def _clear_buffer(self):
        with self.motion_buffer.mutex:
            self.motion_buffer.queue.clear()

    def joint_state_publisher(self):
        try:
            joint_point_msg = JointTrajectoryPoint()
            time = rospy.Time.now()

            with self.lock:
                # Joint states
                joint_point_msg.time_from_start = time
                joint_point_msg.positions = self.joint_positions
                joint_point_msg.velocities = self.joint_velocities
                joint_point_msg.accelerations = self.joint_accelerations
                # print('sending: ' + str(time)
                #      + ' ' + str(self.joint_positions[0]) + ' ' + str(self.joint_positions[1]) + ' ' + str(self.joint_positions[2])
                #      + ' ' + str(self.joint_positions[3]) + ' ' + str(self.joint_positions[4]) + ' ' + str(self.joint_positions[5])
                #      + ' ' + str(self.joint_velocities[0]) + ' ' + str(self.joint_velocities[1]) + ' ' + str(self.joint_velocities[2])
                #      + ' ' + str(self.joint_velocities[3]) + ' ' + str(self.joint_velocities[4]) + ' ' + str(self.joint_velocities[5])
                #      + ' ' + str(self.joint_accelerations[0]) + ' ' + str(self.joint_accelerations[1]) + ' ' + str(self.joint_accelerations[2])
                #      + ' ' + str(self.joint_accelerations[3]) + ' ' + str(self.joint_accelerations[4]) + ' ' + str(self.joint_accelerations[5])
                #  )
                # rospy.loginfo('Moved to position: %s', str(self.joint_positions))
                self.moveit_joint_state_pub.publish(joint_point_msg)

        except Exception as e:
            rospy.loginfo('Unexpected exception in joint state publisher: %s', e)

    """
    update the motion controller state combining with the robot controller information
    """

    def _move_to(self, point, dur):
        while self.ribBufferSize > self.MinimumBufferSize:
            print(self.ribBufferSize)
            rospy.loginfo('too fast!!')
            rospy.loginfo('The number of driver Buffer Data is big enough, not send this time: %d', self.ribBufferSize)
            rospy.sleep(dur)

        if self.ribBufferSize == 0 and self.ControllerConnectedFlag == 0:  # motion start or no robot connected!
            rospy.sleep(dur)

        if self.ribBufferSize == 0 and self.ControllerConnectedFlag == 1:  # motion start but with robot connected!
            pass
            # rospy.loginfo('too slow slow!!')

        with self.lock:
            if not self.sig_stop:

                self.joint_positions = point.positions
                self.joint_velocities = point.velocities
                self.joint_accelerations = point.accelerations
                # self.joint_state_publisher()                #publish the control command to the robot
                rospy.logdebug('Moved to position: %s in %s', str(self.joint_positions), str(dur))
            else:
                rospy.loginfo('Stopping motion immediately, clearing stop signal')
                self.sig_stop = False

    def _motion_worker(self):
        # first update the robot real position

        self.positionUpdatedFlag = rospy.get_param('/aubo_driver/robot_connected', '0')
        while self.positionUpdatedFlag == '0':
            rospy.sleep(self.update_rate / 400)
            self.positionUpdatedFlag = rospy.get_param('/aubo_driver/robot_connected', '0')

        rospy.loginfo('Starting motion worker in motion controller simulator')
        move_duration = rospy.Duration()
        if self.update_rate <> 0.:
            update_duration = rospy.Duration(1. / self.update_rate)
        last_goal_point = JointTrajectoryPoint()

        with self.lock:
            last_goal_point.positions = self.joint_positions[:]

        while not self.sig_shutdown:
            try:
                # rospy.loginfo('processing queue --')
                # rospy.loginfo("getting item from queue")
                current_goal_point = self.motion_buffer.get()
                # rospy.loginfo("processing item. " + str(self.motion_buffer.qsize()))
                intermediate_goal_point = copy.deepcopy(current_goal_point)
                # if current_goal_point.time_from_start == last_goal_point.time_from_start:
                #    print("SAME TIME")
                #    continue
                # If the current time from start is less than or equal to the last, then it's a new trajectory
                if current_goal_point.time_from_start < last_goal_point.time_from_start:
                    move_duration = current_goal_point.time_from_start
                    rospy.loginfo("NEW TRAJECTORY DETECTED!\n")
                # Else it's an existing trajectory and subtract the two
                else:
                    # If current move duration is greater than update_duration, interpolate the joint positions to form a smooth trajectory
                    # Provide an exception to this rule: if update rate is <=0, do not add interpolated points
                    T = current_goal_point.time_from_start.to_sec() - last_goal_point.time_from_start.to_sec()
                    # five degree polynomial interpolation
                    # a0 = last_goal_point.positions[:]
                    a1 = last_goal_point.velocities[:]
                    N = len(a1)
                    T2 = T * T
                    T3 = T2 * T
                    T4 = T3 * T
                    T5 = T4 * T
                    a2 = [0] * N
                    h = [0] * N
                    a3 = [0] * N
                    a4 = [0] * N
                    a5 = [0] * N
                    for i in range(0, N):
                        a2[i] = 0.5 * last_goal_point.accelerations[i]
                        h[i] = current_goal_point.positions[i] - last_goal_point.positions[i]
                        a3[i] = 0.5 / T3 * (20 * h[i] - (
                                    8 * current_goal_point.velocities[i] + 12 * last_goal_point.velocities[i]) * T - (
                                                        3 * last_goal_point.accelerations[i] -
                                                        current_goal_point.accelerations[i]) * T2)
                        a4[i] = 0.5 / T4 * (-30 * h[i] + (
                                    14 * current_goal_point.velocities[i] + 16 * last_goal_point.velocities[i]) * T + (
                                                        3 * last_goal_point.accelerations[i] - 2 *
                                                        current_goal_point.accelerations[i]) * T2)
                        a5[i] = 0.5 / T5 * (12 * h[i] - 6 * (
                                    current_goal_point.velocities[i] + last_goal_point.velocities[i]) * T + (
                                                        current_goal_point.accelerations[i] -
                                                        last_goal_point.accelerations[i]) * T2)
                    if self.update_rate > 0:
                        tt = last_goal_point.time_from_start.to_sec()
                        ti = tt
                        while (tt < current_goal_point.time_from_start.to_sec()) and (0 == self.cancle_trajectory):
                            t1 = tt - ti
                            t2 = t1 * t1
                            t3 = t2 * t1
                            t4 = t3 * t1
                            t5 = t4 * t1
                            for i in range(0, N):
                                intermediate_goal_point.positions[i] = last_goal_point.positions[i] + a1[i] * t1 + a2[
                                    i] * t2 + a3[i] * t3 + a4[i] * t4 + a5[i] * t5
                                intermediate_goal_point.velocities[i] = a1[i] + 2 * a2[i] * t1 + 3 * a3[i] * t2 + 4 * \
                                                                        a4[i] * t3 + 5 * a5[i] * t4
                                intermediate_goal_point.accelerations[i] = 2 * a2[i] + 6 * a3[i] * t1 + 12 * a4[
                                    i] * t2 + 20 * a5[i] * t3
                            tt = tt + update_duration.to_sec()
                            # rospy.loginfo('move to 1:' + str(intermediate_goal_point))
                            self._move_to(intermediate_goal_point, update_duration.to_sec())
                            self.joint_state_publisher()
                            rospy.sleep(update_duration.to_sec())

                        # here need to do some adjustment to make the trajectory smoother
                        move_duration = current_goal_point.time_from_start.to_sec() - tt

                # rospy.loginfo('move to 2:' + str(current_goal_point))
                self._move_to(current_goal_point, move_duration)
                self.joint_state_publisher()
                rospy.sleep(move_duration)
                last_goal_point = copy.deepcopy(current_goal_point)

            except Exception as e:
                rospy.logerr('Unexpected exception: %s', e)

            # if user want stop ,we will clear Trajectory buffer data from moveit
            if self.cancle_trajectory != 0:
                self.cancle_trajectory = 0
                #self._clear_buffer()
                while not self.motion_buffer.empty():
                    self.motion_buffer.get()

        rospy.logdebug("Shutting down motion controller")
