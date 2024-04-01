#!/usr/bin/env python
 
 
 
 # Copyright (c) 2017, Robot Control and Pattern Recognition Group,
 # Institute of Control and Computation Engineering
 # Warsaw University of Technology
 #
 # All rights reserved.
 # 
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #     * Redistributions of source code must retain the above copyright
 #       notice, this list of conditions and the following disclaimer.
 #     * Redistributions in binary form must reproduce the above copyright
 #       notice, this list of conditions and the following disclaimer in the
 #       documentation and/or other materials provided with the distribution.
 #     * Neither the name of the Warsaw University of Technology nor the
 #       names of its contributors may be used to endorse or promote products
 #       derived from this software without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 # Author: Dawid Seredynski
 #
 
 import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
 
 import rospy
 import math
 import PyKDL
 
 from velma_common import *
 from rcprg_planner import *
 from rcprg_ros_utils import exitError
 
 if __name__ == "__main__":
     # define some configurations
     q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
     q_map_goal = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':-1.25, 'right_arm_3_joint':1.57, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
     rospy.init_node('test_jimp_planning', anonymous=False)
 
     rospy.sleep(0.5)
 
     print "This test/tutorial executes complex motions"\
         " in Joint Impedance mode. Planning is used"\
         " in this example.\n"
 
     print "Running python interface for Velma..."
     velma = VelmaInterface()
     print "Waiting for VelmaInterface initialization..."
     if not velma.waitForInit(timeout_s=10.0):
         print "Could not initialize VelmaInterface\n"
         exitError(1)
     print "Initialization ok!\n"
 
     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         print "Motors must be homed and ready to use for this test."
         exitError(1)
 
     print "Waiting for Planner initialization..."
     p = Planner(velma.maxJointTrajLen())
     if not p.waitForInit(timeout_s=10.0):
         print "Could not initialize Planner"
         exitError(2)
     print "Planner initialization ok!"
 
     print "Motors must be enabled every time after the robot enters safe state."
     print "If the motors are already enabled, enabling them has no effect."
     print "Enabling motors..."
     if velma.enableMotors() != 0:
         exitError(3)
 
     print "Switch to jnt_imp mode (no trajectory)..."
     velma.moveJointImpToCurrentPos(start_time=0.5)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(4)
 
     print "Checking if the starting configuration is as expected..."
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
         print "This test requires starting pose:"
         print q_map_starting
         exitError(10)
 
     print "Planning motion to the goal position using set of all joints..."
 
     print "Moving to valid position, using planned trajectory."
     goal_constraint_1 = qMapToConstraints(q_map_goal, 0.01, group=velma.getJointGroup("impedance_joints"))
     for i in range(15):
         rospy.sleep(0.5)
         js = velma.getLastJointState()
         print "Planning (try", i, ")..."
         traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
         if traj == None:
             continue
         print "Executing trajectory..."
         if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
             exitError(5)
         if velma.waitForJoint() == 0:
             break
         else:
             print "The trajectory could not be completed, retrying..."
             continue
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_goal, js[1]):
         exitError(6)
 
     rospy.sleep(1.0)
 
     print "Moving to starting position, using planned trajectory."
     goal_constraint_2 = qMapToConstraints(q_map_starting, 0.01, group=velma.getJointGroup("impedance_joints"))
     for i in range(15):
         rospy.sleep(0.5)
         js = velma.getLastJointState()
         print "Planning (try", i, ")..."
         traj = p.plan(js[1], [goal_constraint_2], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
         if traj == None:
             continue
         print "Executing trajectory..."
         if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
             exitError(7)
         if velma.waitForJoint() == 0:
             break
         else:
             print "The trajectory could not be completed, retrying..."
             continue
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1]):
         exitError(8)
 
     print "Planning motion to the same goal position using subset of joints (right arm only)..."
 
     print "Moving to valid position, using planned trajectory."
     goal_constraint_1 = qMapToConstraints(q_map_goal, 0.01, group=velma.getJointGroup("right_arm"))
     for i in range(15):
         rospy.sleep(0.5)
         js = velma.getLastJointState()
         print "Planning (try", i, ")..."
         traj = p.plan(js[1], [goal_constraint_1], "right_arm", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
         if traj == None:
             continue
         print "Executing trajectory..."
         if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
             exitError(9)
         if velma.waitForJoint() == 0:
             break
         else:
             print "The trajectory could not be completed, retrying..."
             continue
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_goal, js[1]):
         exitError(10)
 
     rospy.sleep(1.0)
 
     print "Moving to starting position, using planned trajectory."
     goal_constraint_2 = qMapToConstraints(q_map_starting, 0.01, group=velma.getJointGroup("right_arm"))
     for i in range(15):
         rospy.sleep(0.5)
         js = velma.getLastJointState()
         print "Planning (try", i, ")..."
         traj = p.plan(js[1], [goal_constraint_2], "right_arm", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
         if traj == None:
             continue
         print "Executing trajectory..."
         if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
             exitError(11)
         if velma.waitForJoint() == 0:
             break
         else:
             print "The trajectory could not be completed, retrying..."
             continue
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1]):
         exitError(12)
 
     exitError(0)