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
 
 from velma_common.velma_interface import VelmaInterface, isConfigurationClose,\
     symmetricalConfiguration
 from control_msgs.msg import FollowJointTrajectoryResult
 from rcprg_ros_utils import exitError
 
 if __name__ == "__main__":
     # Define some configurations
 
     # every joint in position 0
     q_map_0 = symmetricalConfiguration( {'torso_0_joint':0,
         'right_arm_0_joint':0, 'right_arm_1_joint':0, 'right_arm_2_joint':0,
         'right_arm_3_joint':0, 'right_arm_4_joint':0, 'right_arm_5_joint':0,
         'right_arm_6_joint':0} )
 
     # starting position
     q_map_starting = symmetricalConfiguration( {'torso_0_joint':0,
         'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25,
         'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0,} )
 
     # goal position
     q_map_goal = {'torso_0_joint':0,
         'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':-1.25,
         'right_arm_3_joint':2.0, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0,
         'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5,
         'left_arm_6_joint':0 }
 
     # intermediate position
     q_map_intermediate = {'torso_0_joint':0,
         'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.6, 'right_arm_2_joint':-1.25,
         'right_arm_3_joint':-0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0,
         'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5,
         'left_arm_6_joint':0 }
 
     rospy.init_node('test_jimp')
 
     rospy.sleep(0.5)
 
     print("This test/tutorial executes simple motions"\
         " in joint impedance mode. Planning is not used"\
         " in this example.")
 
     print("Running python interface for Velma...")
     velma = VelmaInterface()
     print("Waiting for VelmaInterface initialization...")
     if not velma.waitForInit(timeout_s=10.0):
         exitError(1, msg="Could not initialize VelmaInterface")
     print("Initialization ok!")
 
     print("Motors must be enabled every time after the robot enters safe state.")
     print("If the motors are already enabled, enabling them has no effect.")
     print("Enabling motors...")
     if velma.enableMotors() != 0:
         exitError(2, msg="Could not enable motors")
 
     rospy.sleep(0.5)
 
     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         exitError(1, msg="Motors must be homed and ready to use for this test.")
 
 
     print("Switch to jnt_imp mode (no trajectory)...")
     velma.moveJointImpToCurrentPos(start_time=0.5)
     error = velma.waitForJoint()
     if error != 0:
         exitError(3, msg="The action should have ended without error,"\
                         " but the error code is {}".format(error))
 
     print("Checking if the starting configuration is as expected...")
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
         exitError(10, msg="This test requires starting pose: {}".format(q_map_starting))
 
     print("Moving to position 0 (this motion is too fast and should cause error condition,"\
             " that leads to safe mode in velma_core_cs).")
     velma.moveJoint(q_map_0, 0.05, start_time=0.5, position_tol=0, velocity_tol=0)
     error = velma.waitForJoint()
     if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
         exitError(4, msg="The action should have ended with PATH_TOLERANCE_VIOLATED"\
                     " error status, but the error code is {}".format(error) )
 
     print("Checking if current pose is close to the starting pose...")
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
         exitError(10)
 
     print("waiting 2 seconds...")
     rospy.sleep(2)
 
     print("Motors must be enabled every time after the robot enters safe state.")
     print("If the motors are already enabled, enabling them has no effect.")
     print("Enabling motors...")
     if velma.enableMotors() != 0:
         exitError(5)
 
     print("Moving to position 0 (slowly).")
     velma.moveJoint(q_map_0, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     velma.waitForJoint()
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_0, js[1], tolerance=0.1):
         exitError(10)
 
     rospy.sleep(1.0)
 
     print("Moving to the starting position...")
     velma.moveJoint(q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         exitError(6, msg="The action should have ended without error,"\
                         " but the error code is {}".format(error))
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
         exitError(10)
 
     print("Moving to valid position, using invalid self-colliding trajectory"\
         " (this motion should cause error condition, that leads to safe mode in velma_core_cs).")
     velma.moveJoint(q_map_goal, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
         exitError(7, msg="The action should have ended with PATH_TOLERANCE_VIOLATED"\
                     " error status, but the error code is {}".format(error) )
 
     print("Using relax behavior to exit self collision...")
     velma.switchToRelaxBehavior()
 
     print("waiting 2 seconds...")
     rospy.sleep(2)
 
     print("Motors must be enabled every time after the robot enters safe state.")
     print("If the motors are already enabled, enabling them has no effect.")
     print("Enabling motors...")
     if velma.enableMotors() != 0:
         exitError(8)
 
     print("Moving to the starting position...")
     velma.moveJoint(q_map_starting, 4.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         exitError(9, msg="The action should have ended without error,"\
                         " but the error code is {}".format(error))
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
         exitError(10)
 
     print("To reach the goal position, some trajectory must be exetuted that contains additional,"\
             " intermediate nodes")
 
     print("Moving to the intermediate position...")
     velma.moveJoint(q_map_intermediate, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         exitError(10, msg="The action should have ended without error,"\
                         " but the error code is {}".format(error))
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_intermediate, js[1], tolerance=0.1):
         exitError(10)
 
     print("Moving to the goal position.")
     velma.moveJoint(q_map_goal, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         exitError(11, msg="The action should have ended with PATH_TOLERANCE_VIOLATED"\
                         " error status, but the error code is {}".format(error) )
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_goal, js[1], tolerance=0.1):
         exitError(10)
 
     print("Moving to the intermediate position...")
     velma.moveJoint(q_map_intermediate, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         exitError(12, msg="The action should have ended without error,"\
                         " but the error code is {}".format(error))
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_intermediate, js[1], tolerance=0.1):
         exitError(10)
 
     print("Moving to the starting position...")
     velma.moveJoint(q_map_starting, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         exitError(13, msg="The action should have ended without error,"\
                         " but the error code is {}".format(error))
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
         exitError(10)
 
     exitError(0)
 