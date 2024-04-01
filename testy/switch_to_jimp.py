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
 
 from velma_common.velma_interface import *
 from control_msgs.msg import FollowJointTrajectoryResult
 from rcprg_ros_utils import exitError
 
 if __name__ == "__main__":
     rospy.init_node('switch_to_jimp')
     rospy.sleep(0.5)
 
     print('Running python interface for Velma...')
     velma = VelmaInterface()
     print('Waiting for VelmaInterface initialization...')
     if not velma.waitForInit(timeout_s=10.0):
         exitError(1, msg='Could not initialize VelmaInterface')
     print('Initialization ok!')
 
     print('Motors must be enabled every time after the robot enters safe state.')
     print('If the motors are already enabled, enabling them has no effect.')
     print('Enabling motors...')
     if velma.enableMotors() != 0:
         exitError(2, msg='Could not enable motors')
 
     rospy.sleep(0.5)
 
     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         exitError(3, msg='Motors must be homed and ready to use for this test.')
 
     print('Switch to jnt_imp mode (no trajectory)...')
     velma.moveJointImpToCurrentPos(start_time=0.5)
     error = velma.waitForJoint()
     if error != 0:
         exitError(4, msg='The action should have ended without error,'\
                             ' but the error code is {}'.format(error))
 
     rospy.sleep(0.5)
     diag = velma.getCoreCsDiag()
     if not diag.inStateJntImp():
         exitError(5, msg='The control system should be in jnt_imp mode')
 
     exitError(0)
