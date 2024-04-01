#!/usr/bin/env python
 
 
 
 # Copyright (c) 2021, Robot Control and Pattern Recognition Group,
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
 import sys
 import math
 from velma_common import VelmaInterface
 from rcprg_ros_utils import exitError
 
 def deg2rad(deg):
     return float(deg)/180.0*math.pi
 
 def printUsage():
     print('Usage:')
     print('set_gripper_configuration left|right reset')
     print('set_gripper_configuration left|right <q_f1_deg> <q_f2_deg> <q_f3_deg> <q_spread_deg>')
 
 if __name__ == "__main__":
 
     reset = False
     if len(sys.argv) == 6:
         q_f1_deg = float(sys.argv[2])
         q_f2_deg = float(sys.argv[3])
         q_f3_deg = float(sys.argv[4])
         q_spread_deg = float(sys.argv[5])
     elif len(sys.argv) == 3 and sys.argv[2] == 'reset':
         reset = True
     else:
         printUsage()
         exitError(1, msg='Wrong arguments')
 
     side = sys.argv[1]
     if side != 'left' and side != 'right':
         printUsage()
         exitError(2, msg='Wrong side: "{}"'.format(side))
 
     rospy.init_node('set_gripper_configuration', anonymous=False)
     rospy.sleep(1)
     print "waiting for init..."
 
     velma = VelmaInterface()
 
     print "Waiting for VelmaInterface initialization..."
     if not velma.waitForInit(timeout_s=10.0):
         print "Could not initialize VelmaInterface\n"
         exitError(3)
 
     print "Initialization ok!\n"
 
     diag = velma.getCoreCsDiag()
     if (not diag.inStateCartImp()) and (not diag.inStateJntImp()):
         exitError(4, msg='Velma should be in cart_imp or jnt_imp state')
 
     if reset:
         velma.resetHand(side)
         if velma.waitForHand(side) != 0:
             exitError(5)
         exitError(0)
 
     q = [deg2rad(q_f1_deg), deg2rad(q_f2_deg), deg2rad(q_f3_deg), deg2rad(q_spread_deg)]
     velma.moveHand(side, q, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=False)
     if velma.waitForHand(side) != 0:
         exitError(6)
     exitError(0)