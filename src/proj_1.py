#! /usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL
import numpy as np
from math  import pi

from velma_common.velma_interface import VelmaInterface
from rcprg_ros_utils import exitError

from visualization_msgs.msg import *

from rcprg_ros_utils.marker_publisher import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

import tf_conversions.posemath


class VelmitronoManipulinator:
    def __init__(self):
        rospy.init_node('proj_1')
        self.left_gripper = True # gripper open
        self.right_gripper = True # gripper open

        # starting position
        self.home_postions = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0}
        rospy.logwarn("Nobody expects the spanish inquisition") # xD

        rospy.loginfo("Waiting for Velmitron initialization...")
        self.velma = VelmaInterface()
        if not self.velma.waitForInit(timeout_s=10.0):
            rospy.logerr("Could not initialize VelmaInterface\n")
            exitError(1)
        rospy.loginfo("Initialization ok!\n")

        rospy.loginfo("Enabling motors...")
        if self.velma.enableMotors() != 0:
            exitError(2)

        self.velma.startHomingHP()
        if self.velma.waitForHP() != 0:
            exitError(2)
        rospy.loginfo("Head pan motor homing successful.")

        rospy.loginfo("Sending head tilt motor START_HOMING command...")
        self.velma.startHomingHT()
        if self.velma.waitForHT() != 0:
            exitError(3)
        rospy.loginfo("Head tilt motor homing successful.")

        self.diag = self.velma.getCoreCsDiag()
        if not self.diag.motorsReady():
            rospy.logerr("Motors must be homed and ready to use for this test.")
            exitError(3)

        rospy.loginfo("Waiting for Planner init...")
        # self.velma.maxJointTrajLen = 100
        self.p = Planner(self.velma.maxJointTrajLen())
        if not self.p.waitForInit():
            rospy.logerr("could not initialize PLanner")
            exitError(4)
        rospy.loginfo("Planner initialized")

        rospy.loginfo("Waiting for Octomap init...")
        oml = OctomapListener("/octomap_binary")
        if not oml:
                rospy.logerr("could not initialize Octomap")
                exitError(5)
        rospy.loginfo("Octomap initialized")

        rospy.loginfo("Getting Octomap")
        octomap = oml.getOctomap(timeout_s=5.0)
        if not octomap:
            rospy.logerr("Octomap loading failed")
            exitError(6)
        rospy.loginfo("Getting Octomap Succesful")

        rospy.loginfo("Processing Octomap")
        self.p.processWorld(octomap)
        if not self.p.processWorld(octomap):
            rospy.logerr("Processing Octomap Failed")
            exitError(7)
        rospy.loginfo("Processing Octomap Succesful")

        # Homing
        # self.homing(True) # grippers only
        self.homing()

        rospy.loginfo("Ready to work!")

    def homing(self, grippers_only=False):
        rospy.loginfo("Homing Velimtron...")
        if not grippers_only:
            self.goto_start_position()
            rospy.loginfo("Starting position reached")
        rospy.loginfo("Closing grippers...")
        self.change_gripper_state("left", state="close")
        self.change_gripper_state("right", state="close")
        rospy.loginfo("Gripper close")
        self.right_gripper = False
        self.left_gripper = False
        rospy.loginfo("Velimtron home alone")

    def goto_start_position(self):
        self.velma.moveJoint(self.home_postions,8.0, start_time=0.1, position_tol=15.0/180.0*math.pi)
        self.move_head(0, 0)
        joint_error = self.velma.waitForJoint()
        head_error = self.velma.waitForHead()
        if joint_error != 0 and head_error != 0:
            rospy.logerr("Can not go to start position!")
            rospy.logwarn("Retrying...")
            self.velma.moveJoint(self.home_postions,8.0, start_time=0.1, position_tol=15.0/180.0*math.pi)
            joint_error = self.velma.waitForJoint()
            head_error = self.velma.waitForHead()
            if joint_error != 0 and head_error != 0:
                rospy.logerr("Retrying faild!")
                exitError(999)

    def normalize_angle(self,angle):
        return ( (angle + pi) % (2*pi) ) - pi

    def move_head(self, rotate, slope):
        cords = (rotate, slope)
        curr_rotate, curr_slope = self.velma.getHeadCurrentConfiguration()
        rotate_time = abs(self.normalize_angle(curr_rotate-rotate))*2 + 0.5
        slope_time = abs(self.normalize_angle(curr_slope-slope))*1 + 0.5
        action_time = max(rotate_time, slope_time)
        self.velma.moveHead(cords, action_time, start_time=0.5)
        if self.velma.waitForHead() != 0:
            rospy.logerr("Can not move head!")
            rospy.logwarn("Retrying...")
            self.velma.moveHead(cords, action_time, start_time=0.5)
            if self.velma.waitForHead() != 0:
                rospy.logerr("Retrying faild!")
                exitError(950)

    def joint_mode(self):
        rospy.loginfo("Switching to jnt_imp mode...")
        self.velma.moveJointImpToCurrentPos(start_time=0.2)
        error = self.velma.waitForJoint()
        if error != 0:
            rospy.logerr("Error with switching to jnt_imp mode")
            rospy.logwarn("Retrying...")
            self.velma.enableMotors()
            self.velma.moveJointImpToCurrentPos(start_time=0.2)
            diag = self.velma.getCoreCsDiag()
            diag = self.velma.getCoreCsDiag()
            if not diag.inStateJntImp():
                rospy.logerr("Retrying faild!")
                exitError(100)
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateJntImp():
            rospy.logerr("The core_cs should be in jnt_imp state, but it is not")
            rospy.logwarn("Retrying...")
            self.velma.enableMotors()
            self.velma.moveJointImpToCurrentPos(start_time=0.2)
            diag = self.velma.getCoreCsDiag()
            diag = self.velma.getCoreCsDiag()
            if not diag.inStateJntImp():
                rospy.logerr("Retrying faild!")
                exitError(101)
        self.mode = "joint"
        rospy.loginfo("Switch to jnt_imp mode")

    def cart_mode(self):
        rospy.loginfo("Switching to cart_imp mode...")
        self.velma.moveCartImpRightCurrentPos(start_time=0.2)
        if not self.velma.moveCartImpRightCurrentPos(start_time=0.2):
            rospy.logerr("Error with switching to cart_imp mode")
            rospy.logwarn("Retrying...")
            self.velma.enableMotors()
            self.velma.moveCartImpRightCurrentPos(start_time=0.2)
            if self.velma.waitForEffectorRight() != 0:
                rospy.logerr("Retrying faild!")
            exitError(102)
        if self.velma.waitForEffectorRight() != 0:
            rospy.logerr("The core_cs should be in cart_imp state, but it is not")
            rospy.logwarn("Retrying...")
            self.velma.enableMotors()
            self.velma.moveCartImpRightCurrentPos(start_time=0.2)
            if self.velma.waitForEffectorRight() != 0:
                rospy.logerr("Retrying faild!")
                exitError(103)
        rospy.loginfo("Switch to cart_imp mode")

    def gripper_state(self, side):
        if side == "right":
            return self.right_gripper
        else:
            return self.left_gripper

    def change_gripper_state(self, side="right", is_holding=False, state=None):
        q_close = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
        angle = 0
        q_open = [math.radians(angle), math.radians(angle), math.radians(angle), math.radians(0)]
        q = q_close if self.gripper_state(side) or state=="close" else q_open # gripper status
        if side == "right":
            self.velma.moveHandRight(q, [1, 1, 1, 1], [0.02,0.02,0.02,0.02], 0.01, hold=is_holding)
            if self.velma.waitForHandRight() != 0:
                exitError("Gripper Error code: ", 200)
            self.right_gripper = not self.right_gripper # changing gripper status
            mess = "Right gripper is open" if self.right_gripper else "Right gripper is close"
            rospy.loginfo(mess)
        else:
            self.velma.moveHandLeft(q, [1, 1, 1, 1],  [0.02, 0.02, 0.02, 0.02], 0.01, hold=is_holding)
            if self.velma.waitForHandLeft() != 0:
                exitError("Gripper Error code: ", 201)
            self.left_gripper = not self.left_gripper # changing gripper status
            mess = "Left gripper is open" if self.left_gripper else "Left gripper is close"
            rospy.loginfo(mess)

    def plan_and_execute(self, q_dest, hand=None, accuracy=0.01):
        rospy.loginfo("Planning motion to the goal position using set of all joints...")
        rospy.loginfo("Moving to valid position, using planned trajectory.")
        if hand is not None:
            join = '{}'.format(hand)+"_arm_torso"
        else:
            pass
        for _ in range(5):
            goal_constraints = [qMapToConstraints(q, accuracy, group=self.velma.getJointGroup(join)) for q in q_dest]
            js = self.velma.getLastJointState()
            rospy.loginfo("Planing...")
            traj = self.p.plan(js[1], goal_constraints, join, num_planning_attempts=40, max_velocity_scaling_factor=0.15, planner_id="RTTConnect")
            if traj == None:
                accuracy += 0.05
                continue
            rospy.loginfo("Executing trajectory...")
            if not self.velma.moveJointTraj(traj, start_time=0.5):
                rospy.logerr("Error, retrying...")
                if not self.velma.moveJointTraj(traj, start_time=0.5):
                    exitError(500)
            if self.velma.waitForJoint() == 0:
                break
            else:
                rospy.logwarn("The trajectory could not be completed, retrying...")
                accuracy += 0.05
                continue

    def inv_kin(self, twe_goal, arm_name="right"):
        solv = KinematicsSolverVelma()

        torso_angle = 0.0
        arm_q = []
        for elbow_circle_angle in np.linspace(-math.pi, math.pi, 24):
            for torso_angle in np.linspace(-1.2, 1.2, 30):
                q = solv.calculateIkArm(arm_name, twe_goal, torso_angle, elbow_circle_angle, True, True, True)
                if not q[0] is None:
                    q_dict = {}
                    q_dict["torso_0_joint"] = torso_angle
                    for i in range(len(q)):
                            q_dict['{}'.format(arm_name)+'_arm_{}_joint'.format(i)] = q[i]
                    if q_dict is not None:
                        arm_q.append(q_dict)
        if len(arm_q) > 0:
            info = "IK found: " + str(len(arm_q)) + " options"
            rospy.loginfo(info)
        else:
            rospy.logwarn("No way!")
            return arm_q
        return arm_q

    def transformations_object(self, iteam, x=0, y=0, z=0, hand=None):
        TWOm = self.velma.getTf("B", iteam)
        pose = tf_conversions.posemath.toMsg(TWOm)
        if hand == None:
            if pose.position.y>=0:
                hand="left"
            else:
                hand="right"
        if hand=="left":
            r = PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0))
            v = PyKDL.Vector(x, y, z)
            TGlPl = self.velma.getTf("Gl", "Pl")
            TPlEl = self.velma.getTf("Pl", "El")
            TOGl = PyKDL.Frame(r, v)
            TWEm = TWOm*TOGl*TGlPl*TPlEl
        elif hand=="right":
            r = PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0))
            v = PyKDL.Vector(x, y, z)
            TGrPr = self.velma.getTf("Gr", "Pr")
            TPrEr = self.velma.getTf("Pr", "Er")
            TOGr = PyKDL.Frame(r, v)
            TWEm = TWOm*TOGr*TGrPr*TPrEr
        rospy.loginfo("Transformation complete")
        return TWEm, hand

    def move_around(self):
        head_positions = [(pi/2, pi/2), (pi/2, 0), (pi/2, -pi/2), (0, pi/2), (0, 0), (0, -pi/2), (-pi/2, pi/2), (-pi/2, 0), (-pi/2, -pi/2), (0, 0)]
        torso_0_joint_positions = [np.linspace(-pi/2, pi/4, pi/2)]

        for position in torso_0_joint_positions:
            self.velma.moveJoint(position, 8.0, start_time=0.1, position_tol=15.0/180.0*math.pi)
            for head_position in head_positions:
                self.move_head(head_position[0], head_position[1])

    def cart_move(self, object, hand, distance=0.12, up=False): #prawa graba 12 normalnie 5 bo octo
        rospy.sleep(5)

        r = PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0))
        x = distance
        x = 2*x if up else x # ToDo now z gripper w druga strone
        v = PyKDL.Vector(0, 0, x)
        d = PyKDL.Frame(r, v)

        if hand == "left":
            T_G_Pm = self.velma.getTf("Gl", "Pl")
            T_P_Em = self.velma.getTf("Pl", "El")
        else:
            T_G_Pm = self.velma.getTf("Gr", "Pr")
            T_P_Em = self.velma.getTf("Pr", "Er")
        goal =  object*d*T_G_Pm*T_P_Em

        rospy.loginfo("Start moving in cart mode...\nTheoretically...")
        self.velma.moveCartImp(hand, [goal], [1.0],None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
        if not self.velma.moveCartImp(hand, [goal], [1.0],  None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            rospy.logerr("Help me its 2 am i wanna go to sleep")
            exitError(13)
        rospy.sleep(5)
        self.velma.waitForJoint()
        rospy.loginfo("Move done")

    def find_the_best_corrner(self, length, table):
        best_wektor = [None, 0]
        x = table.p.x()
        y = table.p.y()
        ends = [(-length, -length), (length, -length), (-length, length), (length, length),(-(length/2), 0), ((length/2), 0), (0, (length/2)), (0, (length/2))]
        for wektor in ends:
            table_distance = (x+wektor[0])**2 + (y+wektor[1])**2
            if table_distance < best_wektor[1] or best_wektor[0] is None:
                best_wektor[0] = wektor
                best_wektor[1] = table_distance
        rospy.loginfo("Found new landing location")
        return best_wektor[0]


def main():
    s = VelmitronoManipulinator()
    s.joint_mode()

    # loading objects
    box = s.velma.getTf("B", "box1")
    table_b = s.velma.getTf("B", "table_b")
    table_c = s.velma.getTf("B", "table_c")
    # print("pudelko: ",box)
    #print("stol 1: ",table_b)
    #print("stol 2: ",table_c)



    # determine which table is the starting table
    starting_table = None
    table_b_distance = (box.p.x()-table_b.p.x())**2 + (box.p.y()-table_b.p.y())**2
    table_c_distance = (box.p.x()-table_c.p.x())**2 + (box.p.y()-table_c.p.y())**2
    if table_b_distance > table_c_distance:
        starting_table = "table_c"
    else:
        starting_table = "table_b"

    # position of box
    twe, handtype = s.transformations_object("box1", z=0.1) # dla prawej lapy 10 normalnie 5 octo
    #print("TWE: ", twe, "Hand: ", hand)
    
    # going to box
    rospy.loginfo("Going for object")
    q_map_goal = s.inv_kin(twe, handtype)
    if len(q_map_goal) == 0:
        rospy.logerr("No path to target...")
        exitError(1120)

    s.plan_and_execute(q_map_goal, handtype, 0.02)
    s.change_gripper_state(handtype) # open

    rospy.loginfo("Above object")
    rospy.logerr("Death from above") # xD
    s.cart_mode()
    s.cart_move(box,handtype)

    s.change_gripper_state(handtype, True) # close
    rospy.loginfo("I am holding the object!")

    # going up
    rospy.loginfo("Going up")
    s.cart_move(box,handtype, up=True)
    s.joint_mode()


    # going to target table
    rospy.loginfo("Going to the target table")

    if starting_table == "table_c":
        table = table_b
    else:
        table = table_c

    target = "table_c" if starting_table == "table_b" else "table_b"
    target_wektor, _ = s.transformations_object(target, z=0.85, hand=handtype)
    q_map_goal = s.inv_kin(target_wektor, handtype)

    if len(q_map_goal) == 0:
        # testing other localizations on the table
        rospy.logwarn("Can not go to center of target table...")
        rospy.loginfo("Looking for way to some corner...")
        length = 0.2 if starting_table == "table_c" else 0.4
        wektor = s.find_the_best_corrner(length, table)
        target_wektor, _ = s.transformations_object(target, x=wektor[0], y=wektor[1], z=0.85, hand=handtype)

    q_map_goal = s.inv_kin(target_wektor, handtype)
    if len(q_map_goal) == 0:
        rospy.logerr("No path to target...")
        exitError(1120)
    s.plan_and_execute(q_map_goal, handtype, 0.05)  # dal leewej 01 dla prawej m niej

    rospy.loginfo("Dropping")
    s.change_gripper_state(handtype) # open

    rospy.loginfo("Going up")
    # mozna peztestowac
    # s.cart_mode()
    # s.cart_move(box,handtype, up=True)
    # s.joint_mode()
    
    twe, _ = s.transformations_object("box1", z=0.2)
    q_map_goal = s.inv_kin(twe, handtype)
    if len(q_map_goal) == 0:
        rospy.logerr("No path to target...")
        exitError(1120)

    s.plan_and_execute(q_map_goal, handtype, 0.02)
    
    s.change_gripper_state(handtype) # close

    # i tu homing mozna
    rospy.loginfo("Finish")

    # homing
    s.homing()


if __name__ == "__main__":
    main()
