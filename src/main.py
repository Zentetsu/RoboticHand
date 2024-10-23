from structure import Arm, Hand, BasicStructure
from tools import *

from IRONbark import Module
import threading
import time
import sys
import os

import roboticstoolbox as rtb
from spatialmath import SO3, SE3

class CustomController(Sofa.Core.Controller):
    def __init__(self, arm: Arm, hand: Hand, cube: BasicStructure, generic_solver: False):
        super().__init__()
        self.arm = arm
        self.hand = hand
        self.cube = cube
        self.generic = generic_solver

        self.SOFA_Module = Module(file=os.environ['PHDPATH']  + "/RoboticHand/data/SOFA_" + ('g' if generic_solver else 'i') + "_Module.json")

    def onAnimateBeginEvent(self, event):
        on = self.SOFA_Module.getLSAvailability(listener=True)[1][0] if not self.generic else self.SOFA_Module.getLSAvailability(listener=True)[1][0] or self.SOFA_Module.getLSAvailability(listener=True)[1][1]

        if on and not self.generic:
            target_wrist = self.SOFA_Module["target"]["wrist_p"]
            if self.arm is not None:
                target_forearm = getForearmFromHand(target_wrist, 0)
                # print(target_wrist)
                self.arm.updateTargetPosition(0, target_forearm)

                arm_angles = self.arm.getPosition()
                self.SOFA_Module["sofa_i"]["arm_ang"] = arm_angles
            if self.hand is not None:
                # TODO: Need to be tested
                target_thumb = self.SOFA_Module["target"]["thumb_p"]
                g_target_th_qu = coEulerToQuat(target_wrist, target_thumb)
                self.hand.updateTargetPosition(0, g_target_th_qu)

                target_index = self.SOFA_Module["target"]["index_p"]
                g_target_in_qu = coEulerToQuat(target_wrist, target_index)
                self.hand.updateTargetPosition(1, g_target_in_qu)
        if on and self.generic:
            if self.arm is not None:
                target_wrist_angle = np.array(self.SOFA_Module["target"]["wrist_a"])
                print(target_wrist_angle)

                self.arm.updateAngle(list(target_wrist_angle))

            if self.hand is not None:
                target_index_angle = np.array(self.SOFA_Module["target"]["index_a"])
                target_thumb_angle = np.array(self.SOFA_Module["target"]["thumb_a"])

                self.hand.updateAngle(list(target_index_angle), list(target_thumb_angle))

def checkSharedMemory(arm: Arm, hand: Hand, generic_solver: False) -> None:
    SOFA_Module = Module(file=os.environ['PHDPATH']  + "/RoboticHand/data/SOFA_" + ('g' if generic_solver else 'i') + "_Module.json")

    while SOFA_Module.getLSAvailability(listener=True)[1][0] if not generic_solver else SOFA_Module.getLSAvailability(listener=True)[1][0] or SOFA_Module.getLSAvailability(listener=True)[1][1]:
        time.sleep(0.01)
        try:
            if not generic_solver:
                if arm is not None:
                    target_wrist = SOFA_Module["target"]["wrist"]
                    # print(target_wrist)
                    target_forearm = getForearmFromHand(target_wrist, -48)
                    arm.updateTargetPosition(0, target_forearm)

                    arm_angles = arm.getPosition()

                    SOFA_Module["sofa_i"]["arm_ang"] = arm_angles
                    # print(target_wrist, arm_angles)
                if hand is not None:
                    # target_thumb = SOFA_Module["target"]["thumb"]
                    # g_target_th_qu = coEulerToQuat(target_wrist, target_thumb)
                    # # print(g_target_th_qu)
                    # hand.updateTargetPosition(0, g_target_th_qu)

                    target_index = SOFA_Module["target"]["index"]
                    # print(target_index)
                    g_target_in_qu = coEulerToQuat(target_wrist, target_index)
                    hand.updateTargetPosition(1, g_target_in_qu)

                    hand_angles = hand.getPosition()
                    SOFA_Module["sofa_i"]["hand_ang"] = hand_angles
                    # print("ee", hand_angles)
            else:
                if arm is not None:
                    if SOFA_Module.getLSAvailability(listener=True)[1][0]:
                        target_arm_angles = SOFA_Module["sofa_i"]["arm_ang"]
                    else:
                        target_arm_angles = SOFA_Module["target"]["angles"]
                    # print("g", target_arm_angles, SOFA_Module.getLSAvailability(listener=True)[1][0])
                    arm.updateAngle(target_arm_angles)

                if hand is not None and SOFA_Module.getLSAvailability(listener=True)[1][0]:
                    target_hand_angles = SOFA_Module["sofa_i"]["hand_ang"]
                    hand.updateAngle(target_hand_angles)
        except:
            break

    SOFA_Module.stopModule()

def createObj(node, path, name, position):
    cube = BasicStructure(
        node,
        path + "Others/",
        name,
        positions=[position],
        visu_info=[(0, name, name, position[:3], position[3:-1], True)]
    )
    cube.createStructure(solver="CGLinearSolver", collision=True, constraint=True, type_c=1, deformable=False)
    cube.createVisualization()

    return cube

def createArmTurtle(node, path, target_wrist, generic_solver):
    target_forearm = getForearmFromHand(target_wrist, -48)

    positions = [
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1], # Link 01
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1], # Beam
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1], # Beam
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1], # Link 34
        [0, 0, 0, 0, 0, 0, 1],
    ]

    init_angles = [
        math.radians(0),
        math.radians(0), # Link 01
        math.radians(0),
        math.radians(0), # Beam
        math.radians(0),
        math.radians(0), # Beam
        math.radians(0),
        math.radians(0), # Link 34
        math.radians(0),
    ]

    visu_info = [
        (0, "Joint_0", "XM430-W350-R", [       0,     0,    39], [  0,  0,  0], False),
        (1, "Link_01", "Link_01",      [       0,     0,     0], [  0,  0,  0], False),
        (2, "Joint_1", "XM430-W350-R", [39/2+2.5,     0,     0], [ 90,  0, 90], False),
        (3, "Link_12", "Upper_Arm",    [       0,     0, 22.65], [  0,  0,  0], False),
        (4, "Joint_2", "XM430-W350-R", [    39/2,     0,     0], [ 90, 90, 90], False),
        (5, "Link_23", "Lower_Arm",    [    -2.5, 22.65,     0], [  0,  0,  0], False),
        (6, "Joint_3", "XM430-W350-R", [    39/2,     0,     0], [ 90, 90, 90], False),
        (7, "Link_34", "Link_34",      [    -2.5,     0,     0], [  0,  0,  0], False),
        (8, "Joint_4", "XM430-W350-R", [       0,     0,     0], [-90,  0,  0], False),
    ]

    joint_actuator = [
        (0, math.radians(-90), math.radians(90)), # Joint 0
        (2, math.radians(-90), math.radians(90)), # Joint 1
        (4, math.radians(-90), math.radians(90)), # Joint 2
        (6, math.radians(-90), math.radians(90)), # Joint 3
        (8, math.radians(-90), math.radians(90)), # Joint 4
    ]

    articulation_info = [
        ("Joint_0", 0, 1, [0,  0, 39], [0,    0,    0], 1, [0, 0, 1], 0),
        ("Link_01", 1, 2, [0,  0, 38], [0,    0,    0], 1, [1, 0, 0], 1),
        ("Joint_1", 2, 3, [0,  0,  0], [0,    0,    0], 1, [1, 0, 0], 2),
        ("Link_12", 3, 4, [0,  0,  0], [0,  -24, -128], 0, [0, 0, 0], 3),
        ("Joint_2", 4, 5, [0,  0,  0], [0,    0,    0], 1, [1, 0, 0], 4),
        ("Link_23", 5, 6, [0,  0,  0], [0, -128,    0], 0, [0, 0, 0], 5),
        ("Joint_3", 6, 7, [0,  0,  0], [0,  -36,    0], 1, [1, 0, 0], 6),
        ("Link_34", 7, 8, [0,  0,  0], [0,  -42,    0], 0, [0, 0, 0], 7),
        ("Joint_4", 8, 9, [0,  0,  0], [0,    0,    0], 1, [0, 1, 0], 8),
    ]

    arm = Arm(node, path + "ArmTurtle/", "Arm", positions=positions, init_angles=init_angles, visu_info=visu_info, joint_actuator=joint_actuator, articulation_info=articulation_info)
    arm.createStructure(solver="SparseLDLSolver", constraint=True)
    arm.createArticulation(joint_limit=generic_solver)
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()

    if generic_solver:
        arm.inverseControl([
            target_forearm,
        ])

    return arm

def createArm750(node, path, target_wrist,generic_solver):
    target_forearm = getForearmFromHand(target_wrist, -48)

    positions = [
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
    ]

    init_angles = [
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
    ]

    visu_info = [
        (0, "Joint_0", "R0", [0,      0,    0], [0, 0, 90], False),
        (1, "Joint_1", "R1", [0,      0, 66.9], [0, 0, 90], False),
        (2, "Joint_2", "R2", [0,     57,  303], [0, 0, 90], False),
        (3, "Joint_3", "R3", [0,  82.53,    0], [0, 0, 90], False),
        (4, "Joint_4", "R4", [0, 245.38,    0], [0, 0, 90], False),
        (5, "Joint_5", "R5", [0,     79,    0], [0, 0, 90], True),
    ]

    joint_actuator = [
        (0, math.radians(-90), math.radians(90)), # Joint 0
        (1, math.radians(-90), math.radians(90)), # Joint 1
        (2, math.radians(-90), math.radians(90)), # Joint 2
        (3, math.radians(-90), math.radians(90)), # Joint 3
        (4, math.radians(-90), math.radians(90)), # Joint 4
        (5, math.radians(-90), math.radians(90)), # Joint 5
    ]

    articulation_info = [
        ("R0", 0, 1, [0,      0,  107], [0, 0, 0], 1, [0, 0, 1], 0),
        ("R1", 1, 2, [0,      0, 66.9], [0, 0, 0], 1, [1, 0, 0], 1),
        ("R2", 2, 3, [0,     57,  303], [0, 0, 0], 1, [1, 0, 0], 2),
        ("R3", 3, 4, [0,  82.53,    0], [0, 0, 0], 1, [0, 1, 0], 3),
        ("R4", 4, 5, [0, 245.38,    0], [0, 0, 0], 1, [1, 0, 0], 4),
        ("R5", 5, 6, [0,     79,    0], [0, 0, 0], 1, [0, 1, 0], 5),
    ]

    arm = Arm(node, path + "myArm750/", "Arm", positions=positions, init_angles=init_angles, visu_info=visu_info, joint_actuator=joint_actuator, articulation_info=articulation_info)
    arm.createStructure(solver="SparseLDLSolver", constraint=True)
    arm.createArticulation(joint_limit=generic_solver)
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    if generic_solver:
        arm.inverseControl([
            target_forearm,
        ])

    return arm

def createHand(node, path, target_wrist, target_hand, generic_solver, arm):
    g_target_th_qu = None if target_hand[0] is None else coEulerToQuat(target_wrist, target_hand[0])
    g_target_in_qu = None if target_hand[1] is None else coEulerToQuat(target_wrist, target_hand[1]) if len(target_hand[1]) == 6 else target_hand[1]

    positions = [
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 1],
    ]

    init_angles = [
            math.radians(0), # Wrist link
            math.radians(45), # Meta 2
            math.radians(0.14), # Meta 1 1 #FIX#
            math.radians(-3.45), # Meta 1 2 #FIX#
            math.radians(0), # Meta 1 3

            math.radians(-1.1), # Thumb 1 1 #FIX#
            math.radians(-4.775), # Thumb 1 2 #FIX#
            math.radians(-50), # Thumb 1 3
            math.radians(0), # Thumb 2 1 #FIX#
            math.radians(-10), # Thumb 2 2 #FIX#
            math.radians(-20), # Thumb 2 3
            math.radians(-30), # Thumb 3
            math.radians(70), # Thumb 4
            math.radians(45), # Thumb 5
            math.radians(0), # Thumb 6 fake

            math.radians(-0.165), # Index 1 1 #FIX#
            math.radians(-2.44), # Index 1 2 #FIX#
            math.radians(0), # Index 1 3
            math.radians(10), # index 2
            math.radians(-90), # index 3
            math.radians(-30), # index 4
            math.radians(0), # index 5 fake
    ]

    visu_info = [
        # Wrist
        (0,       "Wrist",      "wrist", [0, 0, 48], [0, 0, 0], False),
        (1,  "Wrist_Link", "wrist_link", [0, 0,  0], [0, 0, 0], False),
        (2,      "Meta_2", "metacarp_2", [0, 0,  0], [0, 0, 0], False),
        (3,    "Meta_1_1",         None, [0, 0,  0], [0, 0, 0], False),
        (4,    "Meta_1_2",         None, [0, 0,  0], [0, 0, 0], False),
        (5,    "Meta_1_3", "metacarp_1", [0, 0,  0], [0, 0, 0], False),

        # Thumb
        (6,  "Thumb_p1_1",         None, [0, 0,  0], [0, 0, 0], False),
        (7,  "Thumb_p1_2",         None, [0, 0,  0], [0, 0, 0], False),
        (8,  "Thumb_p1_3",   "thumb_p1", [0, 0,  0], [0, 0, 0], False),
        (9,  "Thumb_p2_1",         None, [0, 0,  0], [0, 0, 0], False),
        (10, "Thumb_p2_2",         None, [0, 0,  0], [0, 0, 0], False),
        (11, "Thumb_p2_3",   "thumb_p2", [0, 0,  0], [0, 0, 0], False),
        (12,   "Thumb_p3",   "thumb_p3", [0, 0,  0], [0, 0, 0], False),
        (13,   "Thumb_p4",   "thumb_p4", [0, 0,  0], [0, 0, 0], False),
        (14,   "Thumb_p5",   "thumb_p5", [0, 0,  0], [0, 0, 0], True),

        # Index
        (16,  "Index_p1_1",         None, [0, 0,  0], [0, 0, 0], False),
        (17,  "Index_p1_2",         None, [0, 0,  0], [0, 0, 0], False),
        (18,  "Index_p1_3",   "index_p1", [0, 0,  0], [0, 0, 0], False),
        (19,    "Index_p2",   "index_p2", [0, 0,  0], [0, 0, 0], False),
        (20,    "Index_p3",   "index_p3", [0, 0,  0], [0, 0, 0], False),
        (21,    "Index_p4",   "index_p4", [0, 0,  0], [0, 0, 0], True),
    ]

    joint_actuator = [
        # Wrist
        (0, math.radians(-10), math.radians(10)),
        (1, math.radians(-75), math.radians(75)),
        # (2, math.radians(0), math.radians(0)),
        # (3, math.radians(-3.45), math.radians(-3.45)),
        # (4, math.radians(-5), math.radians(5)),

        # Thumb
        # (5, math.radians(0), math.radians(0)), # thumb 1
        # (6, math.radians(-4.7746), math.radians(-4.7746)),
        (7, math.radians(-50), math.radians(10)),
        # (8, math.radians(0), math.radians(10)), # thumb 2
        # (9, math.radians(-10), math.radians(-10)),
        (10, math.radians(-50), math.radians(50)),
        (11, math.radians(-40), math.radians(15)), # Thumb 3
        (12, math.radians(0), math.radians(90)), # Thumb 4
        (13, math.radians(-50), math.radians(70)), # Thumb 5

        # Index
        # (15,  math.radians(-0.165), math.radians(0.165)), # index 1
        # (16,  math.radians(-2.44), math.radians(-2.44)),
        (17,  math.radians(-15), math.radians(15)),
        (18, math.radians(-90), math.radians(10)), # index 2
        (19, math.radians(-120), math.radians(0)), # index 3
        (20, math.radians(-45), math.radians(0)), # index 4
    ]

    articulation_info = [
        ("Wrist",       0, 1,    [        0,         0,       48], [0, 0, 0], 1, [0, 1, 0], 0),
        ("Meta2_c",     1, 2,    [      1.1,     -0.25,  32.6447], [0, 0, 0], 1, [1, 0, 0], 1),
        ("Meta1_c_1",   2, 3,    [  -9.6137,  -1.65902,  48.9203], [0, 0, 0], 1, [1, 0, 0], 2),
        ("Meta1_c_2",   3, 4,    [        0,         0,        0], [0, 0, 0], 1, [0, 1, 0], 3),
        ("Meta1_c_3",   4, 5,    [        0,         0,        0], [0, 0, 0], 1, [0, 0, 1], 4),

        ("Thumbp1_c_1",  5, 6,   [ -12.2179,       5.2, -47.3596], [0, 0, 0], 1, [0, 1, 0], 5),
        ("Thumbp1_c_2",  6, 7,   [        0,         0,        0], [0, 0, 0], 1, [0, 0, 1], 6),
        ("Thumbp1_c_3",  7, 8,   [        0,         0,        0], [0, 0, 0], 1, [1, 0, 0], 7),
        ("Thumbp2_c_1",  8, 9,   [ -13.1755 ,  3.49373,  0.86368], [0, 0, 0], 1, [1, 0, 0], 8),
        ("Thumbp2_c_2",  9, 10,  [        0,         0,        0], [0, 0, 0], 1, [0, 1, 0], 9),
        ("Thumbp2_c_3",  10, 11, [        0,         0,        0], [0, 0, 0], 1, [0, 0, 1], 10),
        ("Thumbp3_c",    11, 12, [  -3.4515,         0,  20.4197], [0, 0, 0], 1, [0, 1, 0], 11),
        ("Thumbp4_c",    12, 13, [     -1.2,         0,       40], [0, 0, 0], 1, [0, 1, 0], 12),
        ("Thumbp5_c",    13, 14, [     -7.2,         0,       36], [0, 0, 0], 1, [0, 1, 0], 13),
        ("Thumbp6_c",    14, 15, [  5.70894,  0.003393,  33.9805], [0, 0, 0], 0, [0, 0, 0], 14),

        ("Indexp1_c_1",   4, 16, [ -13.8285,     -1.45,  30.1857], [0, 0, 0], 1, [1, 0, 0], 15),
        ("Indexp1_c_2",  16, 17, [        0,         0,        0], [0, 0, 0], 1, [0, 0, 1], 16),
        ("Indexp1_c_3",  17, 18, [        0,         0,        0], [0, 0, 0], 1, [0, 1, 0], 17),
        ("Indexp2_c",    18, 19, [-0.099998,      -1.6,       14], [0, 0, 0], 1, [1, 0, 0], 18),
        ("Indexp3_c",    19, 20, [        0,   1.99797,       48], [0, 0, 0], 1, [1, 0, 0], 19),
        ("Indexp4_c",    20, 21, [        0,  0.899992,  27.9039], [0, 0, 0], 1, [1, 0, 0], 20),
        ("Indexp5_c",    21, 22, [-0.036879, -0.910405,  22.2986], [0, 0, 0], 0, [0, 0, 0], 21),

    ]

    hand = Hand(node, path + "Hand/files/", "Hand", positions=positions, init_angles=init_angles,visu_info=visu_info, joint_actuator=joint_actuator, articulation_info=articulation_info)
    hand.createStructure(solver="SparseLDLSolver", constraint=True)
    hand.createArticulation(joint_limit=not generic_solver)
    hand.createRigid()
    hand.createVisualization()
    hand.createArticulationCenter(indice=[15, 22])

    if arm:
        hand.attachToRobot()

    if not generic_solver:
        hand.inverseControl([
            g_target_th_qu,
            g_target_in_qu,
        ])

    return hand

def createScene(root) -> None:
    path = os.environ['PHDPATH'] + "/RoboticHand/model/"

    _generic_solver = True if "generic" in sys.argv else False
    _cube = True if "cube" in sys.argv else False
    _sphere = True if "sphere" in sys.argv else False
    _arm750 = True if "arm750" in sys.argv else False
    _hand = True if "hand" in sys.argv else False
    _shared_mem = True if "sm" in sys.argv else False

    addPlugins(root)
    initScene(root, path + "Others/", ground=True, generic_solver=_generic_solver)

    sim = root.addChild("Simulation")
    arm, hand, cube = None, None, None

    # # cube_pos = [0, 57+327.91+79-15, 173.9+303+50, 0, 0, 0, 1] #top of the arm
    if _cube:
        cube_pos =[-125, 350, 27, 0, 0, 0, 1] # on the floor
        cube = createObj(sim, path, "Cube", cube_pos)

    if _sphere:
        sphere_pos =[-125, 350, 50, 0, 0, 0, 1] # on the floor
        createObj(sim, path, "Sphere", sphere_pos)

    target_wrist = [0, 57+327.91+79+48, 173.9+303, 0, 0, 0] #Origin pos for top cube #[0, 0, 48, 90, 0, -90] #
    if _arm750:
        arm = createArm750(sim, path, target_wrist, not _generic_solver)

    # matrix = matrixOriginToWrist(target_wrist[:3], target_wrist[3:])
    # invert_matrix = invertMatrix(matrix)

    # g_target_th_eu = [7.04271402e+01, 6.68115182e+02, 5.02962263e+02, -145.34, 85.77, 9.61]
    # g_target_th_qu = [*g_target_th_eu[:3], *degToQuat(g_target_th_eu[3:])]

    # w_target_th_eu = invert_matrix @ np.array([*g_target_th_eu[:3], 1])
    # w_target_th_eu = [*w_target_th_eu[:3], *g_target_th_eu[3:]]
    # print(w_target_th_eu)

    if _hand:
        w_target_th_eu = [-3.21, 124.094, 19.73, -152.9, 13.67, -75]
        w_target_in_eu = [70.427, 156.2, 26.06, -145.34, 85.77, 9.61]#[1.840503, 223.3336, 23.694411, 0, 0, 0] #[-23.694411, -1.840503, 271.3336, 0, 0, 0]
        hand = createHand(sim, path, target_wrist, [
            w_target_th_eu,
            w_target_in_eu
        ], _generic_solver, _arm750)

    if _shared_mem:
        # thread = threading.Thread(target=checkSharedMemory, args=(arm, hand, _generic_solver))
        # thread.start()

        controller = CustomController(arm, hand, cube, _generic_solver)
        root.addObject(controller)

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
