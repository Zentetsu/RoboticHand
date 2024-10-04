from structure import Arm, Hand, BasicStructure
from tools import *

from IRONbark import Module
import threading
import time
import os


def checkSharedMemory(arm: Arm, hand: Hand) -> None:
    SOFA_Module = Module(file=os.environ['PHDPATH']  + "/RoboticHand/data/SOFA_Module.json")

    while SOFA_Module.getLSAvailability(listener=True)[1][0]:
        time.sleep(0.1)

        try:
            if arm is not None:
                target_wrist = SOFA_Module["target"]["wrist"]
                print(target_wrist)
                target_forearm = getForearmFromHand(target_wrist, -48)
                arm.updateTargetPosition(0, target_forearm)

                c_angles = arm.getPosition()

                SOFA_Module["sofa"]["angles"] = c_angles

            # if hand is not None:
            #     target_thumb = SOFA_Module["target"]["thumb"]
            #     # print(target_thumb)
            #     g_target_th_qu = coEulerToQuat(target_wrist, target_thumb)
            #     hand.updateTargetPosition(0, g_target_th_qu)

            #     target_index = SOFA_Module["target"]["index"]
            #     print(target_index)
            #     g_target_in_qu = coEulerToQuat(target_wrist, target_index)
            #     hand.updateTargetPosition(1, g_target_in_qu)

        except:
            pass

    SOFA_Module.stopModule(name="sofa")

def createCube(node, path) -> None:
    cube = BasicStructure(
        node,
        path + "Others/",
        "Cube",
        positions=[[-125, 350, 50, 0, 0, 0, 1]],
        visu_info=[(0, "Cube", "Cube", [0, 0, 0], [0, 0, 0], True)]
    )
    cube.createStructure(solver="SparseLDLSolver", collision=True, constraint=True, type_c=1)
    cube.createVisualization()

def createArmTurtle(node, path, target_wrist):
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
    arm.createArticulation()
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    arm.inverseControl([
        target_forearm,
    ])

    return arm

def createArm750(node, path, target_wrist):
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
        (5, "Joint_5", "R5", [0,     79,    0], [0, 0, 90], False),
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
    arm.createArticulation()
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    arm.inverseControl([
        target_forearm,
    ])

    return arm

def createHand(node, path, target_wrist, target_hand):
    g_target_th_qu = coEulerToQuat(target_wrist, target_hand[0])
    g_target_in_qu = coEulerToQuat(target_wrist, target_hand[1])

    hand = Hand(node, path + "Hand/files/", "Hand")
    hand.createStructure(solver="SparseLDLSolver", constraint=True)
    hand.createArticulation()
    hand.createRigid()
    hand.createVisualization()
    hand.createArticulationCenter()
    hand.attachToRobot()
    hand.inverseControl([
        g_target_th_qu,
        g_target_in_qu,
    ])

    return hand

def createScene(root) -> None:
    path = os.environ['PHDPATH'] + "/RoboticHand/model/"

    addPlugins(root)
    initScene(root, path + "Others/", ground=True)

    sim = root.addChild("Simulation")
    arm, hand = None, None

    createCube(sim, path)

    target_wrist = [0, 500, 300, 0, 0, 0]
    arm = createArm750(sim, path, target_wrist)

    # matrix = matrixOriginToWrist(target_wrist[:3], target_wrist[3:])
    # invert_matrix = invertMatrix(matrix)

    # g_target_th_eu = [-50.0, 400.0, 230.0, 170, 25, -75]
    # g_target_th_qu = [*g_target_th_eu[:3], *degToQuat(g_target_th_eu[3:])]

    # w_target_th_eu = invert_matrix @ np.array([*g_target_th_eu[:3], 1])
    # w_target_th_eu = [*w_target_th_eu[:3], *g_target_th_eu[3:]]
    # print(w_target_th_eu)

    # w_target_th_eu = [-50.0, 120.0, 25.0, 170, 25, -75]
    # w_target_in_eu = [-80.0, 170.0, 15.0, 180, 90, 0]
    # hand = createHand(sim, path, target_wrist, [
    #     w_target_th_eu,
    #     w_target_in_eu
    # ])

    # thread = threading.Thread(target=checkSharedMemory, args=(arm, hand))
    # thread.start()

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
