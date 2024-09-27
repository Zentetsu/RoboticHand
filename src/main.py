from structure import Arm, Hand, BasicStructure
from tools import *

from IRONbark import Module
import threading
import time
import os


def checkSharedMemory(arm: Arm, hand: Hand) -> None:
    SOFA_Module = Module(file=os.environ['PHDPATH']  + "/RoboticHand/data/SOFA_Module.json")

    while SOFA_Module.getLSAvailability(listener=True)[1][0]:
        time.sleep(1)

        try:
            target_wrist = SOFA_Module["target"]["wrist"]
            # print(target_wrist)
            target_thumb = SOFA_Module["target"]["thumb"]
            # print(target_thumb)
            target_index = SOFA_Module["target"]["index"]
            # print(target_index)

            target_forearm = getForearmFromHand(target_wrist, -48)
            arm.updateTargetPosition(0, target_forearm)

            g_target_th_qu = coEulerToQuat(target_wrist, target_thumb)
            hand.updateTargetPosition(0, g_target_th_qu)

            g_target_in_qu = coEulerToQuat(target_wrist, target_index)
            hand.updateTargetPosition(1, g_target_in_qu)

            c_angles = arm.getPosition()

            for i in range(0, len(c_angles)):
                SOFA_Module["sofa"]["a" + str(i)] = c_angles[i]
        except:
            pass

    SOFA_Module.stopModule()

def createScene(root) -> None:
    path = os.environ['PHDPATH'] + "/RoboticHand/model/"

    addPlugins(root)
    initScene(root, path + "Others/", ground=True)

    sim = root.addChild("Simulation")

    cube = BasicStructure(
        sim,
        path + "Others/",
        "Cube",
        positions=[[-125, 350, 50, 0, 0, 0, 1]],
        visu_info=[(0, "Cube", "Cube", [0, 0, 0], [0, 0, 0], True)]
    )
    cube.createStructure(solver="SparseLDLSolver", collision=True, constraint=True, type_c=1)
    cube.createVisualization()

    target_wrist = [0, 250, 205, 0, 0, 0]
    target_forearm = getForearmFromHand(target_wrist, -48)

    arm = Arm(sim, path + "Arm/", "Arm")
    arm.createStructure(solver="SparseLDLSolver", constraint=True)
    arm.createArticulation()
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    arm.inverseControl([
        target_forearm,
    ])

    # matrix = matrixOriginToWrist(target_wrist[:3], target_wrist[3:])
    # invert_matrix = invertMatrix(matrix)

    # g_target_th_eu = [-50.0, 400.0, 230.0, 170, 25, -75]
    # g_target_th_qu = [*g_target_th_eu[:3], *degToQuat(g_target_th_eu[3:])]

    # w_target_th_eu = invert_matrix @ np.array([*g_target_th_eu[:3], 1])
    # w_target_th_eu = [*w_target_th_eu[:3], *g_target_th_eu[3:]]
    # print(w_target_th_eu)

    w_target_th_eu = [-50.0, 120.0, 25.0, 170, 25, -75]
    g_target_th_qu = coEulerToQuat(target_wrist, w_target_th_eu)

    w_target_in_eu = [-80.0, 170.0, 15.0, 180, 90, 0]
    g_target_in_qu = coEulerToQuat(target_wrist, w_target_in_eu)

    hand = Hand(sim, path + "Hand/files/", "Hand")
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

    thread = threading.Thread(target=checkSharedMemory, args=(arm, hand))
    thread.start()

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
