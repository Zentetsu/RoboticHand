from structure import Arm, Hand, BasicStructure
from tools import *

import os


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
    cube.createStructure(solver="SparseLDLSolver", constraint=True)
    cube.createRigid(collision=True)
    cube.createVisualization()

    r_target_arm = degToQuat([-30, 0, 0])
    arm = Arm(sim, path + "Arm/", "Arm")
    arm.createStructure(solver="SparseLDLSolver", constraint=True)
    arm.createArticulation()
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    arm.inverseControl([
        [0, 175, 150, r_target_arm[0], r_target_arm[1], r_target_arm[2], r_target_arm[3]],
    ])

    r_target_in = degToQuat([230, 120, 70])
    hand = Hand(sim, path + "Hand/files/", "Hand")
    hand.createStructure(solver="SparseLDLSolver", constraint=True)
    hand.createArticulation()
    hand.createRigid()
    hand.createVisualization()
    hand.createArticulationCenter()
    hand.attachToRobot()
    hand.inverseControl([
        None, #[-100, 350, 350, 0, 0, 0, 1],
        [-75, 370, 75, r_target_in[0], r_target_in[1], r_target_in[2], r_target_in[3]],
    ])

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
