from structure import Arm, Hand, BasicStructure
from tools import *

import os


def createScene(root) -> None:
    path = os.environ['PHDPATH'] + "/RoboticHand/model/"

    addPlugins(root)
    initScene(root, path + "Others/", ground=True)

    # cube = BasicStructure(root, path + "Others/", "Cube", positions=[[-200, 200, 50, 0, 0, 0, 1]], visu_info=[(0, "Cube", "Cube", [0, 0, 0], [0, 0, 0], True)])
    # cube.createStructure(solver="CGLinearSolver")
    # cube.createRigid(collision=True)
    # cube.createVisualization()

    sim = root.addChild("Simulation")

    arm = Arm(sim, path + "Arm/", "Arm")
    arm.createStructure(solver="SparseLDLSolver", constraint=True)
    arm.createArticulation()
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    arm.inverseControl([
        [0, 225, 250, 0, 0, 0, 1],
    ])

    hand = Hand(sim, path + "Hand/files/", "Hand")
    hand.createStructure(solver="SparseLDLSolver", constraint=True)
    hand.createArticulation()
    hand.createRigid()
    hand.createVisualization()
    hand.createArticulationCenter()
    hand.attachToRobot()
    hand.inverseControl([
        [-100, 350, 350, 0, 0, 0, 1],
        [-100, 400, 300, 0, 0, 0, 1],
    ])

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
