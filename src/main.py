from structure import Arm, Hand, BasicStructure
from tools import *

import os


def createScene(root) -> None:
    path = os.environ['PHDPATH'] + "/RoboticHand/model/"

    addPlugins(root)
    initScene(root, path + "Others/", ground=False)

    # sphere = BasicStructure(root, path + "Others/", "Cube", positions=[[0, 0, 0, 0, 0, 0, 1]])
    # sphere.createStructure(solver="CGLinearSolver")
    # # sphere.createArticulation()
    # sphere.createRigid()
    # sphere.createVisualization(collision=True)
    # # sphere.createArticulationCenter()

    sim = root.addChild("Simulation")

    arm = Arm(sim, path + "Arm/", "Arm")
    arm.createStructure(solver="SparseLDLSolver", constraint=True)
    arm.createArticulation()
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    arm.inverseControl([0, 250, 50, 0, 0, 0, 1])

    hand = Hand(sim, path + "Hand/files/", "Hand")
    hand.createStructure(solver="SparseLDLSolver", constraint=True)
    hand.createArticulation()
    hand.createRigid()
    hand.createVisualization()
    hand.createArticulationCenter()
    hand.attachToRobot()

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)