from structure import Arm, Hand, Finger, Object, BasicStructure
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
    arm.inverseControl([150, 150, 150, 0, 0, 0, 1])

    # hand = Hand(sim, path)
    # hand.inverseControl(thumb=[-120, 200, 20, 0, 0, 0, 1], index=[-60, 200, 400, 0, 0, 0, 1])

    # Cube = Object(root, "Cube", path + "Others/Cube.obj", position=[-60, 250, 192, 0, 0, 0, 1])

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
