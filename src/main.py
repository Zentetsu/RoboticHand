from structure import Hand, Finger, Object
from tools import *

import os


def createScene(root) -> None:
    path = os.environ['PHDPATH'] + "/RoboticHand/model/"

    addPlugins(root)
    initScene(root, path)

    sim = root.addChild("Simulation")

    hand = Hand(sim, path)
    hand.inverseControl(thumb=[-120, 200, 20, 0, 0, 0, 1], index=[-60, 200, 400, 0, 0, 0, 1])

    Cube = Object(root, "Cube", path + "Cube.obj", position=[-60, 250, 192, 0, 0, 0, 1])

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
