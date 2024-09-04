import SofaRuntime
import Sofa


class Finger:
    def __init__(self, node, name, translation=[0, 0, 0]) -> None:
        print("Init finger: " + name + "...")
        self.node = node