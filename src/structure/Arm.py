import SofaRuntime
import Sofa

from structure import BasicStructure, Finger
import math


class Arm(BasicStructure):
    def __init__(self, node, path, name="BasicStructure", translation=[0, 0, 0], positions=None, init_angles=None, visu_info=None):
        positions = [
            [0,         0,      0, 0,0,0,1],
            [0,         0,      0, 0,0,0,1],
            [0,         0,      0, 0,0,0,1],
            [0,         0,      0, 0,0,0,1],
            [0,         0,      0, 0,0,0,1],
        ]

        init_angles = [
            math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(0),
        ]

        visu_info = [
            ("Joint_0", "XM430-W350-R", [0,    0, 39], [0,   0,  0]),
            ("Joint_1", "XM430-W350-R", [39/2, 0,  0], [90,  0, 90]),
            ("Joint_2", "XM430-W350-R", [39/2, 0,  0], [90, 90, 90]),
            ("Joint_3", "XM430-W350-R", [39/2, 0,  0], [90, 90, 90]),
            ("Joint_4", "XM430-W350-R", [0,    0,  0], [-90, 0,  0]),
        ]

        self.joint_actuator = [
            (0, math.radians(-90), math.radians(90)),
            (1, math.radians(-90), math.radians(90)),
            (2, math.radians(-90), math.radians(90)),
        ]

        super().__init__(node, path, name, translation, positions, init_angles, visu_info)

        self.ext = ".stl"

    def createArticulation(self, joint_actuator=False) -> None:
        super().createArticulation(joint_actuator)
        print("Update " + self.name + " articulation...")

        for joint in self.joint_actuator:
            self.articulation.addObject("JointActuator", name="joint_" + str(joint[0]), index=joint[0], maxAngleVariation=0.001, minAngle=joint[1], maxAngle=joint[2])

    def createArticulationCenter(self) -> None:
        print("Create " + self.name + " articulation center...")

        self.centers = self.articulation.addChild("ArticulationCenters")

        BasicStructure.addCenter(self.centers, "Joint_0", 0, 1, [0, 0, 77], [0,   0,    0], 0, 0, 1, [0, 0, 1], 0)
        BasicStructure.addCenter(self.centers, "Joint_1", 1, 2, [0, 0,  0], [0, -24, -128], 0, 0, 1, [1, 0, 0], 1)
        BasicStructure.addCenter(self.centers, "Joint_2", 2, 3, [0, 0,  0], [0, -128,   0], 0, 0, 1, [1, 0, 0], 2)
        BasicStructure.addCenter(self.centers, "Joint_3", 3, 4, [0, 0,  0], [0, -124,   0], 0, 0, 1, [1, 0, 0], 3)

        self.indice = 4
