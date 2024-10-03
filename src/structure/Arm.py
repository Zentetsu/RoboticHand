import SofaRuntime
import Sofa

from structure import ComplexStructure
import math


class Arm(ComplexStructure):
    def __init__(self, node, path, name="Arm", translation=[0, 0, 0], rotation=[0, 0, 0], positions=None, init_angles=None, visu_info=None, joint_actuator=None, articulation_info=None):
        self.joint_actuator = joint_actuator
        self.articulation_info = articulation_info

        super().__init__(node, path, name, translation, rotation, positions, init_angles, visu_info)

        self.ext = ".stl"

    def createArticulationCenter(self) -> None:
        print("Create " + self.name + " articulation center...")

        self.centers = self.articulation.addChild("ArticulationCenters")

        for i, articullation in enumerate(self.articulation_info):
            ComplexStructure.addCenter(
                self.centers,
                articullation[0],
                articullation[1],
                articullation[2],
                articullation[3],
                articullation[4],
                0,
                0,
                articullation[5],
                articullation[6],
                articullation[7])

        self.indice = [len(self.articulation_info)]
