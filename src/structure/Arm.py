import SofaRuntime
import Sofa

from scipy.spatial.transform import Rotation as R
from structure import ComplexStructure
import numpy as np
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

    def getPosition(self):
        angles = [0] * len(self.mo.position.value)
        d_angle = np.array([0, 0, 0])
        angles_ = []

        for i in range(1, len(angles)):
            angles[i] = list(R.from_quat(self.mo.position.value[i][3:]).as_euler('xyz', degrees=True) - d_angle)
            d_angle = d_angle + np.array(angles[i])

            if self.articulation_info[i-1][5] == 1:
                angles_.append(round(np.dot(np.array(angles[i]), np.array(self.articulation_info[i-1][6])), 2))

        return angles_