import SofaRuntime # type: ignore
import Sofa # type: ignore

from scipy.spatial.transform import Rotation as R
from structure import ComplexStructure
import numpy as np
import math
import copy


class Hand(ComplexStructure):
    def __init__(self, node, path, name="Hand", translation=[0, 0, 0], rotation=[0, 0, 0], positions=None, init_angles=None, visu_info=None, joint_actuator=None, articulation_info=None):
        self.joint_actuator = joint_actuator
        self.articulation_info = articulation_info

        super().__init__(node, path, name, translation, rotation, positions, init_angles, visu_info)

        self.ext = ".stl"

    def createArticulationCenter(self, indice: list) -> None:
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

        self.indice = indice

    def updateAngle(self, n_angles_in, n_angles_th):
        n_a = [
            n_angles_in[0], n_angles_in[1], 0.002443461, -0.06021386, n_angles_in[2],
            -0.191986, -0.083339472, n_angles_th[0], 0, -0.174533, n_angles_th[1], n_angles_th[2], n_angles_th[3], n_angles_th[4], 0,  #n_angles_in[7], #n_angles_in[8], #n_angles_in[9], #n_angles_in[10], n_angles_in[11],
            -0.0028797933, -0.04258603, n_angles_in[3], n_angles_in[4], n_angles_in[5], n_angles_in[6], 0
        ]

        self.structure.angles = n_a

    def getPosition(self):
        rotation_quat = R.from_euler('xyz', [math.radians(0), math.radians(90), math.radians(90)]).as_quat()
        angles = [0] * len(self.mo.position.value)
        d_angle = np.array([0, 0, 0])
        angles_ = []

        # print(self.mo.position.value)
        for i in range(1, len(angles)):
            quaternion = np.array(self.mo.position.value[i][3:])
            rotated_quat = R.from_quat(rotation_quat).inv() * R.from_quat(quaternion)

            # rotated_quat = R.from_quat(quaternion)
            angles[i] = list(rotated_quat.as_euler('xyz', degrees=True) - d_angle)
            d_angle = d_angle + np.array(angles[i])

            if self.articulation_info[i-1][5] == 1:
                angles_.append(round(np.dot(np.array(angles[i]), np.array(self.articulation_info[i-1][6])), 4))
            else:
                angles_.append(0)

            # angles_.append(list(self.mo.position.value[i]))

        return angles_

    def attachToRobot(self) -> None:
        self.node.addObject('RigidMapping', input="@./Arm/Articulation/Rigid/dofs", output="@./Hand/Articulation/Rigid/dofs", index=6)

        rotation_quat = R.from_euler('xyz', [0, math.radians(90), math.radians(90)]).as_quat()
        new_pos = copy.copy(self.mo.position.value)

        for i in range(0, len(self.mo.position.value)-1):
            quaternion = np.array(new_pos[i][3:])
            rotated_quat = R.from_quat(rotation_quat) * R.from_quat(quaternion)
            new_quaternion = rotated_quat.as_quat()

            rotation_matrix = R.from_quat(rotation_quat).as_matrix()
            new_position = rotation_matrix @ np.array(new_pos[i][:3])

            result = np.concatenate((new_position, new_quaternion))
            new_pos[i] = result

        self.mo.findData("position").setData = new_pos
