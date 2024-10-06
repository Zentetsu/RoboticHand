import SofaRuntime
import Sofa

from scipy.spatial.transform import Rotation as R
from structure import ComplexStructure
import numpy as np
import math
import copy


class Hand(ComplexStructure):
    def __init__(self, node, path, name="Hand", translation=[0, 0, 0], rotation=[0, 0, 0], positions=None, init_angles=None, visu_info=None):
        positions = [
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 1],
        ]

        init_angles = [
                math.radians(0), # Wrist link
                math.radians(0), # Meta 2
                math.radians(0), # Meta 1
                math.radians(-20), # Thumb 1
                math.radians(20), # Thumb 2
                math.radians(-20), # Thumb 3
                math.radians(40), # Thumb 4
                math.radians(40), # Thumb 5
                math.radians(0), # Thumb 6 fake
                math.radians(0), # Index 1
                math.radians(0), # index 2
                math.radians(0), # index 3
                math.radians(0), # index 4
                math.radians(0), # index 5 fake
        ]

        visu_info = [
            # Wrist
            (0,      "Wrist",      "wrist", [0, 0, 48], [0, 0, 0], False),
            (1, "Wrist_Link", "wrist_link", [0, 0,  0], [0, 0, 0], False),
            (2,     "Meta_2", "metacarp_2", [0, 0,  0], [0, 0, 0], False),
            (3,     "Meta_1", "metacarp_1", [0, 0,  0], [0, 0, 0], False),

            # Thumb
            (4,   "Thumb_p1",   "thumb_p1", [0, 0,  0], [0, 0, 0], False),
            (5,   "Thumb_p2",   "thumb_p2", [0, 0,  0], [0, 0, 0], False),
            (6,   "Thumb_p3",   "thumb_p3", [0, 0,  0], [0, 0, 0], False),
            (7,   "Thumb_p4",   "thumb_p4", [0, 0,  0], [0, 0, 0], False),
            (8,   "Thumb_p5",   "thumb_p5", [0, 0,  0], [0, 0, 0], False),

            # Index
            (10,  "Index_p1",   "index_p1", [0, 0,  0], [0, 0, 0], False),
            (11,  "Index_p2",   "index_p2", [0, 0,  0], [0, 0, 0], False),
            (12,  "Index_p3",   "index_p3", [0, 0,  0], [0, 0, 0], False),
            (13,  "Index_p4",   "index_p4", [0, 0,  0], [0, 0, 0], True),
        ]

        self.joint_actuator = [
            # Wrist
            (0, math.radians(-10), math.radians(10)),
            (1, math.radians(-90), math.radians(90)),
            # (2, math.radians(-90),  math.radians(90)),

            # Thumb
            (3, math.radians(-180), math.radians(180)),
            # (4, math.radians(-180), math.radians(100)), # Thumb 1
            (5, math.radians(-180), math.radians(10)), # Thumb 2
            (6, math.radians(-180), math.radians(180)), # Thumb 3
            (7, math.radians(-180), math.radians(180)), # Thumb 4
            (8, math.radians(-180), math.radians(180)), # Thumb 5

            # Index
            (9,  math.radians(-10), math.radians(10)),
            (10, math.radians(-90), math.radians(10)),
            (11, math.radians(-90), math.radians(0)),
            (12, math.radians(-90), math.radians(0)),
            (13, math.radians(-90), math.radians(0)),
        ]

        super().__init__(node, path, name, translation, rotation, positions, init_angles, visu_info)

        self.ext = ".stl"

    def createArticulationCenter(self) -> None:
        print("Create " + self.name + " articulation center...")

        self.centers = self.articulation.addChild("ArticulationCenters")

        ComplexStructure.addCenter(self.centers,     "Wrist",  0, 1,  [        0,         0,       48], [0, 0, 0], 0, 0, 1, [    0,      1,     0], 0)
        ComplexStructure.addCenter(self.centers,   "Meta2_c",  1, 2,  [      1.1,     -0.25,  32.6447], [0, 0, 0], 0, 0, 1, [    1,      0,     0], 1)
        ComplexStructure.addCenter(self.centers,   "Meta1_c",  2, 3,  [  -9.6137,  -1.65902,  48.9203], [0, 0, 0], 0, 0, 1, [-0.04, -0.005, 0.955], 2)

        ComplexStructure.addCenter(self.centers, "Thumbp1_c",  3, 4,  [ -10.4018,    4.8352, -47.8296], [0, 0, 0], 0, 0, 1, [  0.8,   0.04,  0.16], 3)
        ComplexStructure.addCenter(self.centers, "Thumbp2_c",  4, 5,  [ -13.0416,   4.00953, 0.621956], [0, 0, 0], 0, 0, 1, [-0.17,    0.0,  0.83], 4)
        ComplexStructure.addCenter(self.centers, "Thumbp3_c",  5, 6,  [ -7.40277,  -0.05484,   19.341], [0, 0, 0], 0, 0, 1, [ 0.04,   0.95,  0.01], 5)
        ComplexStructure.addCenter(self.centers, "Thumbp4_c",  6, 7,  [ -9.05358, -0.346076,  38.9789], [0, 0, 0], 0, 0, 1, [ 0.04,   0.95,  0.01], 6)
        ComplexStructure.addCenter(self.centers, "Thumbp5_c",  7, 8,  [ -7.28673, -0.358486,  35.7636], [0, 0, 0], 0, 0, 1, [ 0.04,   0.95,  0.01], 7)
        ComplexStructure.addCenter(self.centers, "Thumbp6_c",  8, 9,  [-0.087104, -0.622104,   34.546], [0, 0, 0], 0, 0, 0, [    0,      0,     0], 8)

        ComplexStructure.addCenter(self.centers, "Indexp1_c",  3, 10, [ -15.0262,  -2.13672,  29.5661], [0, 0, 0], 0, 0, 1, [-0.05,   0.95,     0], 9)
        ComplexStructure.addCenter(self.centers, "Indexp2_c", 10, 11, [-0.027241,  -1.60289,       14], [0, 0, 0], 0, 0, 1, [ 0.95,   0.05,     0], 10)
        ComplexStructure.addCenter(self.centers, "Indexp3_c", 11, 12, [-0.090864,   1.99792,       48], [0, 0, 0], 0, 0, 1, [ 0.95,   0.05,     0], 11)
        ComplexStructure.addCenter(self.centers, "Indexp4_c", 12, 13, [-0.040892,   0.89907,  27.9039], [0, 0, 0], 0, 0, 1, [ 0.95,   0.05,     0], 12)
        ComplexStructure.addCenter(self.centers, "Indexp5_c", 13, 14, [ 0.004486,  0.911137,  22.2986], [0, 0, 0], 0, 0, 0, [    0,      0,     0], 13)

        self.indice = [9, 14]

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
