import SofaRuntime
import Sofa

from structure import BasicStructure
import math


class Hand(BasicStructure):
    def __init__(self, node, path, name="Hand", translation=[0, 0, 0], positions=None, init_angles=None, visu_info=None):
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
                math.radians(-10), # Thumb 1
                math.radians(20), # Thumb 2
                math.radians(0), # Thumb 3
                math.radians(0), # Thumb 4
                math.radians(0), # Thumb 5
                math.radians(0), # Thumb 6 fake
                math.radians(0), # Index 1
                math.radians(0), # index 2
                math.radians(0), # index 3
                math.radians(0), # index 4
                math.radians(0), # index 5 fake
        ]

        visu_info = [
            # Wrist
            (0,      "Wrist",      "wrist", [0, 0, 48], [0, 0, 0]),
            (1, "Wrist_Link", "wrist_link", [0, 0,  0], [0, 0, 0]),
            (2,     "Meta_2", "metacarp_2", [0, 0,  0], [0, 0, 0]),
            (3,     "Meta_1", "metacarp_1", [0, 0,  0], [0, 0, 0]),

            # Thumb
            (4,   "Thumb_p1",   "thumb_p1", [0, 0,  0], [0, 0, 0]),
            (5,   "Thumb_p2",   "thumb_p2", [0, 0,  0], [0, 0, 0]),
            (6,   "Thumb_p3",   "thumb_p3", [0, 0,  0], [0, 0, 0]),
            (7,   "Thumb_p4",   "thumb_p4", [0, 0,  0], [0, 0, 0]),
            (8,   "Thumb_p5",   "thumb_p5", [0, 0,  0], [0, 0, 0]),

            # Index
            (10,  "Index_p1",   "index_p1", [0, 0,  0], [0, 0, 0]),
            (11,  "Index_p2",   "index_p2", [0, 0,  0], [0, 0, 0]),
            (12,  "Index_p3",   "index_p3", [0, 0,  0], [0, 0, 0]),
            (13,  "Index_p4",   "index_p4", [0, 0,  0], [0, 0, 0]),
        ]

        self.joint_actuator = [
            # Wrist
            (0, math.radians(-90), math.radians(90)),
            (1, math.radians(-90), math.radians(90)),
            # (2, math.radians(-90),  math.radians(90)),

            # Thumb
            (3, math.radians(-90), math.radians(90)),
            # (4, math.radians(-90), math.radians(100)), # Thumb 1
            (5, math.radians(-90), math.radians(90)), # Thumb 2
            (6, math.radians(-90), math.radians(90)), # Thumb 3
            (7, math.radians(-90), math.radians(90)), # Thumb 4
            # (8, math.radians(-90), math.radians(90)), # Thumb 5

            # Index
            (9,  math.radians(-90), math.radians(90)),
            (10, math.radians(-90), math.radians(90)),
            (11, math.radians(-90), math.radians(90)),
            (12, math.radians(-90), math.radians(90)),
            (13, math.radians(-90), math.radians(90)),
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

        BasicStructure.addCenter(self.centers,     "Wrist",  0, 1,  [        0,         0,       48], [0, 0, 0], 0, 0, 1, [    0,      1,     0], 0)
        BasicStructure.addCenter(self.centers,   "Meta2_c",  1, 2,  [      1.1,     -0.25,  32.6447], [0, 0, 0], 0, 0, 1, [    1,      0,     0], 1)
        BasicStructure.addCenter(self.centers,   "Meta1_c",  2, 3,  [  -9.6137,  -1.65902,  48.9203], [0, 0, 0], 0, 0, 1, [-0.04, -0.005, 0.955], 2)

        BasicStructure.addCenter(self.centers, "Thumbp1_c",  3, 4,  [ -10.4018,    4.8352, -47.8296], [0, 0, 0], 0, 0, 1, [  0.8,   0.04,  0.16], 3)
        BasicStructure.addCenter(self.centers, "Thumbp2_c",  4, 5,  [ -13.0416,   4.00953, 0.621956], [0, 0, 0], 0, 0, 1, [-0.17,    0.0,  0.83], 4)
        BasicStructure.addCenter(self.centers, "Thumbp3_c",  5, 6,  [ -7.40277,  -0.05484,   19.341], [0, 0, 0], 0, 0, 1, [ 0.04,   0.95,  0.01], 5)
        BasicStructure.addCenter(self.centers, "Thumbp4_c",  6, 7,  [ -9.05358, -0.346076,  38.9789], [0, 0, 0], 0, 0, 1, [ 0.04,   0.95,  0.01], 6)
        BasicStructure.addCenter(self.centers, "Thumbp5_c",  7, 8,  [ -7.28673, -0.358486,  35.7636], [0, 0, 0], 0, 0, 1, [ 0.04,   0.95,  0.01], 7)
        BasicStructure.addCenter(self.centers, "Thumbp6_c",  8, 9,  [-0.087104, -0.622104,   34.546], [0, 0, 0], 0, 0, 0, [    0,      0,     0], 8)

        BasicStructure.addCenter(self.centers, "Indexp1_c",  3, 10, [ -15.0262,  -2.13672,  29.5661], [0, 0, 0], 0, 0, 1, [-0.05,   0.95,     0], 9)
        BasicStructure.addCenter(self.centers, "Indexp2_c", 10, 11, [-0.027241,  -1.60289,       14], [0, 0, 0], 0, 0, 1, [ 0.95,   0.05,     0], 10)
        BasicStructure.addCenter(self.centers, "Indexp3_c", 11, 12, [-0.090864,   1.99792,       48], [0, 0, 0], 0, 0, 1, [ 0.95,   0.05,     0], 11)
        BasicStructure.addCenter(self.centers, "Indexp4_c", 12, 13, [-0.040892,   0.89907,  27.9039], [0, 0, 0], 0, 0, 1, [ 0.95,   0.05,     0], 12)
        BasicStructure.addCenter(self.centers, "Indexp5_c", 13, 14, [ 0.004486,  0.911137,  22.2986], [0, 0, 0], 0, 0, 0, [    0,      0,     0], 13)

        self.indice = 9
