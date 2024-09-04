import SofaRuntime
import Sofa

from structure import Finger, edit
import math


class Hand:
    def __init__(self, node, path, translation=[0, 0, 0]):
        print("Init hand...")

        self.node = node
        self.path = path + "Hand/files/"

        self.positions = [
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1], # Thumb 6 fake
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1],
                [0,         0,      0, 0,0,0,1], # Index 5 fake
            ]

        self.init_angles = [
                math.radians(0), # Wrist link
                math.radians(0), # Meta 2
                math.radians(0), # Meta 1
                math.radians(-20), # Thumb 1
                math.radians(45), # Thumb 2
                math.radians(-25), # Thumb 3
                math.radians(25), # Thumb 4
                math.radians(0), # Thumb 5
                math.radians(0), # Thumb 6 fake
                math.radians(0), # Index 1
                math.radians(0), # index 2
                math.radians(0), # index 3
                math.radians(0), # index 4
                math.radians(0), # index 5 fake
            ]

        self.createHand()
        self.createArticulation(translation)
        self.createVisualization()

    def inverseControl(self, thumb=[0, 0, 0, 0, 0, 0, 1], index=[0, 0, 0, 0, 0, 0, 1]):
        print("Define goal...")

        # self.target2 = self.node.addChild("EffectorTarget2")

        # self.target2.addObject("EulerImplicitSolver", firstOrder=True)
        # self.target2.addObject("CGLinearSolver", iterations=100, threshold=1e-2, tolerance=1e-5)
        # self.target2.addObject("MechanicalObject", name="dofs", template="Rigid3", position=thumb, showObject=1, showObjectScale=100, drawMode=1)
        # self.target2.addObject("UncoupledConstraintCorrection")

        # self.rigid.addObject("PositionEffector", name="pe2", template="Rigid3", indices=9, effectorGoal=thumb, useDirections=[1, 1, 1, 0, 0, 0])
        # print("thumb", thumb)

        self.target1 = self.node.addChild("EffectorTarget1")

        self.target1.addObject("EulerImplicitSolver", firstOrder=True)
        self.target1.addObject("CGLinearSolver", iterations=100, threshold=1e-2, tolerance=1e-5)
        self.target1.addObject("MechanicalObject", name="dofs", template="Rigid3", position=index, showObject=1, showObjectScale=100, drawMode=1)
        self.target1.addObject("UncoupledConstraintCorrection")

        self.rigid.addObject("PositionEffector", name="pe1", template="Rigid3", indices=14, effectorGoal=index, useDirections=[1, 1, 1, 0, 0, 0])

        print("index", index)


    def createHand(self) -> None:
        print("Create hand...")

        self.hand = self.node.addChild("Hand")

        self.hand.addData("angles", self.init_angles, None, "Hand angles", "", "vector<float>")
        self.hand.addObject("EulerImplicitSolver", rayleighMass=0.01)

        self.hand.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixd")
        self.hand.addObject("GenericConstraintCorrection")

    def createArticulation(self, translation) -> None:
        print("Create hand articulation...")

        self.articulation = self.hand.addChild("Articulation")

        self.articulation.addObject("MechanicalObject", name="dofs", template="Vec1", rest_position=self.hand.getData("angles").getLinkPath(), position=self.init_angles)
        self.articulation.addObject("ArticulatedHierarchyContainer")
        self.articulation.addObject("UniformMass", vertexMass=0.1)

        self.rigid = self.articulation.addChild("Rigid")

        self.rigid.addObject("MechanicalObject", name="dofs", template="Rigid3", showObject=True, showObjectScale=1, position=self.positions[0:15], translation=translation)
        self.rigid.addObject("ArticulatedSystemMapping", input1=self.articulation.dofs.getLinkPath(), output=self.rigid.dofs.getLinkPath())

        self.articulation.addObject("JointActuator", name="joint" + str(0), index=0, maxAngleVariation=0.001, minAngle=math.radians(-10), maxAngle=math.radians(10))
        self.articulation.addObject("JointActuator", name="joint" + str(1), index=1, maxAngleVariation=0.001, minAngle=math.radians(-90), maxAngle=math.radians(90))

        # self.articulation.addObject("JointActuator", name="joint" + str(9), index=9, maxAngleVariation=0.001, minAngle=math.radians(-5), maxAngle=math.radians(5))
        self.articulation.addObject("JointActuator", name="joint" + str(10), index=10, maxAngleVariation=0.001, minAngle=math.radians(-90), maxAngle=math.radians(30))
        self.articulation.addObject("JointActuator", name="joint" + str(11), index=11, maxAngleVariation=0.001, minAngle=math.radians(-110), maxAngle=math.radians(0))
        self.articulation.addObject("JointActuator", name="joint" + str(12), index=12, maxAngleVariation=0.001, minAngle=math.radians(-75), maxAngle=math.radians(0))
        self.articulation.addObject("JointActuator", name="joint" + str(13), index=13, maxAngleVariation=0.001, minAngle=math.radians(-75), maxAngle=math.radians(0))

        # self.articulation.addObject("JointActuator", name="joint" + str(3), index=3, maxAngleVariation=0.001, minAngle=math.radians(-45), maxAngle=math.radians(0))
        # # self.articulation.addObject("JointActuator", name="joint" + str(4), index=4, maxAngleVariation=0.001, minAngle=math.radians(90), maxAngle=math.radians(180))
        # self.articulation.addObject("JointActuator", name="joint" + str(5), index=5, maxAngleVariation=0.001, minAngle=math.radians(-45), maxAngle=math.radians(90))
        # self.articulation.addObject("JointActuator", name="joint" + str(6), index=6, maxAngleVariation=0.001, minAngle=math.radians(-10), maxAngle=math.radians(90))
        # self.articulation.addObject("JointActuator", name="joint" + str(7), index=7, maxAngleVariation=0.001, minAngle=math.radians(-10), maxAngle=math.radians(45))

        self.articulation.addObject('RestShapeSpringsForceField', stiffness=1e10, points=[i for i in range(0, len(self.init_angles))])

    def createVisualization(self) -> None:
        print("Create hand visualization...")

        self.visu = self.rigid.addChild("Visu")

        edit.addPart(node=self.visu, name="Wrist", index=0, filename=self.path + "wrist.stl", translation=[0, 0, 48])
        edit.addPart(node=self.visu, name="Wrist_Link", index=1, filename=self.path + "wrist_link.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Meta_2", index=2, filename=self.path + "metacarp_2.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Meta_1", index=3, filename=self.path + "metacarp_1.stl", translation=[0, 0, 0], rotation=[0, 0, 0])

        # Index
        edit.addPart(node=self.visu, name="Index_p1", index=10, filename=self.path + "index_p1.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Index_p2", index=11, filename=self.path + "index_p2.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Index_p3", index=12, filename=self.path + "index_p3.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Index_p4", index=13, filename=self.path + "index_p4.stl", translation=[0, 0, 0], rotation=[0, 0, 0], collision=True)

        # Thumb
        edit.addPart(node=self.visu, name="Thumb_p1", index=4, filename=self.path + "thumb_p1.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Thumb_p2", index=5, filename=self.path + "thumb_p2.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Thumb_p3", index=6, filename=self.path + "thumb_p3.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Thumb_p4", index=7, filename=self.path + "thumb_p4.stl", translation=[0, 0, 0], rotation=[0, 0, 0])
        edit.addPart(node=self.visu, name="Thumb_p5", index=8, filename=self.path + "thumb_p5.stl", translation=[0, 0, 0], rotation=[0, 0, 0])

        self.centers = self.articulation.addChild("ArticulationCenters")
        edit.addCenter(self.centers, "Wrist_Link_c", 0, 1, [            0,             0,         48*2], [0, 0, 0], 0, 0, 1, [0, 1, 0], 0)
        edit.addCenter(self.centers, "Meta2_c", 1, 2,      [      (1.1)*2,     (-0.25)*2,  (32.6447)*2], [0, 0, 0], 0, 0, 1, [1, 0, 0], 1)
        edit.addCenter(self.centers, "Meta1_c", 2, 3,      [  (-9.6137)*2,  (-1.65902)*2,  (48.9203)*2], [0, 0, 0], 0, 0, 1, [-0.04, -0.005, 0.955], 2)

        edit.addCenter(self.centers, "Thumbp1_c", 3, 4,    [ (-10.4018)*2,    (4.8352)*2, (-47.8296)*2], [0, 0, 0], 0, 0, 1, [0.8, 0.04, 0.16], 3)
        edit.addCenter(self.centers, "Thumbp2_c", 4, 5,    [ (-13.0416)*2,   (4.00953)*2, (0.621956)*2], [0, 0, 0], 0, 0, 1, [-0.17, 0.0, 0.83], 4)
        edit.addCenter(self.centers, "Thumbp3_c", 5, 6,    [ (-7.40277)*2,  (-0.05484)*2,   (19.341)*2], [0, 0, 0], 0, 0, 1, [0.04, 0.95, 0.01], 5)
        edit.addCenter(self.centers, "Thumbp4_c", 6, 7,    [ (-9.05358)*2, (-0.346076)*2,  (38.9789)*2], [0, 0, 0], 0, 0, 1, [0.04, 0.95, 0.01], 6)
        edit.addCenter(self.centers, "Thumbp5_c", 7, 8,    [ (-7.28673)*2, (-0.358486)*2,  (35.7636)*2], [0, 0, 0], 0, 0, 1, [0.04, 0.95, 0.01], 7)
        edit.addCenter(self.centers, "Thumbp6_c", 8, 9,    [(-0.087104*2), (-0.622104*2),   (34.546)*2], [0, 0, 0], 0, 0, 0, [0, 0, 0], 8)

        edit.addCenter(self.centers, "Indexp1_c", 3, 10,   [ (-15.0262)*2,  (-2.13672)*2,  (29.5661)*2], [0, 0, 0], 0, 0, 1, [-0.05, 0.95, 0], 9)
        edit.addCenter(self.centers, "Indexp2_c", 10, 11,  [(-0.027241)*2,  (-1.60289)*2,       (14)*2], [0, 0, 0], 0, 0, 1, [0.95, 0.05, 0], 10)
        edit.addCenter(self.centers, "Indexp3_c", 11, 12,  [(-0.090864)*2,   (1.99792)*2,       (48)*2], [0, 0, 0], 0, 0, 1, [0.95, 0.05, 0], 11)
        edit.addCenter(self.centers, "Indexp4_c", 12, 13,  [(-0.040892)*2,   (0.89907)*2,  (27.9039)*2], [0, 0, 0], 0, 0, 1, [0.95, 0.05, 0], 12)
        edit.addCenter(self.centers, "Indexp5_c", 13, 14,  [(0.004486)*2,   (0.911137)*2,  (22.2986)*2], [0, 0, 0], 0, 0, 0, [0, 0, 0], 13)
