import SofaRuntime
import Sofa

from structure import Finger
import numpy as np
import math

from abc import abstractmethod


class BasicStructure():
    def __init__(self, node, path, name="BasicStructure", translation=[0, 0, 0], rotation=[0, 0, 0], positions=None, init_angles=None, visu_info=None):
        print("Init " + name + "...")

        self.node = node
        self.name = name
        self.path = path
        self.translation = translation
        self.rotation = rotation

        if positions is None:
            self.positions = [
                [0, 0, 0, 0, 0, 0, 1]
            ]
        else:
            self.positions = positions

        if init_angles is None:
            self.init_angles = [
                math.radians(0)
            ]
        else:
            self.init_angles = init_angles

        if visu_info is None:
            self.visu_info = [
                (name, name, [0, 0, 0], [0, 0, 0])
            ]
        else:
            self.visu_info = visu_info

        self.structure = None
        self.articulation = None
        self.rigid = None
        self.visu = None
        self.collision = None
        self.indice = 0

        self.ext = ".obj"

    def createStructure(self, solver=None, constraint=False) -> None:
        print("Create " + self.name + "...")

        self.structure = self.node.addChild(self.name)

        self.structure.addData("angles", self.init_angles, None, self.name + " angles", "", "vector<float>")

        if solver == "SparseLDLSolver":
            self.structure.addObject("EulerImplicitSolver")
            self.structure.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
        elif solver == "CGLinearSolver":
            self.structure.addObject("EulerImplicitSolver", rayleighMass=0.01)
            self.structure.addObject("CGLinearSolver", threshold=1e-5, tolerance=1e-5, iterations=50)

        if constraint:
            self.structure.addObject("GenericConstraintCorrection")

    def createArticulation(self, joint_actuator=False) -> None:
        print("Create " + self.name + " articulation...")

        if self.structure is None:
            print("WARNING: Need to call createStructure first.")
            return

        self.articulation = self.structure.addChild("Articulation")

        self.articulation.addObject("MechanicalObject", name="dofs", template="Vec1", rest_position=self.structure.getData("angles").getLinkPath(), position=self.init_angles)
        self.articulation.addObject("ArticulatedHierarchyContainer")
        self.articulation.addObject("UniformMass", vertexMass=0.1)

        self.articulation.addObject("RestShapeSpringsForceField", stiffness=1e10, points=[i for i in range(0, len(self.init_angles))])

    def createRigid(self) -> None:
        print("Create " + self.name + " rigid...")

        if self.articulation is None:
            self.rigid = self.structure.addChild("Rigid")

            self.structure.addObject("MechanicalObject", template="Rigid3")
            self.structure.addObject("UniformMass", totalMass=0.0001)
            self.structure.addObject("UncoupledConstraintCorrection")
        else:
            self.rigid = self.articulation.addChild("Rigid")

            self.mo = self.rigid.addObject("MechanicalObject", name="dofs", template="Rigid3", showObject=True, showObjectScale=10, position=self.positions[0:len(self.positions)], translation=self.translation, rotation=self.rotation)
            self.rigid.addObject("ArticulatedSystemMapping", input1=self.articulation.dofs.getLinkPath(), output=self.rigid.dofs.getLinkPath())

    def createVisualization(self, collision=False) -> None:
        print("Create " + self.name + " visualization...")

        if self.rigid is None:
            self.visu = self.structure.addChild("Visu")
        else:
            self.visu = self.rigid.addChild("Visu")

        for i, info in enumerate(self.visu_info):
            BasicStructure.addPart(self.visu, info[1], info[0], self.path + info[2] + self.ext, self.positions[i], info[3], info[4], rigid=True, collision=collision)

    def createArticulationCenter(self) -> None:
        print("Create " + self.name + " articulation center...")

        if self.articulation is None:
            print("WARNING: Need to call createArticulation first.")
            return

        self.centers = self.articulation.addChild("ArticulationCenters")

        BasicStructure.addCenter(self.centers, self.name, 0, 1, [0, 0, 0], [0, 0, 0], 0, 0, 1, [0, 1, 0], 0)

    def inverseControl(self, target_position) -> None:
        print("Defining " + self.name + " goal...")

        self.target = self.node.addChild(self.name + "_EffectorTarget")

        self.target.addObject("EulerImplicitSolver", firstOrder=True)
        self.target.addObject("CGLinearSolver", iterations=100, threshold=1e-2, tolerance=1e-5)
        self.target.addObject("MechanicalObject", name="dofs", template="Rigid3", position=target_position, showObject=1, showObjectScale=10, drawMode=1)
        self.target.addObject("UncoupledConstraintCorrection")

        self.rigid.addObject("PositionEffector", name="pe1", template="Rigid3", indices=self.indice, effectorGoal=self.target.dofs.findData('position').getLinkPath(), useDirections=[1, 1, 1, 0, 0, 0])

    @staticmethod
    def addPart(node, name, index, filename, position, translation=[0, 0, 0], rotation=[0, 0, 0], color=[1, 0, 0, 1], rigid=False, collision=False) -> None:
        part = node.addChild(name)

        if filename[-3:len(filename)] == "stl":
            part.addObject("MeshSTLLoader", name=name, filename=filename)
        else:
            part.addObject("MeshOBJLoader", name=name, filename=filename)

        if rigid:
            part.addObject("MechanicalObject", template="Rigid3", position=position)
            part.addObject("RigidMapping", index=index, globalToLocalCoords=False)

        if collision and not rigid:
            print("WARNING: need to create rigid to add collision.")
        elif collision:
            BasicStructure.addCollision(part, name)

        BasicStructure.addVisu(part, name, index, translation, rotation, color, rigid)

    @staticmethod
    def addVisu(node, name, index, translation, rotation, color, rigid) -> None:
        visu = node.addChild("Visu_"+str(index))

        visu.addObject("OglModel", name="VisualModel", src="@../" + name, color=color, translation=translation, rotation=rotation)

        if rigid:
            visu.addObject("RigidMapping")

    @staticmethod
    def addCollision(node, name) -> None:
        collision = node.addChild("Collision")

        collision.addObject("MeshTopology", src="@../" + name)
        collision.addObject("MechanicalObject")
        collision.addObject("TriangleCollisionModel")
        collision.addObject("LineCollisionModel")
        collision.addObject("PointCollisionModel")
        collision.addObject("RigidMapping")

    @staticmethod
    def addCenter(node, name, parentIndex, childIndex, posOnParent, posOnChild, articulationProcess, isTranslation, isRotation, axis, articulationIndex):
        center = node.addChild(name)
        center.addObject("ArticulationCenter", parentIndex=parentIndex, childIndex=childIndex, posOnParent=posOnParent, posOnChild=posOnChild, articulationProcess=articulationProcess)

        articulation = center.addChild("Articulation")
        articulation.addObject("Articulation", translation=isTranslation, rotation=isRotation, rotationAxis=axis, articulationIndex=articulationIndex)
