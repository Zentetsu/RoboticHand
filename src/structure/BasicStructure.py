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
                (0, name, name, [0, 0, 0], [0, 0, 0], False)
            ]
        else:
            self.visu_info = visu_info

        self.structure = None
        self.articulation = None
        self.rigid = None
        self.visu = None
        self.collision = None
        self.indice = 0

        self.ext = ".msh"

    def createStructure(self, solver=None, collision=False, constraint=False, type_c=0) -> None:
        print("Create " + self.name + "...")

        self.structure = self.node.addChild(self.name)

        if solver == "SparseLDLSolver":
            self.structure.addObject("EulerImplicitSolver", rayleighStiffness=1e-3, rayleighMass=1e-3)
            self.structure.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
        elif solver == "CGLinearSolver":
            self.structure.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
            self.structure.addObject("CGLinearSolver", threshold=1e-5, tolerance=1e-5, iterations=25)

        if "ComplexStructure" not in str(type(self).__base__):
            self.structure.addObject("MechanicalObject", name="Rigid_DOF", template="Rigid3", position=self.positions[0])

            if collision:
                self.structure.addObject("UniformMass", totalMass=0.005)

        if constraint:
            self.structure.addObject("UncoupledConstraintCorrection" if type_c else "GenericConstraintCorrection")#, linearSolver="@../Solver")

    def createRigid(self, collision=False) -> None:
        print("Create " + self.name + " rigid...")

        self.rigid = self.structure

        self.rigid.addObject("MechanicalObject", template="Rigid3", position=self.positions[0])

        if collision:
            self.rigid.addObject("UniformMass", totalMass=0.1)

    def createVisualization(self, collision=False) -> None:
        print("Create " + self.name + " visualization...")

        if self.visu is None:
            self.visu = self.structure

        if self.rigid is None:
            self.rigid = self.structure

        complx = "ComplexStructure" in str(type(self).__base__)

        for i, info in enumerate(self.visu_info):
            print(info[5])
            BasicStructure.addPart(self.visu, info[1], info[0], self.path + info[2] + self.ext, self.positions[i], info[3], info[4], rigid=self.rigid is not None, collision=info[5], complx=complx)

    @staticmethod
    def addPart(node, name, index, filename, position, translation=[0, 0, 0], rotation=[0, 0, 0], color=[0, 0, 1, 1], rigid=False, collision=False, complx=False) -> None:
        print("Add part " + name + "...")

        if complx:
            part = node.addChild(name)
        else:
            part = node

        BasicStructure.addMesh(part, name, filename)

        if complx:
            part.addObject("MechanicalObject", template="Rigid3", position=position)
            part.addObject("RigidMapping", index=index, globalToLocalCoords=False)

        BasicStructure.addVisu(part, name, index, translation, rotation, color, rigid, complx)

        if collision:
            BasicStructure.addCollision(part, name, complx, translation=translation, rotation=rotation)

    @staticmethod
    def addVisu(node, name, index, translation, rotation, color, rigid, complx) -> None:
        print("Add visu " + name + "...")

        visu = node.addChild("Visu_"+str(index))

        visu.addObject("OglModel", name="VisualModel", src="@../" + name, color=color, translation=translation, rotation=rotation, scale=1)

        if rigid and not complx:
            visu.addObject("RigidMapping", input="@../Rigid_DOF", output="@VisualModel")
        elif rigid:
            visu.addObject("RigidMapping", output="@VisualModel")

    @staticmethod
    def addCollision(node, name, complx, translation, rotation) -> None:
        print("Add collision " + name + "...")

        collision = node.addChild("Collision")

        collision.addObject("MeshTopology", src="@../" + name)
        collision.addObject("MechanicalObject", name="Collision_RM", translation=translation, rotation=rotation)
        collision.addObject("TriangleCollisionModel")
        collision.addObject("LineCollisionModel")
        collision.addObject("PointCollisionModel")
        if complx:
            collision.addObject("RigidMapping")
        else:
            collision.addObject("RigidMapping", input="@../Rigid_DOF", output="@Collision_RM")

    @staticmethod
    def addMesh(node, name, filename) -> None:
        print("Add mesh " + name + "...")

        if filename[-3:len(filename)] == "stl":
            node.addObject("MeshSTLLoader", name=name, filename=filename, scale=1)
        elif filename[-3:len(filename)] == "obj":
            node.addObject("MeshOBJLoader", name=name, filename=filename, scale=1)
        elif filename[-3:len(filename)] == "msh":
            node.addObject('MeshGmshLoader', name=name, filename=filename, createSubelements=True, flipNormals=0, scale=1)
