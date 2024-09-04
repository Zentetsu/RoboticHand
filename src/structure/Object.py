import SofaRuntime
import Sofa

from structure import Finger, edit
import math


class Object:
    def __init__(self, node, name, path, position=[0, 0, 0, 0, 0, 0, 1], translation=[0, 0, 0]):
        print("Init Object " + name + "...")

        self.sphere = node.addChild(name)
        self.sphere.addObject("EulerImplicitSolver")
        self.sphere.addObject("CGLinearSolver", threshold=1e-5, tolerance=1e-5, iterations=50)
        self.sphere.addObject("MechanicalObject", template="Rigid3", position=position)
        self.sphere.addObject("UniformMass", totalMass=0.1)
        self.sphere.addObject("UncoupledConstraintCorrection")

        self.addCollision(path)
        self.addVisu(path)

    def addCollision(self, path) -> None:
        print("Init collision..")

        self.collision = self.sphere.addChild("Collision")
        self.collision.addObject("MeshOBJLoader", name="loader", filename=path, scale=1)
        self.collision.addObject("MeshTopology", src="@loader")
        self.collision.addObject("MechanicalObject")
        self.collision.addObject("TriangleCollisionModel")
        self.collision.addObject("LineCollisionModel")
        self.collision.addObject("PointCollisionModel")
        self.collision.addObject("RigidMapping")

    def addVisu(self, path) -> None:
        self.visu = self.sphere.addChild("Visu")
        self.visu.addObject("MeshOBJLoader", name="loader", filename=path)
        self.visu.addObject("OglModel", name="Visual", src="@loader", color=[0.0, 0.1, 0.5], scale=1)
        self.visu.addObject("RigidMapping")