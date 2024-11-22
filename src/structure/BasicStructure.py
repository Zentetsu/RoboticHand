"""
File: BasicStructure.py
Created Date: Friday, October 11th 2024, 8:41:52 am
Author: Zentetsu

----

Last Modified: Fri Nov 22 2024
Modified By: Zentetsu

----

Project: structure
Copyright (c) 2024 Zentetsu

----

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http: //www.gnu.org/licenses/>.

----
HISTORY:
Date      	By	Comments
----------	---	---------------------------------------------------------
"""  # noqa

import SofaRuntime  # type: ignore
import Sofa  # type: ignore

import numpy as np
import copy
import math

from scipy.spatial.transform import Rotation
from abc import abstractmethod


class BasicStructure:
    """BasicStructure is a class that represents the basic structure of a robotic hand component.

    It provides methods to create and manage the structure, including visualization and collision handling.
    """

    def __init__(
        self,
        node: Sofa.Core.Node,
        path: str,
        name: str = "BasicStructure",
        translation: list = [0, 0, 0],
        rotation: list = [0, 0, 0],
        positions: list = None,
        init_angles: list = None,
        visu_info: list = None,
    ) -> None:
        """Initialize the BasicStructure.

        Args:
            node: The Sofa.Core.Node object.
            path: The path to the structure files.
            name: The name of the structure.
            translation: The translation vector.
            rotation: The rotation vector.
            positions: The positions list.
            init_angles: The initial angles list.
            visu_info: The visualization information list.

        """
        print("Init " + name + "...")

        self.node = node
        self.name = name
        self.path = path
        self.translation = translation
        self.rotation = rotation

        if positions is None:
            self.positions = [[0, 0, 0, 0, 0, 0, 1]]
        else:
            self.positions = positions

        if init_angles is None:
            self.init_angles = [math.radians(0)]
        else:
            self.init_angles = init_angles

        if visu_info is None:
            self.visu_info = [(0, name, name, self.positions[:3], self.positions[3:], False)]
        else:
            self.visu_info = visu_info

        self.structure = None
        self.articulation = None
        self.rigid = None
        self.visu = None
        self.collision = None
        self.indice = 0
        self.deformable = False

        self.ext = ".stl"

    def createStructure(self, solver: str = None, collision: bool = False, constraint: bool = False, type_c: int = 0, deformable: bool = False) -> None:
        """Create the structure of the robotic hand component.

        Args:
            solver: The solver to use for the structure.
            collision: Whether to add collision handling.
            constraint: Whether to add constraints.
            type_c: The type of constraint correction to use.
            deformable: Whether the structure is deformable.

        """
        print("Create " + self.name + "...")

        self.structure = self.node.addChild(self.name)
        self.deformable = deformable

        if solver == "SparseLDLSolver":
            self.structure.addObject("EulerImplicitSolver", rayleighStiffness=1e-4, rayleighMass=1e-4)
            self.structure.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
        elif solver == "CGLinearSolver":
            self.structure.addObject("EulerImplicitSolver")  # , rayleighStiffness=1e-3, rayleighMass=1e-3)
            self.structure.addObject("CGLinearSolver", threshold=1e-5, tolerance=1e-5, iterations=50)

        if "ComplexStructure" not in str(type(self).__base__):
            if self.deformable:
                self.__createDeformable()
            else:
                self.mo = self.structure.addObject("MechanicalObject", name="Rigid_DOF", template="Rigid3", position=self.positions[0])

            if collision:
                self.structure.addObject("UniformMass", totalMass=0.005)  # TODO: CHeck with arm weight

        if constraint:
            self.structure.addObject("UncoupledConstraintCorrection" if type_c else "GenericConstraintCorrection")  # , linearSolver="@../Solver")

    def __createDeformable(self) -> None:
        """Create the deformable structure of the robotic hand component."""
        BasicStructure.addMesh(self.structure, self.name + "_def", self.path + self.name + ".msh", translation=self.visu_info[0][3], rotation=self.visu_info[0][4])
        self.structure.addObject("MeshTopology", src="@" + self.name + "_def")
        self.structure.addObject("MechanicalObject", template="Vec3")
        self.structure.addObject("ParallelTetrahedronFEMForceField", poissonRatio=0.1, youngModulus=50)
        # self.structure.addObject('BoxROI', box=[-75, 300, -25, -175, 400, 75], drawBoxes=True, name='boxROI')
        # self.structure.addObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness=1e1)
        # self.structure.addObject('RestShapeSpringsForceField', stiffness=1e0)
        # self.structure.addObject('LinearSolverConstraintCorrection')
        # self.structure.addObject('FixedPlaneProjectiveConstraint', template='Vec3', name='default12', direction=[0, 0, 1], dmin=-5, dmax=5)
        # self.structure.addObject('FixedProjectiveConstraint', template='Vec3', name='default13', indices=[0, 1, 2, 3])

    def updatePosition(self, n_pos: np.ndarray, matrix: bool = False) -> None:
        """Update the position of the mechanical object.

        Args:
            n_pos: The new position.
            matrix: Whether the position is given as a transformation matrix.

        """
        # print(self.mo.position.value, n_pos)

        if matrix:
            translation = list(n_pos[:3, 3] * 1000)
            rotation = copy.copy(n_pos[:3, :3])

            rotation = Rotation.from_matrix(rotation)
            quaternion = rotation.as_quat()
            self.mo.position = [[*translation, *quaternion]]
        else:
            pos = list(n_pos)
            self.mo.position = [[*pos, 0, 0, 0, 1]]

    def createRigid(self, collision: bool = False) -> None:
        """Create the rigid structure of the robotic hand component.

        Args:
            collision: Whether to add collision handling.

        """
        print("Create " + self.name + " rigid...")

        self.rigid = self.structure

        self.rigid.addObject("MechanicalObject", template="Rigid3", position=self.positions[0])

        if collision:
            self.rigid.addObject("UniformMass", totalMass=0.1)

    def createVisualization(self, collision: bool = False) -> None:
        """Create the visualization of the robotic hand component.

        Args:
            collision: Whether to add collision handling.

        """
        print("Create " + self.name + " visualization...")

        if self.visu is None:
            self.visu = self.structure

        if self.rigid is None:
            self.rigid = self.structure

        complx = "ComplexStructure" in str(type(self).__base__)

        for i, info in enumerate(self.visu_info):
            trans = [0, 0, 0] if not self.deformable and not complx else info[3]
            rota = [0, 0, 0] if not self.deformable and not complx else info[4]
            defo = True if self.deformable and not complx else False
            path = None if info[2] is None else self.path + info[2] + self.ext
            BasicStructure.addPart(self.visu, info[1], info[0], path, self.positions[i], trans, rota, rigid=self.rigid is not None, collision=info[5], complx=complx, deformable=defo)

    @staticmethod
    def addPart(
        node: Sofa.Core.Node,
        name: str,
        index: int,
        filename: str,
        position: list,
        translation: list = [0, 0, 0],
        rotation: list = [0, 0, 0],
        color: list = [0, 0, 1, 1],
        rigid: bool = False,
        collision: bool = False,
        complx: bool = False,
        deformable: bool = False,
    ) -> None:
        """Add a part to the node.

        Args:
            node: The node to add the part to.
            name: The name of the part.
            index: The index of the part.
            filename: The filename of the mesh.
            position: The position of the part.
            translation: The translation vector.
            rotation: The rotation vector.
            color: The color of the part.
            rigid: Whether the part is rigid.
            collision: Whether to add collision handling.
            complx: Whether the part is complex.
            deformable: Whether the part is deformable.

        """
        print("Add part " + name + "...")

        if complx:
            part = node.addChild(name)
        else:
            part = node

        if filename is not None:
            BasicStructure.addMesh(part, name, filename)

        if complx:
            if deformable:
                print("defo")
                BasicStructure.addMesh(part, name + "_def", filename[:-3] + "msh", translation=translation, rotation=translation)
                part.addObject("MeshTopology", src="@" + name + "_def")
                part.addObject("MechanicalObject", template="Vec3")
                part.addObject("ParallelTetrahedronFEMForceField", poissonRatio=0.49, youngModulus=100, method="large")
            else:
                part.addObject("MechanicalObject", template="Rigid3", position=position)
                part.addObject("RigidMapping", index=index, globalToLocalCoords=False)

        if filename is not None:
            BasicStructure.addVisu(part, name, index, translation, rotation, color, rigid, complx, deformable)

        if collision:
            BasicStructure.addCollision(part, name, complx, translation, rotation, deformable)

    @staticmethod
    def addVisu(node: Sofa.Core.Node, name: str, index: int, translation: list, rotation: list, color: list, rigid: bool, complx: bool, deformable: bool) -> None:
        """Add visualization to the node.

        Args:
            node: The node to add the visualization to.
            name: The name of the visualization.
            index: The index of the visualization.
            translation: The translation vector.
            rotation: The rotation vector.
            color: The color of the visualization.
            rigid: Whether the visualization is rigid.
            complx: Whether the visualization is complex.
            deformable: Whether the visualization is deformable.

        """
        print("Add visu " + name + "...")

        visu = node.addChild("Visu_" + str(index))

        visu.addObject("OglModel", name="VisualModel", src="@../" + name, color=color, translation=translation, rotation=rotation, scale=1)

        if rigid and not complx:
            if deformable:
                visu.addObject("BarycentricMapping")
            else:
                visu.addObject("RigidMapping", input="@../Rigid_DOF", output="@VisualModel")
        elif rigid:
            if deformable:
                visu.addObject("BarycentricMapping")
            else:
                visu.addObject("RigidMapping", output="@VisualModel")

    @staticmethod
    def addCollision(node: Sofa.Core.Node, name: str, complx: bool, translation: list, rotation: list, deformable: bool) -> None:
        """Add collision handling to the node.

        Args:
            node: The node to add the collision to.
            name: The name of the collision.
            complx: Whether the collision is complex.
            translation: The translation vector.
            rotation: The rotation vector.
            deformable: Whether the collision is deformable.

        """
        print("Add collision " + name + "...")

        collision = node.addChild("Collision")

        collision.addObject("MeshTopology", src="@../" + name)
        collision.addObject("MechanicalObject", name="Collision_RM", translation=translation, rotation=rotation)

        if complx:
            collision.addObject("TriangleCollisionModel", contactResponse="StickContactConstraint")  # , contactResponse="StickContactForceField")
            collision.addObject("LineCollisionModel", contactResponse="StickContactConstraint")  # , contactResponse="StickContactForceField")
            collision.addObject("PointCollisionModel", contactResponse="StickContactConstraint")  # , contactResponse="StickContactForceField")
            if deformable:
                collision.addObject("BarycentricMapping")
            else:
                collision.addObject("RigidMapping")
        else:
            collision.addObject("TriangleCollisionModel")
            collision.addObject("LineCollisionModel")
            collision.addObject("PointCollisionModel")
            if deformable:
                collision.addObject("BarycentricMapping")
            else:
                collision.addObject("RigidMapping", input="@../Rigid_DOF", output="@Collision_RM")

    @staticmethod
    def addMesh(node: Sofa.Core.Node, name: str, filename: str, translation: list = [0, 0, 0], rotation: list = [0, 0, 0]) -> None:
        """Add a mesh to the node.

        Args:
            node: The node to add the mesh to.
            name: The name of the mesh.
            filename: The filename of the mesh.
            translation: The translation vector.
            rotation: The rotation vector.

        """
        print("Add mesh " + name + "...")

        if filename[-3 : len(filename)] == "stl":
            node.addObject("MeshSTLLoader", name=name, filename=filename, scale=1)
        elif filename[-3 : len(filename)] == "obj":
            node.addObject("MeshOBJLoader", name=name, filename=filename, scale=1)
        elif filename[-3 : len(filename)] == "msh":
            node.addObject("MeshGmshLoader", name=name, filename=filename, translation=translation, rotation=rotation, scale=1)
