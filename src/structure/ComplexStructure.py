"""
File: ComplexStructure.py
Created Date: Tuesday, October 8th 2024, 10:28:16 am
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

from scipy.spatial.transform import Rotation as R
import numpy as np
import math
import copy

from structure import BasicStructure


class ComplexStructure(BasicStructure):
    """ComplexStructure is a class that extends BasicStructure to create a more complex robotic structure.

    It includes methods for creating the structure, articulation, rigid body, visualization, and inverse control.
    """

    def __init__(
        self,
        node: Sofa.Core.Node,
        path: str,
        name: str = "ComplexStructure",
        translation: list = [0, 0, 0],
        rotation: list = [0, 0, 0],
        positions: list = None,
        init_angles: list = None,
        visu_info: list = None,
    ) -> None:
        """Initialize the ComplexStructure.

        Args:
            node: The Sofa.Core.Node to which this structure belongs.
            path: The path to the structure's configuration.
            name: The name of the structure.
            translation: The translation vector for the structure.
            rotation: The rotation vector for the structure.
            positions: The initial positions for the structure.
            init_angles: The initial angles for the structure.
            visu_info: Visualization information for the structure.

        """
        print("Init " + name + "...")

        super().__init__(node, path, name, translation, rotation, positions, init_angles, visu_info)

    def createStructure(self, solver: str, constraint: bool = False) -> None:
        """Create the structure with the given solver and constraint.

        Args:
            solver: The solver to be used for the structure.
            constraint: Boolean indicating whether to apply constraints.

        """
        super().createStructure(solver, constraint=constraint)

        self.structure.addData("angles", self.init_angles, None, self.name + " angles", "", "vector<float>")

    def createArticulation(self, joint_limit: bool = False) -> None:
        """Create the articulation for the structure.

        Args:
            joint_limit: Boolean indicating whether to apply joint limits.

        """
        print("Create " + self.name + " articulation...")

        if self.structure is None:
            print("WARNING: Need to call createStructure first.")
            return

        self.articulation = self.structure.addChild("Articulation")

        self.articulation.addObject("MechanicalObject", name="dofs", template="Vec1", rest_position=self.structure.getData("angles").getLinkPath(), position=self.init_angles)
        self.articulation.addObject("ArticulatedHierarchyContainer")
        self.articulation.addObject("UniformMass", totalMass=45)  # TODO: Check with obj weight

        if joint_limit:
            for joint in self.joint_actuator:
                self.articulation.addObject("JointActuator", name="joint_" + str(joint[0]), index=joint[0], maxAngleVariation=0.001, minAngle=joint[1], maxAngle=joint[2])

            self.articulation.addObject("RestShapeSpringsForceField", stiffness=1e0, points=[i for i in range(0, len(self.init_angles))])  # TODO: Check for inverse solver
        else:
            self.articulation.addObject("RestShapeSpringsForceField", stiffness=1e13, angularStiffness=1e13, points=[i for i in range(0, len(self.init_angles))])  # TODO: Check for generic solver

    def createRigid(self) -> None:
        """Create the rigid body for the structure."""
        print("Create " + self.name + " rigid...")

        self.rigid = self.articulation.addChild("Rigid")

        self.mo = self.rigid.addObject(
            "MechanicalObject",
            name="dofs",
            template="Rigid3",
            showObject=True,
            showObjectScale=10,
            position=self.positions[0 : len(self.positions)],
            translation=self.translation,
            rotation=self.rotation,
        )
        self.rigid.addObject("ArticulatedSystemMapping", input1=self.articulation.dofs.getLinkPath(), output=self.rigid.dofs.getLinkPath(), applyRestPosition=True)

    def createVisualization(self, collision: bool = False) -> None:
        """Create the visualization for the structure.

        Args:
            collision: Boolean indicating whether to include collision visualization.

        """
        print("Create " + self.name + " visualization...")

        if self.rigid is None:
            self.visu = self.structure.addChild("Visu")
        else:
            self.visu = self.rigid.addChild("Visu")

        super().createVisualization(collision)

    def inverseControl(self, target_position: list) -> None:
        """Define the goal for inverse control.

        Args:
            target_position: The target positions for the control.

        """
        print("Defining " + self.name + " goal...")

        self.target = [None] * len(target_position)

        for i, target in enumerate(target_position):
            if target is None:
                continue

            self.target[i] = self.node.addChild(self.name + "_EffectorTarget_" + str(i))

            self.target[i].addObject("EulerImplicitSolver", firstOrder=True)
            self.target[i].addObject("CGLinearSolver", iterations=100, threshold=1e-2, tolerance=1e-5)
            self.target[i].addObject("MechanicalObject", name="dofs", template="Rigid3", position=target, showObject=1, showObjectScale=10, drawMode=1)
            self.target[i].addObject("UncoupledConstraintCorrection", name="UCC_t_" + self.name + "_" + str(i), defaultCompliance="0.1")

            self.rigid.addObject(
                "PositionEffector",
                name="pe_" + str(i),
                template="Rigid3",
                indices=self.indice[i],
                effectorGoal=self.target[i].dofs.findData("position").getLinkPath(),
                useDirections=[1, 1, 1, 1, 1, 1],
            )

    def updateAngle(self, n_angles: list) -> None:
        """Update the angles of the structure.

        Args:
            n_angles: The new angles to be set.

        """
        self.structure.angles = n_angles

    def updatePosition(self, n_pos: list) -> None:
        """Update the position of the mechanical object.

        Args:
            n_pos: The new position to be set.

        """
        self.mo.position = n_pos

    def updateTargetPosition(self, target_id: int, n_target_position: list) -> None:
        """Update the target position of the specified target.

        Args:
            target_id: The ID of the target to be updated.
            n_target_position: The new target position to be set.

        """
        if self.target[target_id] is None:
            return

        self.target[target_id].dofs.findData("position").setData = [n_target_position]

    @staticmethod
    def addCenter(
        node: Sofa.Core.Node,
        name: str,
        parentIndex: int,
        childIndex: int,
        posOnParent: list,
        posOnChild: list,
        articulationProcess: int,
        isTranslation: bool,
        isRotation: bool,
        axis: list,
        articulationIndex: int,
    ) -> None:
        """Add an articulation center to the node.

        Args:
            node: The Sofa.Core.Node to which the center is added.
            name: The name of the center.
            parentIndex: The index of the parent.
            childIndex: The index of the child.
            posOnParent: The position on the parent.
            posOnChild: The position on the child.
            articulationProcess: The articulation process.
            isTranslation: Boolean indicating if translation is involved.
            isRotation: Boolean indicating if rotation is involved.
            axis: The axis of rotation.
            articulationIndex: The index of the articulation.

        """
        center = node.addChild(name)
        center.addObject("ArticulationCenter", parentIndex=parentIndex, childIndex=childIndex, posOnParent=posOnParent, posOnChild=posOnChild, articulationProcess=articulationProcess)

        articulation = center.addChild("Articulation")
        articulation.addObject("Articulation", translation=isTranslation, rotation=isRotation, rotationAxis=axis, articulationIndex=articulationIndex)
