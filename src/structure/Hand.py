"""
File: Hand.py
Created Date: Monday, September 16th 2024, 7:11:23 pm
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
from structure import ComplexStructure
import numpy as np
import math
import copy


class Hand(ComplexStructure):
    """Hand class represents a robotic hand structure with articulation and joint actuator functionalities."""

    def __init__(
        self,
        node: Sofa.Core.Node,
        path: str,
        name: str = "Hand",
        translation: list = [0, 0, 0],
        rotation: list = [0, 0, 0],
        positions: list = None,
        init_angles: list = None,
        visu_info: list = None,
        joint_actuator: list = None,
        articulation_info: list = None,
    ) -> None:
        """Initialize the Hand object.

        Args:
            node: The node to which the hand is attached.
            path: The path to the hand's model.
            name: The name of the hand.
            translation: The translation vector for the hand's position.
            rotation: The rotation vector for the hand's orientation.
            positions: The initial positions of the hand's joints.
            init_angles: The initial angles of the hand's joints.
            visu_info: Visualization information for the hand.
            joint_actuator: The joint actuator for the hand.
            articulation_info: Information about the hand's articulations.

        """
        self.joint_actuator = joint_actuator
        self.articulation_info = articulation_info

        super().__init__(node, path, name, translation, rotation, positions, init_angles, visu_info)

        self.ext = ".stl"

    def createArticulationCenter(self, indice: list) -> None:
        """Create articulation centers for the hand.

        Args:
            indice: List of indices for the articulation centers.

        """
        print("Create " + self.name + " articulation center...")

        self.centers = self.articulation.addChild("ArticulationCenters")

        for i, articullation in enumerate(self.articulation_info):
            ComplexStructure.addCenter(
                self.centers, articullation[0], articullation[1], articullation[2], articullation[3], articullation[4], 0, 0, articullation[5], articullation[6], articullation[7]
            )

        self.indice = indice

    def updateAngle(self, n_angles_pa: list, n_angles_in: list, n_angles_th: list) -> None:
        """Update the angles of the hand's joints.

        Args:
            n_angles_pa: List of angles for the palm.
            n_angles_in: List of angles for the index finger.
            n_angles_th: List of angles for the thumb.

        """
        n_a = [
            -n_angles_pa[0],
            n_angles_pa[1],
            0.002443461,
            -0.06021386,
            n_angles_pa[2],
            -0.191986,
            -0.083339472,
            n_angles_th[0],
            0,
            -0.174533,
            n_angles_th[1],
            n_angles_th[2],
            n_angles_th[3],
            n_angles_th[4],
            0,  # n_angles_in[7], #n_angles_in[8], #n_angles_in[9], #n_angles_in[10], n_angles_in[11],
            -0.0028797933,
            -0.04258603,
            n_angles_in[0],
            n_angles_in[1],
            n_angles_in[2],
            n_angles_in[3],
            0,
        ]

        self.structure.angles = n_a

    def getPosition(self) -> list:
        """Get the current position of the hand's joints.

        Returns:
            A list of angles representing the current position of the hand's joints.

        """
        rotation_quat = R.from_euler("xyz", [math.radians(0), math.radians(90), math.radians(90)]).as_quat()
        angles = [0] * len(self.mo.position.value)
        d_angle = np.array([0, 0, 0])
        angles_ = []

        # print(self.mo.position.value)
        for i in range(1, len(angles)):
            quaternion = np.array(self.mo.position.value[i][3:])
            rotated_quat = R.from_quat(rotation_quat).inv() * R.from_quat(quaternion)

            # rotated_quat = R.from_quat(quaternion)
            angles[i] = list(rotated_quat.as_euler("xyz", degrees=True) - d_angle)
            d_angle = d_angle + np.array(angles[i])

            if self.articulation_info[i - 1][5] == 1:
                angles_.append(round(np.dot(np.array(angles[i]), np.array(self.articulation_info[i - 1][6])), 4))
            else:
                angles_.append(0)

            # angles_.append(list(self.mo.position.value[i]))

        return angles_

    def attachToRobot(self) -> None:
        """Attach the hand to the robot by mapping the hand's articulation to the robot's arm."""
        self.node.addObject("RigidMapping", input="@./Arm/Articulation/Rigid/dofs", output="@./Hand/Articulation/Rigid/dofs", index=6)

        rotation_quat = R.from_euler("xyz", [-math.radians(90), 0, 0]).as_quat()
        new_pos = copy.copy(self.mo.position.value)

        for i in range(0, len(self.mo.position.value) - 1):
            quaternion = np.array(new_pos[i][3:])
            rotated_quat = R.from_quat(rotation_quat) * R.from_quat(quaternion)
            new_quaternion = rotated_quat.as_quat()

            rotation_matrix = R.from_quat(rotation_quat).as_matrix()
            new_position = rotation_matrix @ np.array(new_pos[i][:3])

            result = np.concatenate((new_position, new_quaternion))
            new_pos[i] = result

        self.mo.findData("position").setData = new_pos
