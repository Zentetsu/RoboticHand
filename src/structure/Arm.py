"""
File: Arm.py
Created Date: Tuesday, September 10th 2024, 3:25:08 pm
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


class Arm(ComplexStructure):
    """Arm class represents a robotic arm structure with articulation centers and joint actuators."""

    def __init__(
        self,
        node: Sofa.Core.Node,
        path: str,
        name: str = "Arm",
        translation: list = [0, 0, 0],
        rotation: list = [0, 0, 0],
        positions: list = None,
        init_angles: list = None,
        visu_info: list = None,
        joint_actuator: list = None,
        articulation_info: list = None,
    ) -> None:
        """Initialize the Arm class.

        Args:
            node: The Sofa node to which the arm belongs.
            path: The path to the arm's model.
            name: The name of the arm.
            translation: The translation vector for the arm.
            rotation: The rotation vector for the arm.
            positions: The positions of the arm's components.
            init_angles: The initial angles of the arm's joints.
            visu_info: Visualization information for the arm.
            joint_actuator: The joint actuator for the arm.
            articulation_info: Information about the arm's articulations.

        """
        self.joint_actuator = joint_actuator
        self.articulation_info = articulation_info

        super().__init__(node, path, name, translation, rotation, positions, init_angles, visu_info)

        self.ext = ".stl"

    def createArticulationCenter(self) -> None:
        """Create the articulation centers for the arm."""
        print("Create " + self.name + " articulation center...")

        self.centers = self.articulation.addChild("ArticulationCenters")

        for i, articullation in enumerate(self.articulation_info):
            ComplexStructure.addCenter(
                self.centers, articullation[0], articullation[1], articullation[2], articullation[3], articullation[4], 0, 0, articullation[5], articullation[6], articullation[7]
            )

        self.indice = [len(self.articulation_info)]

    def getPosition(self) -> list:
        """Get the current position of the arm's joints.

        Returns:
            list: A list of angles representing the current position of the arm's joints.

        """
        angles = [0] * len(self.mo.position.value)
        d_angle = np.array([0, 0, 0])
        angles_ = []

        for i in range(1, len(angles)):
            angles[i] = list(R.from_quat(self.mo.position.value[i][3:]).as_euler("xyz", degrees=True) - d_angle)
            d_angle = d_angle + np.array(angles[i])

            if self.articulation_info[i - 1][5] == 1:
                angles_.append(round(np.dot(np.array(angles[i]), np.array(self.articulation_info[i - 1][6])), 2))

        return angles_
