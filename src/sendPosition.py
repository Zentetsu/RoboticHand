"""
File: sendPosition.py
Created Date: Tuesday, October 8th 2024, 10:28:16 am
Author: Zentetsu

----

Last Modified: Fri Nov 22 2024
Modified By: Zentetsu

----

Project: src
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

from IRONbark import Module
import numpy as np
import threading
import keyboard
import time
import math
import os

import roboticstoolbox as rtb
from spatialmath import SO3, SE3


from curtsies import Input


class ControllerKB:
    """ControllerKB class to handle keyboard inputs for controlling the robotic hand."""

    def __new__(cls, verbose: bool = False) -> "ControllerKB":
        """Create and return a new instance of ControllerKB."""
        if not hasattr(cls, "instance") or not cls.instance:
            cls.instance = super().__new__(cls)
            cls.instance.ControllerKB(verbose)

        return cls.instance

    def ControllerKB(self, verbose: bool) -> None:
        """Initialize the ControllerKB with verbosity option."""
        self.verbose = verbose

        self.command = {
            "esc": False,
            "0": False,
            "1": False,
            "2": False,
            "up": False,
            "down": False,
            "left": False,
            "right": False,
            "page up": False,
            "page down": False,
            "cmd": False,
            "i": False,
            "o": False,
            "space": False,
        }

    def createInput(self) -> None:
        """Start listening for keyboard input."""
        keyboard.hook(self.updateInput)

    def updateInput(self, key) -> None:
        """Update the command dictionary.

        Args:
            key: The key event from the keyboard.
        """
        for k in self.command.keys():
            self.command[k] = keyboard.is_pressed(k)

    def getInput(self) -> dict:
        """Return the current state of the command dictionary."""
        return self.command


if __name__ == "__main__":
    path = "/home/zentetsu/Documents/PhD/"  # os.environ["PHDPATH"]
    links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(
        path + "/RoboticHand/model/myArm750/myArm750.URDF"
    )
    IK_arm = rtb.Robot(
        links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath
    )

    links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(
        path + "/RoboticHand/model/hand/index.URDF"
    )
    IK_hand_in = rtb.Robot(
        links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath
    )

    links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(
        path + "/RoboticHand/model/hand/thumb.URDF"
    )
    IK_hand_th = rtb.Robot(
        links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath
    )

    Target_Module = Module(file="./data/Target_Module.json")
    q0_arm = Target_Module["target"]["wrist_a"]
    q0_hand_in = Target_Module["target"]["index_a"]
    q0_hand_th = Target_Module["target"]["thumb_a"]

    T_ar = IK_arm.fkine(q0_arm)
    T_in = IK_hand_in.fkine(q0_hand_in)
    T_th = IK_hand_th.fkine(q0_hand_th)

    T = [T_ar, T_in, T_th]

    cKB = ControllerKB()
    cKB.createInput()

    target = 0
    weights = [1, 1, 1, 0, 0, 0]

    while not cKB.getInput()["esc"]:
        time.sleep(0.01)

        if cKB.getInput()["0"]:
            target = 0
        elif cKB.getInput()["1"]:
            target = 1
        elif cKB.getInput()["2"]:
            target = 2

        finger = 1 if not cKB.getInput()["cmd"] else 0
        if cKB.getInput()["up"]:
            if not cKB.getInput()["cmd"]:
                T[target] = T[target] * SE3.Tx(0.001)
            else:
                T[target] = T[target] * SE3.Rx(math.radians(1))
        elif cKB.getInput()["down"]:
            if not cKB.getInput()["cmd"]:
                T[target] = T[target] * SE3.Tx(-0.001)
            else:
                T[target] = T[target] * SE3.Rx(math.radians(-1))
        if cKB.getInput()["left"]:
            if not cKB.getInput()["cmd"]:
                T[target] = T[target] * SE3.Ty(0.001)
            else:
                T[target] = T[target] * SE3.Ry(math.radians(1))
        elif cKB.getInput()["right"]:
            if not cKB.getInput()["cmd"]:
                T[target] = T[target] * SE3.Ty(-0.001)
            else:
                T[target] = T[target] * SE3.Ry(math.radians(-1))
        if cKB.getInput()["page up"]:
            if not cKB.getInput()["cmd"]:
                T[target] = T[target] * SE3.Tz(0.001)
            else:
                T[target] = T[target] * SE3.Rz(math.radians(1))
        elif cKB.getInput()["page down"]:
            if not cKB.getInput()["cmd"]:
                T[target] = T[target] * SE3.Tz(-0.001)
            else:
                T[target] = T[target] * SE3.Rz(math.radians(-1))

        sol = IK_arm.ikine_QP(T[0], q0=q0_arm, pi=0.001)
        q0_arm = sol.q

        sol = IK_hand_in.ikine_LM(T[1], q0=q0_hand_in, mask=weights)
        if not np.isnan(sol.q).any():
            q0_hand_in = sol.q

        sol = IK_hand_th.ikine_LM(T[2], q0=q0_hand_th, mask=weights)
        if not np.isnan(sol.q).any():
            q0_hand_th = sol.q

        eul = [list(T[0].eul()), list(T[1].eul()), list(T[2].eul())]
        Target_Module["target"]["wrist_p"] = np.array(
            [
                T[0].x * 1000,
                T[0].y * 1000,
                T[0].z * 1000,
                *eul[0],
            ]
        )
        Target_Module["target"]["index_p"] = np.array(
            [
                T[1].x * 1000,
                T[1].y * 1000,
                T[1].z * 1000,
                *eul[1],
            ]
        )
        Target_Module["target"]["thumb_p"] = np.array(
            [
                T[2].x * 1000,
                T[2].y * 1000,
                T[2].z * 1000,
                *eul[2],
            ]
        )
        Target_Module["target"]["wrist_a"] = q0_arm
        Target_Module["target"]["index_a"] = q0_hand_in
        Target_Module["target"]["thumb_a"] = q0_hand_th

    # time.sleep(5)
    Target_Module.stopModule(name="target")
