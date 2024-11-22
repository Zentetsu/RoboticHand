"""
File: updateRobot.py
Created Date: Wednesday, October 2nd 2024, 12:26:51 pm
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

from pymycobot import MyArmM
from IRONbark import Module
import time


if __name__ == "__main__":
    myarmm = MyArmM("/dev/tty.usbserial-58690034881", 1000000)
    speed = 2

    robot_Module = Module(file="./data/Robot_Module.json")
    angles = [0] * 7

    while robot_Module.getLSAvailability(listener=True)[1][0]:
        time.sleep(0.01)
        print(robot_Module["sofa_i"]["arm_ang"])
        n_angles = robot_Module["sofa_i"]["arm_ang"]
        angles[0] = -n_angles[0]
        angles[1] = -n_angles[1]
        angles[2] = n_angles[2]
        angles[3] = -n_angles[3]
        angles[4] = -n_angles[4]
        angles[5] = -n_angles[5]
        angles[6] = -75
        print(angles)

        myarmm.set_joints_angle(angles, speed)

    robot_Module.stopModule(name="robot")
