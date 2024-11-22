"""
File: tools.py
Created Date: Tuesday, October 8th 2024, 10:26:40 am
Author: Zentetsu

----

Last Modified: Fri Nov 22 2024
Modified By: Zentetsu

----

Project: tools
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

import math

import numpy as np
import Sofa  # type: ignore
import Sofa.Gui  # type: ignore
import SofaRuntime  # type: ignore
from scipy.spatial.transform import Rotation as R

USE_GUI = False


def addPlugins(node: Sofa.Core.Node) -> None:
    """Add required plugins to the given Sofa node.

    Args:
        node: The Sofa node to which plugins will be added.

    """
    print("Add plugins...")

    node.addObject(
        "RequiredPlugin",
        pluginName=[
            "Sofa.Component.Constraint.Lagrangian.Correction",
            "Sofa.Component.Constraint.Lagrangian.Solver",
            "Sofa.Component.Constraint.Projective",
            "Sofa.Component.Collision.Detection.Intersection",
            "Sofa.Component.Collision.Detection.Algorithm",
            "Sofa.Component.Collision.Response.Contact",
            "Sofa.Component.Collision.Geometry",
            "Sofa.Component.Topology.Container.Constant",
            "Sofa.Component.LinearSolver.Iterative",
            "Sofa.Component.LinearSolver.Direct",
            "Sofa.Component.SolidMechanics.FEM.Elastic",
            "Sofa.Component.SolidMechanics.Spring",
            "Sofa.Component.ODESolver.Backward",
            "Sofa.Component.Mapping.NonLinear",
            "Sofa.Component.Mapping.Linear",
            "Sofa.Component.MechanicalLoad",
            "Sofa.Component.StateContainer",
            "Sofa.Component.AnimationLoop",
            "Sofa.Component.IO.Mesh",
            "Sofa.Component.Visual",
            "Sofa.Component.Mass",
            "Sofa.GL.Component.Rendering3D",
            "ArticulatedSystemPlugin",
            "SoftRobots.Inverse",
            "SoftRobots",
            "MultiThreading",
        ],
    )


def initScene(node: Sofa.Core.Node, path: str, ground: bool = False, generic_solver: int = 0) -> None:
    """Initialize the Sofa scene with the given parameters.

    Args:
        node: The Sofa node to which the scene will be added.
        path: The path to the resources needed for the scene.
        ground: A boolean indicating whether to add ground to the scene.
        generic_solver: An integer indicating whether to use a generic solver.

    """
    print("Init scene...")

    node.addObject("FreeMotionAnimationLoop")
    node.addObject("VisualStyle", displayFlags="showVisualModels showBehaviorModels showInteractionForceFields", bbox=[-1, -1, -1, 1, 1, 1])

    node.gravity = [0, 0, -9810.0]
    node.dt = 0.001

    if generic_solver:
        node.addObject("GenericConstraintSolver", tolerance=1e-7, maxIterations=1000)

    node.addObject("CollisionPipeline")
    node.addObject("ParallelBruteForceBroadPhase")
    node.addObject("ParallelBVHNarrowPhase")
    node.addObject("CollisionResponse", response="FrictionContactConstraint", responseParams="mu=1.5")
    node.addObject("LocalMinDistance", name="Proximity", alarmDistance=5, contactDistance=1)
    # node.addObject('NewProximityIntersection', alarmDistance=5, contactDistance=1)
    # node.addObject('MinProximityIntersection', alarmDistance=10, contactDistance=5)

    # Inverse solver
    # node.addObject("LCPConstraintSolver", maxIt=1000, tolerance=0.001)
    if not generic_solver:
        node.addObject("QPInverseProblemSolver", maxIterations=10000, tolerance=1e-2, epsilon=1e-4, printLog=False)

    if ground:
        addGround(node, path)


def addGround(node: Sofa.Core.Node, path: str) -> None:
    """Add ground to the given Sofa node.

    Args:
        node: The Sofa node to which the ground will be added.
        path: The path to the ground resource.

    """
    print("Add ground...")

    ground = node.addChild("Ground")

    ground.addObject("MeshSTLLoader", name="loader", filename=path + "Ground.stl", rotation=[0, 0, 0], scale=1, translation=[0, 0, 0])
    ground.addObject("MeshTopology", src="@loader")
    ground.addObject("MechanicalObject", src="@loader")
    ground.addObject("TriangleCollisionModel", moving=0, simulated=0)
    ground.addObject("LineCollisionModel", moving=0, simulated=0)
    ground.addObject("PointCollisionModel", moving=0, simulated=0)
    ground.addObject("OglModel", name="Visual", src="@loader", color=[0.5, 0.5, 0.5, 1])


def launchGUI(root: Sofa.Core.Node) -> None:
    """Launch the Sofa GUI or run the simulation for a few iterations.

    Args:
        root: The root node of the Sofa simulation.

    """
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("Scene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1920, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()


def eulToQuat(euler_angles: list, rad: bool = True) -> np.ndarray:
    """Convert Euler angles to quaternion.

    Args:
        euler_angles: A list or array of Euler angles [roll, pitch, yaw].
        rad: A boolean indicating whether the angles are in radians (default is True).

    Returns:
        A numpy array representing the quaternion [x, y, z, w].

    """
    return R.from_euler("xyz", euler_angles, degrees=not rad).as_quat()


def getForearmFromHand(palm_coords: list, local_translation_y: float) -> tuple:
    """Calculate the forearm position and orientation from the hand's palm coordinates.

    Args:
        palm_coords: A list or array containing the palm coordinates and orientation [x, y, z, roll, pitch, yaw].
        local_translation_y: The local translation along the y-axis.

    Returns:
        A tuple containing the forearm position and orientation (x, y, z, qx, qy, qz, qw).

    """
    x, y, z = palm_coords[0:3]
    qx, qy, qz, qw = eulToQuat([palm_coords[3], palm_coords[4], palm_coords[5]])

    T_y = local_translation_y
    t_local = np.array([0, T_y, 0])

    rotation = R.from_quat([qx, qy, qz, qw])
    R_forearm = rotation.as_matrix()

    t_global = R_forearm.dot(t_local)
    p_forearm = np.array([x, y, z])
    p_wrist = p_forearm + t_global

    return p_wrist[0], p_wrist[1], p_wrist[2], qx, qy, qz, qw


def eulerRotationMatrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Calculate the rotation matrix from Euler angles.

    Args:
        roll: The roll angle in degrees.
        pitch: The pitch angle in degrees.
        yaw: The yaw angle in degrees.

    Returns:
        A 3x3 numpy array representing the rotation matrix.

    """
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def matrixOriginToWrist(wrist_pos: list, wrist_ori: list) -> np.ndarray:
    """Calculate the transformation matrix from the origin to the wrist.

    Args:
        wrist_pos: A list containing the wrist position [x, y, z].
        wrist_ori: A list containing the wrist orientation [roll, pitch, yaw].

    Returns:
        A 4x4 numpy array representing the transformation matrix.

    """
    R = eulerRotationMatrix(*wrist_ori)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = wrist_pos

    return T


def invertMatrix(T: np.ndarray) -> np.ndarray:
    """Invert a given transformation matrix.

    Args:
        T: A 4x4 numpy array representing the transformation matrix.

    Returns:
        A 4x4 numpy array representing the inverted transformation matrix.

    """
    R = T[:3, :3]
    t = T[:3, 3]

    R_inv = R.T
    t_inv = -R_inv @ t

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv


def coEulerToQuat(c_wridt: list, c_finger: list, add_rotation_xyz: tuple = (0, 0, 0)) -> list:
    """Convert wrist and finger coordinates from Euler angles to quaternion.

    Args:
        c_wridt: A list containing the wrist coordinates and orientation [x, y, z, roll, pitch, yaw].
        c_finger: A list containing the finger coordinates and orientation [x, y, z, roll, pitch, yaw].
        add_rotation_xyz: A tuple containing additional rotation angles (roll, pitch, yaw) in degrees.

    Returns:
        A list representing the target coordinates and orientation in quaternion [x, y, z, qx, qy, qz, qw].

    """
    matrix = matrixOriginToWrist(c_wridt[:3], c_wridt[3:])

    g_target_th_po = matrix @ np.array([*c_finger[:3], 1])
    additional_rotation = R.from_euler("xyz", add_rotation_xyz, degrees=True).as_matrix()

    combined_rotation_matrix = matrix[:3, :3] @ eulerRotationMatrix(*c_finger[3:]) @ additional_rotation
    g_eu = R.from_matrix(combined_rotation_matrix).as_euler("xyz", degrees=True)

    g_target_th_eu = [*g_target_th_po[:3], *g_eu]
    g_target_th_qu = [*g_target_th_eu[:3], *eulToQuat(g_target_th_eu[3:], rad=False)]

    return g_target_th_qu
