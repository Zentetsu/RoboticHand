import SofaRuntime
import Sofa.Gui
import Sofa

from scipy.spatial.transform import Rotation as R
import numpy as np
import math

USE_GUI = True


def addPlugins(node) -> None:
    print("Add plugins...")

    node.addObject("RequiredPlugin", pluginName=[
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
        "Sofa.Component.SolidMechanics.Spring",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.Mapping.NonLinear",
        "Sofa.Component.Mapping.Linear",
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
    ])

def initScene(node, path, ground=False) -> None:
    print("Init scene...")

    node.addObject("FreeMotionAnimationLoop")
    node.addObject("VisualStyle", displayFlags="showVisualModels showBehaviorModels showInteractionForceFields", bbox=[-1,-1,-1,1,1,1])

    # Collision
    node.gravity = [0, 0, -9.810]
    node.dt = 0.01
    # node.addObject("GenericConstraintSolver", tolerance=1e-7, maxIterations=1000)
    node.addObject("CollisionPipeline")
    node.addObject("ParallelBruteForceBroadPhase")
    node.addObject("ParallelBVHNarrowPhase")
    node.addObject("LocalMinDistance", name="Proximity", alarmDistance=0.2, contactDistance=0.09, angleCone=0.0)
    node.addObject("CollisionResponse", response="FrictionContactConstraint", responseParams="mu=0.6")
    node.addObject('NewProximityIntersection', alarmDistance=0.6, contactDistance=0.3)
    # node.addObject('MinProximityIntersection', alarmDistance=1.0, contactDistance=0.5)

    # Inverse solver

    # node.addObject("LCPConstraintSolver", maxIt=1000, tolerance=0.001)
    node.addObject("QPInverseProblemSolver", maxIterations=10000, tolerance=0.001, epsilon=0.001, printLog=False)

    if ground:
        addGround(node, path)

def addGround(node, path) -> None:
    print("Add ground...")

    ground = node.addChild("Ground")

    ground.addObject("MeshOBJLoader", name="loader", filename=path + "Ground.obj", rotation=[270, 0, 0], scale=1, translation=[0, 0, 0])
    ground.addObject("MeshTopology", src="@loader")
    ground.addObject("MechanicalObject", src="@loader")
    ground.addObject("TriangleCollisionModel", moving=0, simulated=0)
    ground.addObject("LineCollisionModel")
    ground.addObject("PointCollisionModel")
    ground.addObject("OglModel", name="Visual", src="@loader", color=[0.5, 0.5, 0.5, 1])

def launchGUI(root) -> None:
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("Scene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()

def degToQuat(euler_angles):
    return R.from_euler('xyz', euler_angles, degrees=True).as_quat()

def getForearmFromHand(palm_coords, local_translation_y):
    x, y, z = palm_coords[0:3]
    qx, qy, qz, qw = degToQuat([palm_coords[3], palm_coords[4], palm_coords[5]])

    T_y = local_translation_y
    t_local = np.array([0, T_y, 0])

    rotation = R.from_quat([qx, qy, qz, qw])
    R_forearm = rotation.as_matrix()

    t_global = R_forearm.dot(t_local)
    p_forearm = np.array([x, y, z])
    p_wrist = p_forearm + t_global

    return p_wrist[0], p_wrist[1], p_wrist[2], qx, qy, qz, qw

def eulerRotationMatrix(roll, pitch, yaw):
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx

def matrixOriginToWrist(wrist_pos, wrist_ori):
    R = eulerRotationMatrix(*wrist_ori)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = wrist_pos

    return T

def invertMatrix(T):
    R = T[:3, :3]
    t = T[:3, 3]

    R_inv = R.T
    t_inv = -R_inv @ t

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv

def coEulerToQuat(c_wridt, c_finger):
    matrix = matrixOriginToWrist(c_wridt[:3], c_wridt[3:])

    g_target_th_po = matrix @ np.array([*c_finger[:3], 1])
    g_eu = R.from_matrix(matrix[:3, :3] @ eulerRotationMatrix(*c_finger[3:])).as_euler('xyz', degrees=True)
    g_target_th_eu = [*g_target_th_po[:3], *g_eu]
    g_target_th_qu = [*g_target_th_eu[:3], *degToQuat(g_target_th_eu[3:])]

    return g_target_th_qu
