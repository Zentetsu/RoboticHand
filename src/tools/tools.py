import SofaRuntime
import Sofa.Gui
import Sofa

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
    node.gravity = [0, 0, -9810]
    node.dt = 0.01
    # node.addObject("GenericConstraintSolver", tolerance=1e-7, maxIterations=1000)
    node.addObject("CollisionPipeline")
    node.addObject("ParallelBruteForceBroadPhase")
    node.addObject("ParallelBVHNarrowPhase")
    node.addObject("CollisionResponse", response="FrictionContactConstraint", responseParams="mu=1")
    node.addObject("LocalMinDistance", name="Proximity", alarmDistance=2, contactDistance=1)

    # Inverse solver
    node.addObject("QPInverseProblemSolver", maxIterations=10000, tolerance=1e-6, epsilon=0.001, printLog=False)

    if ground:
        addGround(node, path)

def addGround(node, path) -> None:
    print("Add ground...")

    ground = node.addChild("Ground")

    ground.addObject("MeshOBJLoader", name="loader", filename=path + "Ground.obj", rotation=[270, 0, 0], scale=1, translation=[0, 0, 0])
    ground.addObject("MeshTopology", src="@loader")
    ground.addObject("MechanicalObject", src="@loader")
    ground.addObject("TriangleCollisionModel")
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
