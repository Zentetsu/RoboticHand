import SofaRuntime
import Sofa
import math
import os

from scipy.spatial.transform import Rotation as R
import numpy as np


def addPlugins(node) -> None:
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


def createScene(root) -> None:
    addPlugins(root)
    root.addObject(
        "VisualStyle",
        displayFlags="showVisualModels showBehaviorModels showInteractionForceFields",
        bbox=[-1, -1, -1, 1, 1, 1],
    )  #  showCollisionModels
    root.addObject("FreeMotionAnimationLoop")

    # Collision
    root.gravity = [0, 0, -9810.0]
    root.dt = 0.01
    # root.addObject("GenericConstraintSolver", tolerance=1e-7, maxIterations=1000)
    root.addObject("CollisionPipeline")
    root.addObject("ParallelBruteForceBroadPhase")
    root.addObject("ParallelBVHNarrowPhase")
    root.addObject(
        "LocalMinDistance", name="Proximity", alarmDistance=5, contactDistance=0.5
    )
    root.addObject(
        "CollisionResponse", response="FrictionContactConstraint", responseParams="mu=1"
    )
    # root.addObject('RuleBasedContactManager', name="Response", response="FrictionContactConstraint", rules="default")
    root.addObject("NewProximityIntersection", alarmDistance=5, contactDistance=0.5)
    # root.addObject('MinProximityIntersection', alarmDistance=0.5, contactDistance=0.2)

    # Inverse solver

    # root.addObject("LCPConstraintSolver", maxIt=1000, tolerance=0.1)
    root.addObject(
        "QPInverseProblemSolver",
        maxIterations=10000,
        tolerance=1e-2,
        epsilon=1e-2,
        printLog=False,
    )

    ground = root.addChild("ground")
    ground.addObject(
        "MeshSTLLoader",
        name="loader",
        filename=os.environ["PHDPATH"] + "/RoboticHand/model/Others/Ground.stl",
        rotation=[0, 0, 0],
        scale=1,
        translation=[0, 0, 0],
    )
    ground.addObject("MeshTopology", src="@loader")
    ground.addObject("MechanicalObject", src="@loader")
    ground.addObject("TriangleCollisionModel", moving=0, simulated=0)
    ground.addObject("LineCollisionModel", moving=0, simulated=0)
    ground.addObject("PointCollisionModel", moving=0, simulated=0)
    ground.addObject("OglModel", name="Visual", src="@loader", color=[0.5, 0.5, 0.5, 1])

    sim = root.addChild("sim")
    # sim.addObject("LCPConstraintSolver", maxIt=1000, tolerance=0.001)

    cube = sim.addChild("cube")
    cube.addObject("EulerImplicitSolver", rayleighStiffness=1e-1, rayleighMass=1e-1)
    cube.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
    cube.addObject(
        "MechanicalObject",
        name="Rigid_DOF",
        template="Rigid3",
        position=[100, 0, 50, 0, 0, 0, 1],
    )
    cube.addObject("UniformMass", totalMass=500)
    cube.addObject("UncoupledConstraintCorrection")  # , linearSolver="@../Solver")

    cube.addObject(
        "MeshSTLLoader",
        name="cube",
        filename=os.environ["PHDPATH"] + "/RoboticHand/model/Others/Cube.stl",
        scale=1,
    )

    visu = cube.addChild("Visu_")
    visu.addObject(
        "OglModel",
        name="VisualModel",
        src="@../cube",
        translation=[0, 0, 0],
        rotation=[0, 0, 0],
        scale=1,
    )
    visu.addObject("RigidMapping", input="@../Rigid_DOF", output="@VisualModel")

    collision = cube.addChild("Collision")
    collision.addObject("MeshTopology", src="@../cube")
    collision.addObject("MechanicalObject", name="Collision_RM")
    collision.addObject("TriangleCollisionModel")
    collision.addObject("LineCollisionModel")
    collision.addObject("PointCollisionModel")
    collision.addObject("RigidMapping", input="@../Rigid_DOF", output="@Collision_RM")
    # collision.addObject("RestShapeSpringsForceField", stiffness=1e10)

    structure = None
    articulation = None
    rigid = None
    visu = None
    collision = None
    indice = 0

    positions = [
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1],
    ]

    init_angles = [
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
    ]

    visu_info = [
        (0, "Joint_0", "R0", [0, 0, 0], [0, 0, 90], False),
        (1, "Joint_1", "R1", [0, 0, 66.9], [0, 0, 90], False),
        (2, "Joint_2", "R2", [0, 57, 303], [0, 0, 90], False),
        (3, "Joint_3", "R3", [0, 82.53, 0], [0, 0, 90], False),
        (4, "Joint_4", "R4", [0, 245.38, 0], [0, 0, 90], False),
        (5, "Joint_5", "R5", [0, 79, 0], [0, 0, 90], False),
    ]

    joint_actuator = [
        (0, math.radians(-90), math.radians(90)),  # Joint 0
        (1, math.radians(-90), math.radians(90)),  # Joint 1
        (2, math.radians(-90), math.radians(90)),  # Joint 2
        (3, math.radians(-90), math.radians(90)),  # Joint 3
        (4, math.radians(-90), math.radians(90)),  # Joint 4
        (5, math.radians(-90), math.radians(90)),  # Joint 5
    ]

    articulation_info = [
        ("R0", 0, 1, [0, 0, 107], [0, 0, 0], 1, [0, 0, 1], 0),
        ("R1", 1, 2, [0, 0, 66.9], [0, 0, 0], 1, [1, 0, 0], 1),
        ("R2", 2, 3, [0, 57, 303], [0, 0, 0], 1, [1, 0, 0], 2),
        ("R3", 3, 4, [0, 82.53, 0], [0, 0, 0], 1, [0, 1, 0], 3),
        ("R4", 4, 5, [0, 245.38, 0], [0, 0, 0], 1, [1, 0, 0], 4),
        ("R5", 5, 6, [0, 79, 0], [0, 0, 0], 1, [0, 1, 0], 5),
    ]

    structure = sim.addChild("arm")
    structure.addObject(
        "EulerImplicitSolver", rayleighStiffness=1e-2, rayleighMass=1e-2
    )
    structure.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
    structure.addObject("GenericConstraintCorrection")
    structure.addData("angles", init_angles, None, "robot angles", "", "vector<float>")

    articulation = structure.addChild("Articulation")
    articulation.addObject(
        "MechanicalObject",
        name="dofs",
        template="Vec1",
        rest_position=structure.getData("angles").getLinkPath(),
        position=init_angles,
    )
    articulation.addObject("ArticulatedHierarchyContainer")
    articulation.addObject("UniformMass", totalMass=4500)
    for joint in joint_actuator:
        articulation.addObject(
            "JointActuator",
            name="joint_" + str(joint[0]),
            index=joint[0],
            maxAngleVariation=0.001,
            minAngle=joint[1],
            maxAngle=joint[2],
        )
    articulation.addObject(
        "RestShapeSpringsForceField",
        stiffness=1e3,
        points=[i for i in range(0, len(init_angles))],
    )

    rigid = articulation.addChild("Rigid")
    mo = rigid.addObject(
        "MechanicalObject",
        name="dofs",
        template="Rigid3",
        showObject=True,
        showObjectScale=10,
        position=positions[0 : len(positions)],
    )
    rigid.addObject(
        "ArticulatedSystemMapping",
        input1=articulation.dofs.getLinkPath(),
        output=rigid.dofs.getLinkPath(),
    )

    visu = rigid.addChild("Visu")
    complx = True

    path = os.environ["PHDPATH"] + "/RoboticHand/model/myArm750/"
    for i, info in enumerate(visu_info):
        print(info[5])
        addPart(
            visu,
            info[1],
            info[0],
            path + info[2] + ".stl",
            positions[i],
            info[3],
            info[4],
            rigid=rigid is not None,
            collision=info[5],
            complx=complx,
        )

    centers = articulation.addChild("ArticulationCenters")

    for i, articullation in enumerate(articulation_info):
        addCenter(
            centers,
            articullation[0],
            articullation[1],
            articullation[2],
            articullation[3],
            articullation[4],
            0,
            0,
            articullation[5],
            articullation[6],
            articullation[7],
        )

    indice = [len(articulation_info)]

    target_wrist = [0, 500, 300, 0, 0, 0]
    target_position = getForearmFromHand(target_wrist, -48)
    target_position = [
        target_position,
    ]
    print(target_position, len(target_position))
    target = [None] * 1

    for i, tt in enumerate(target_position):
        if target is None:
            continue

        print(
            target,
            i,
        )
        target[i] = sim.addChild("_EffectorTarget_")

        target[i].addObject("EulerImplicitSolver", firstOrder=True)
        target[i].addObject(
            "CGLinearSolver", iterations=100, threshold=1e-2, tolerance=1e-5
        )
        target[i].addObject(
            "MechanicalObject",
            name="dofs",
            template="Rigid3",
            position=tt,
            showObject=1,
            showObjectScale=10,
            drawMode=1,
        )
        target[i].addObject("UncoupledConstraintCorrection", defaultCompliance="0.1")

        rigid.addObject(
            "PositionEffector",
            name="pe_" + str(i),
            template="Rigid3",
            indices=indice[i],
            effectorGoal=target[i].dofs.findData("position").getLinkPath(),
            useDirections=[1, 1, 1, 1, 1, 1],
        )


def addPart(
    node,
    name,
    index,
    filename,
    position,
    translation=[0, 0, 0],
    rotation=[0, 0, 0],
    color=[0, 0, 1, 1],
    rigid=False,
    collision=False,
    complx=False,
) -> None:
    print("Add part " + name + "...")

    if complx:
        part = node.addChild(name)
    else:
        part = node

    addMesh(part, name, filename)

    if complx:
        part.addObject("MechanicalObject", template="Rigid3", position=position)
        part.addObject("RigidMapping", index=index, globalToLocalCoords=False)

    addVisu(part, name, index, translation, rotation, color, rigid, complx)

    if collision:
        addCollision(part, name, complx)


def addVisu(node, name, index, translation, rotation, color, rigid, complx) -> None:
    print("Add visu " + name + "...")

    visu = node.addChild("Visu_" + str(index))

    visu.addObject(
        "OglModel",
        name="VisualModel",
        src="@../" + name,
        color=color,
        translation=translation,
        rotation=rotation,
        scale=1,
    )

    if rigid and not complx:
        visu.addObject("RigidMapping", input="@../Rigid_DOF", output="@VisualModel")
    elif rigid:
        visu.addObject("RigidMapping", output="@VisualModel")


def addCollision(node, name, complx) -> None:
    print("Add collision " + name + "...")

    collision = node.addChild("Collision")

    collision.addObject("MeshTopology", src="@../" + name)
    collision.addObject("MechanicalObject", name="Collision_RM")
    collision.addObject("TriangleCollisionModel")
    collision.addObject("LineCollisionModel")
    collision.addObject("PointCollisionModel")
    if complx:
        collision.addObject("RigidMapping")
    else:
        collision.addObject(
            "RigidMapping", input="@../Rigid_DOF", output="@Collision_RM"
        )


def addMesh(node, name, filename) -> None:
    print("Add mesh " + name + "...")

    if filename[-3 : len(filename)] == "stl":
        node.addObject("MeshSTLLoader", name=name, filename=filename, scale=1)
    else:
        node.addObject("MeshOBJLoader", name=name, filename=filename, scale=1)


def addCenter(
    node,
    name,
    parentIndex,
    childIndex,
    posOnParent,
    posOnChild,
    articulationProcess,
    isTranslation,
    isRotation,
    axis,
    articulationIndex,
):
    center = node.addChild(name)
    center.addObject(
        "ArticulationCenter",
        parentIndex=parentIndex,
        childIndex=childIndex,
        posOnParent=posOnParent,
        posOnChild=posOnChild,
        articulationProcess=articulationProcess,
    )

    articulation = center.addChild("Articulation")
    articulation.addObject(
        "Articulation",
        translation=isTranslation,
        rotation=isRotation,
        rotationAxis=axis,
        articulationIndex=articulationIndex,
    )


def degToQuat(euler_angles):
    return R.from_euler("xyz", euler_angles, degrees=True).as_quat()


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
