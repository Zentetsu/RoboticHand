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
    ])

def addGround(rootNode):
    ground = rootNode.addChild('ground')
    ground.addObject("MeshSTLLoader", name="loader", filename="Ground.stl", rotation=[0, 0, 0], scale=1, translation=[0, 0, 0])
    ground.addObject("MeshTopology", src="@loader")
    ground.addObject("MechanicalObject", src="@loader")
    ground.addObject("TriangleCollisionModel") #, moving=0, simulated=0)
    ground.addObject("LineCollisionModel") #, moving=0, simulated=0)
    ground.addObject("PointCollisionModel") #, moving=0, simulated=0)
    ground.addObject("OglModel", name="Visual", src="@loader", color=[0.5, 0.5, 0.5, 1])

def createScene(rootNode):
    addPlugins(rootNode)

    rootNode.addObject("FreeMotionAnimationLoop")
    rootNode.addObject("VisualStyle", displayFlags="showVisualModels showBehaviorModels showInteractionForceFields", bbox=[-1, -1, -1, 1, 1, 1])

    rootNode.gravity = [0, 0, -9810.0]
    rootNode.dt = 0.001

    rootNode.addObject("CollisionPipeline")
    rootNode.addObject('ParallelBruteForceBroadPhase')
    rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('CollisionResponse', response="FrictionContactConstraint", responseParams="mu=1.5")
    rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance=5, contactDistance=0.1)

    rootNode.addObject("QPInverseProblemSolver", maxIterations=10000, tolerance=1e-2, epsilon=1e-4, printLog=False)

    addGround(rootNode)

    tran = [-125, 350, 26]
    rota = [0, 0, 0]

    structure = rootNode.addChild("cube")
    structure.addObject("EulerImplicitSolver", rayleighStiffness=1e-4, rayleighMass=1e-4)
    structure.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")


    structure.addObject('MeshGmshLoader', name="cube_def", filename="Cube.msh", translation=tran, rotation=rota, scale=1)
    structure.addObject("MeshTopology", src="@cube_def")
    structure.addObject("MechanicalObject", template="Vec3")
    structure.addObject('ParallelTetrahedronFEMForceField', poissonRatio=0.1, youngModulus=50)
    # structure.addObject('LinearSolverConstraintCorrection')
    structure.addObject("UniformMass", totalMass=0.005)
    structure.addObject("UncoupledConstraintCorrection")
    structure.addObject("MeshSTLLoader", name="cube", filename="Cube.stl", scale=1)


    visu = structure.addChild('visu')
    visu.addObject("OglModel", name="VisualModel", src="@../cube", translation=tran, rotation=rota, scale=1)
    visu.addObject('BarycentricMapping')


    collision = structure.addChild('collision')
    collision.addObject("MeshTopology", src="@../cube")
    collision.addObject("MechanicalObject", name="Collision_RM", translation=tran, rotation=rota)
    collision.addObject("TriangleCollisionModel")
    collision.addObject("LineCollisionModel")
    collision.addObject("PointCollisionModel")
    collision.addObject('BarycentricMapping')

    return rootNode
