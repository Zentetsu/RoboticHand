import SofaRuntime # type: ignore
import Sofa # type: ignore

from scipy.spatial.transform import Rotation as R
import numpy as np
import math
import copy

from structure import BasicStructure


class ComplexStructure(BasicStructure):
    def __init__(self, node, path, name="ComplexStructure", translation=[0, 0, 0], rotation=[0, 0, 0], positions=None, init_angles=None, visu_info=None):
        print("Init " + name + "...")

        super().__init__(node, path, name, translation, rotation, positions, init_angles, visu_info)

    def createStructure(self, solver=None, constraint=False) -> None:
        super().createStructure(solver, constraint=constraint)

        self.structure.addData("angles", self.init_angles, None, self.name + " angles", "", "vector<float>")

    def createArticulation(self, joint_limit=False) -> None:
        print("Create " + self.name + " articulation...")

        if self.structure is None:
            print("WARNING: Need to call createStructure first.")
            return

        self.articulation = self.structure.addChild("Articulation")

        self.articulation.addObject("MechanicalObject", name="dofs", template="Vec1", rest_position=self.structure.getData("angles").getLinkPath(), position=self.init_angles)
        self.articulation.addObject("ArticulatedHierarchyContainer")
        self.articulation.addObject("UniformMass", totalMass=45) #TODO: Check with obj weight

        if joint_limit:
            for joint in self.joint_actuator:
                self.articulation.addObject("JointActuator", name="joint_" + str(joint[0]), index=joint[0], maxAngleVariation=0.001, minAngle=joint[1], maxAngle=joint[2])

        self.articulation.addObject("RestShapeSpringsForceField", stiffness=1e5, points=[i for i in range(0, len(self.init_angles))]) #TODO: Check for inverse solver

    def createRigid(self) -> None:
        print("Create " + self.name + " rigid...")

        self.rigid = self.articulation.addChild("Rigid")

        self.mo = self.rigid.addObject("MechanicalObject", name="dofs", template="Rigid3", showObject=True, showObjectScale=10, position=self.positions[0:len(self.positions)], translation=self.translation, rotation=self.rotation)
        self.rigid.addObject("ArticulatedSystemMapping", input1=self.articulation.dofs.getLinkPath(), output=self.rigid.dofs.getLinkPath())

    def createVisualization(self, collision=False) -> None:
        print("Create " + self.name + " visualization...")

        if self.rigid is None:
            self.visu = self.structure.addChild("Visu")
        else:
            self.visu = self.rigid.addChild("Visu")

        super().createVisualization(collision)

    def inverseControl(self, target_position) -> None:
        print("Defining " + self.name + " goal...")

        self.target = [None] * len(target_position)

        for i, target in enumerate(target_position):
            if target is None:
                continue

            self.target[i] = self.node.addChild(self.name + "_EffectorTarget_" + str(i))

            self.target[i].addObject("EulerImplicitSolver", firstOrder=True)
            self.target[i].addObject("CGLinearSolver", iterations=100, threshold=1e-2, tolerance=1e-5)
            self.target[i].addObject("MechanicalObject", name="dofs", template="Rigid3", position=target, showObject=1, showObjectScale=10, drawMode=1)
            self.target[i].addObject("UncoupledConstraintCorrection", defaultCompliance="0.1")

            self.rigid.addObject("PositionEffector", name="pe_" + str(i), template="Rigid3", indices=self.indice[i], effectorGoal=self.target[i].dofs.findData('position').getLinkPath(), useDirections=[1, 1, 1, 1, 1, 1])

    def updateAngle(self, n_angles):
        self.structure.angles = np.deg2rad(n_angles).tolist()

    def updatePosition(self, n_pos):
        self.mo.position = n_pos

    def updateTargetPosition(self, target_id, n_target_position):
        if self.target[target_id] is None:
            return

        self.target[target_id].dofs.findData("position").setData = [n_target_position]

    @staticmethod
    def addCenter(node, name, parentIndex, childIndex, posOnParent, posOnChild, articulationProcess, isTranslation, isRotation, axis, articulationIndex):
        center = node.addChild(name)
        center.addObject("ArticulationCenter", parentIndex=parentIndex, childIndex=childIndex, posOnParent=posOnParent, posOnChild=posOnChild, articulationProcess=articulationProcess)

        articulation = center.addChild("Articulation")
        articulation.addObject("Articulation", translation=isTranslation, rotation=isRotation, rotationAxis=axis, articulationIndex=articulationIndex)
