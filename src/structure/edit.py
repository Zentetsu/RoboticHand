import SofaRuntime
import Sofa


def addPart(node, name, index, filename, translation=[0, 0, 0], rotation=[0, 0, 0], color=[1, 0, 0, 1], collision=False) -> None:
    part = node.addChild(name)

    part.addObject("MeshSTLLoader", name=name, filename=filename, translation=translation)
    part.addObject("MechanicalObject", template="Rigid3", position=[0,0,0,0,0,0,1])
    part.addObject("RigidMapping", index=index, globalToLocalCoords=True)

    if collision:
        addCollision(part, name)

    addVisu(part, name, index, translation, rotation, color)

def addVisu(node, name, index, translation, rotation, color) -> None:
    visu = node.addChild("Visu_"+str(index))

    visu.addObject("OglModel", name="VisualModel", src="@../" + name, color=color, translation=translation, rotation=rotation)
    visu.addObject("RigidMapping")

def addCenter(node, name, parentIndex, childIndex, posOnParent, posOnChild, articulationProcess, isTranslation, isRotation, axis, articulationIndex):
    center = node.addChild(name)
    center.addObject("ArticulationCenter", parentIndex=parentIndex, childIndex=childIndex, posOnParent=posOnParent, posOnChild=posOnChild, articulationProcess=articulationProcess)

    articulation = center.addChild("Articulation")
    articulation.addObject("Articulation", translation=isTranslation, rotation=isRotation, rotationAxis=axis, articulationIndex=articulationIndex)

def addCollision(node, name) -> None:
    collision = node.addChild("Collision")

    collision.addObject("MeshTopology", src="@../" + name)
    collision.addObject("MechanicalObject")
    collision.addObject("TriangleCollisionModel")
    collision.addObject("LineCollisionModel")
    collision.addObject("PointCollisionModel")
    collision.addObject("RigidMapping")
