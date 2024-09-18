from structure import Arm, Hand, BasicStructure
from tools import *

import os


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

# def getFingerFromHand(wrist_coords, translation, rotation):
#     qx_f, qy_f, qz_f, qw_f = degToQuat(rotation)

#     rotation_wrist = R.from_quat(wrist_coords[3:])
#     R_wrist = rotation_wrist.as_matrix()

#     t_global = R_wrist.dot(translation)
#     p_wrist = np.array(wrist_coords[:3])
#     p_finger = p_wrist + t_global

#     qx = wrist_coords[6] * qx_f + wrist_coords[5] * qw_f + wrist_coords[4] * qz_f - wrist_coords[3] * qy_f
#     qy = wrist_coords[6] * qy_f - wrist_coords[5] * qz_f + wrist_coords[4] * qw_f + wrist_coords[3] * qx_f
#     qz = wrist_coords[6] * qz_f + wrist_coords[5] * qy_f - wrist_coords[4] * qx_f + wrist_coords[3] * qw_f
#     qw = wrist_coords[6] * qw_f - wrist_coords[5] * qx_f - wrist_coords[4] * qy_f - wrist_coords[3] * qz_f

#     q_finger = (qx, qy, qz, qw)

#     return p_finger, q_finger

# def fingerToWrist(finger, wrist):
#     p_finger_global = np.array(finger[:3])
#     p_wrist_global = np.array(wrist[:3])
#     p_finger_relative = p_finger_global - p_wrist_global

#     wrist_rotation = R.from_quat(wrist[3:])
#     wrist_rotation_inv = wrist_rotation.inv()

#     p_finger_wrist = wrist_rotation_inv.apply(p_finger_relative)

#     wrist_quat_inv = [-wrist[3+0], -wrist[3+1], -wrist[3+2], wrist[3+3]]

#     def quaternion_multiply(q1, q2):
#         x1, y1, z1, w1 = q1
#         x2, y2, z2, w2 = q2
#         return [
#             w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
#             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
#             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
#             w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
#         ]

#     finger_quat_in_wrist_space = quaternion_multiply(wrist_quat_inv, finger[3:])

#     return np.concatenate((p_finger_wrist, finger_quat_in_wrist_space), axis=None)

# def quaternionToEeuler(quat):
#     r = R.from_quat(quat)
#     euler_angles = r.as_euler('xyz', degrees=True)

#     return euler_angles

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

def createScene(root) -> None:
    path = os.environ['PHDPATH'] + "/RoboticHand/model/"

    addPlugins(root)
    initScene(root, path + "Others/", ground=True)

    sim = root.addChild("Simulation")

    # cube = BasicStructure(
    #     root,
    #     path + "Others/",
    #     "Cube",
    #     positions=[[-125, 350, 50, 0, 0, 0, 1]],
    #     visu_info=[(0, "Cube", "Cube", [0, 0, 0], [0, 0, 0], True)]
    # )
    # cube.createStructure(solver="SparseLDLSolver", constraint=True)
    # cube.createRigid(collision=True)
    # cube.createVisualization()

    target_wrist = [-100, 100 , 205, 0, 0, 45]
    target_forearm = getForearmFromHand(target_wrist, -48)

    arm = Arm(sim, path + "Arm/", "Arm")
    arm.createStructure(solver="SparseLDLSolver", constraint=True)
    arm.createArticulation()
    arm.createRigid()
    arm.createVisualization()
    arm.createArticulationCenter()
    arm.inverseControl([
        target_forearm,
    ])

    # matrix = matrixOriginToWrist(target_wrist[:3], target_wrist[3:])
    # invert_matrix = invertMatrix(matrix)

    # g_target_th_eu = [-50.0, 400.0, 230.0, 170, 25, -75]
    # g_target_th_qu = [*g_target_th_eu[:3], *degToQuat(g_target_th_eu[3:])]

    # w_target_th_eu = invert_matrix @ np.array([*g_target_th_eu[:3], 1])
    # w_target_th_eu = [*w_target_th_eu[:3], *g_target_th_eu[3:]]
    # print(w_target_th_eu)

    w_target_th_eu = [-50.0, 120.0, 25.0, 170, 25, -75]
    g_target_th_qu = coEulerToQuat(target_wrist, w_target_th_eu)

    w_target_in_eu = [-80.0, 170.0, 15.0, 180, 90, 0]
    g_target_in_qu = coEulerToQuat(target_wrist, w_target_in_eu)

    hand = Hand(sim, path + "Hand/files/", "Hand")
    hand.createStructure(solver="SparseLDLSolver", constraint=True)
    hand.createArticulation()
    hand.createRigid()
    hand.createVisualization()
    hand.createArticulationCenter()
    hand.attachToRobot()
    hand.inverseControl([
        g_target_th_qu,
        g_target_in_qu,
    ])

if __name__ == "__main__":
    root = Sofa.Core.Node("root")
    # root = None

    createScene(root)
    launchGUI(root)
