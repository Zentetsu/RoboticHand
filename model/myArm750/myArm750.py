import roboticstoolbox as rtb
from spatialmath import SO3, SE3
from scipy.spatial.transform import Rotation as R
import math
import os

links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(
    os.environ["PHDPATH"] + "RoboticHand/model/hand/thumb.URDF"
)

robot = rtb.Robot(
    links,
    name=name,
    urdf_string=urdf_string,
    urdf_filepath=urdf_filepath,
)

q0 = [
    math.radians(0),  # Wrist link
    math.radians(45),  # Meta 2
    math.radians(0),  # Meta 1 3
    math.radians(0),  # Index 1 3
    math.radians(10),  # index 2
    math.radians(-90),  # index 3
    math.radians(-30),  # index 4
]

q0 = [
    math.radians(-45),  # Thumb 1 3
    math.radians(-20),  # Thumb 2 3
    math.radians(-30),  # Thumb 3
    math.radians(70),  # Thumb 4
    math.radians(45),  # Thumb 5
]

# for _ in range(0, 50):
T = robot.fkine(q0)

# T = SE3(-0.08098404,  0.23265317, -0.02936324) * \
#     SE3.Rx(6.42373888, "deg") * \
#     SE3.Ry(68.00529557, "deg") * \
#     SE3.Rz(-27.26857766, "deg")
print(T)


# sol = robot.ikine_GN(T, q0=q0)
# sol = robot.ikine_LM(T, q0=q0)
# sol = robot.ikine_NR(T, q0=q0)
sol = robot.ikine_QP(T, q0=q0)
print(sol.success, sol.q)
# q0 = sol.q

rotation_matrix = T.R
rotation = R.from_matrix(rotation_matrix)
euler_angles = rotation.as_euler("xyz", degrees=True)
print("Euler angles (degrees):", euler_angles)
