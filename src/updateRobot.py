from pymycobot import MyArmM
from IRONbark import Module
import time


if __name__ == "__main__":
    myarmm = MyArmM("/dev/tty.usbserial-58690034881", 1000000)
    speed = 2

    robot_Module = Module(file="./data/Robot_Module.json")
    angles = [0] * 7

    while robot_Module.getLSAvailability(listener=True)[1][0]:
        time.sleep(0.1)
        print(robot_Module["sofa"]["angles"])
        n_angles = robot_Module["sofa"]["angles"]
        angles[0] = -n_angles[0]
        angles[1] = n_angles[1]
        angles[2] = n_angles[2]
        angles[3] = -n_angles[3]
        angles[4] = -n_angles[4]
        angles[5] = -n_angles[5]
        angles[6] = -75
        print(angles)

        myarmm.set_joints_angle(angles, speed)

    robot_Module.stopModule(name="robot")