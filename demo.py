# demo.py
from pymycobot import MyArmM
import time

# Initiate MyCobot
# Create object code here for windows version
myarmm = MyArmM("/dev/tty.usbserial-58690034881", 1000000)

print(myarmm.get_joints_angle())
print(myarmm.get_robot_power_status())

angles = [
    -7.58, # to invert
    0.1,
    -13.47,
    -27.59, # to invert
    15.45, # to invert
    27.56, # to invert
    -75.0,
]

speed = 2

myarmm.set_joints_angle(angles, speed)

# i = 7
# #loop 7 times
# while i > 0:
#     myarmm.set_tool_led_color(0, 0, 255)  # blue light on
#     time.sleep(2)  # wait for 2 seconds
#     myarmm.set_tool_led_color(255, 0, 0)  #red light on
#     time.sleep(2)  # wait for 2 seconds
#     myarmm.set_tool_led_color(0, 255, 0)  #green light on
#     time.sleep(2)  # wait for 2 seconds
#     i -= 1