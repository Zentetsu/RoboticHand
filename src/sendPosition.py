from IRONbark import Module

from scipy.signal import butter, lfilter
from filterpy.kalman import KalmanFilter
from bleak import BleakClient
import numpy as np
import threading
import asyncio
import signal
import time
import ast


# IMU_ADD = "118FD1F0-FC55-3456-24AD-53815F5F3946"
# UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
# GRAVITY = 9.81

# ACC_SCALE = 16 * GRAVITY / 32768.0  # 16g range
# GYRO_SCALE = 2000 / 32768.0         # ±2000°/s range
# ORIENT_SCALE = 180 / 32768.0        # Orientation in degrees

# last_time = None
# frequencies = []

# # Filter setup (optional)
# def butter_lowpass(cutoff, fs, order=5):
#     nyquist = 0.5 * fs
#     normal_cutoff = cutoff / nyquist
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return b, a

# def lowpass_filter(data, cutoff=5, fs=10, order=5):  # Updated cutoff frequency for 10 Hz
#     b, a = butter_lowpass(cutoff, fs, order=order)
#     return lfilter(b, a, data)

# def highpass_filter(data, cutoff=0.5, fs=10, order=1):
#     nyquist = 0.5 * fs
#     normal_cutoff = cutoff / nyquist
#     b, a = butter(order, normal_cutoff, btype='high', analog=False)
#     return lfilter(b, a, data)

# alpha = 0.98  # Tuning parameter (between 0 and 1)
# dt = 0.1      # Sampling time (in seconds)

# # Initialize angles
# roll_estimated = 0.0
# pitch_estimated = 0.0
# yaw_estimated = 0.0

# def complementary_filter(accel_data, gyro_data):
#     global roll_estimated, pitch_estimated, yaw_estimated

#     ax, ay, az = accel_data
#     wx, wy, wz = gyro_data

#     roll_acc = np.arctan2(ay, az) * (180 / np.pi)
#     pitch_acc = np.arctan2(-ax, np.sqrt(ay**2 + az**2)) * (180 / np.pi)

#     roll_estimated = alpha * (roll_estimated + wx * dt) + (1 - alpha) * roll_acc
#     pitch_estimated = alpha * (pitch_estimated + wy * dt) + (1 - alpha) * pitch_acc
#     yaw_estimated += wz * dt  # Assuming yaw is not directly measured from accelerometer

#     return roll_estimated, pitch_estimated, yaw_estimated

# # Kalman Filter variables
# q_angle = 0.001  # Process noise variance for the accelerometer
# q_bias = 0.003   # Process noise variance for the gyro bias
# r_measure = 0.03 # Measurement noise variance

# # Kalman filter variables
# roll_kalman = 0.0
# pitch_kalman = 0.0
# roll_bias = 0.0
# pitch_bias = 0.0
# P = np.array([[1, 0], [0, 1]])  # Estimate error covariance matrix

# def kalman_filter(roll_acc, pitch_acc, wx, wy):
#     global roll_kalman, pitch_kalman, roll_bias, pitch_bias, P

#     # Time update (Predict)
#     dt = 0.1  # Assuming a sample rate of 10 Hz
#     roll_kalman += (wx - roll_bias) * dt
#     pitch_kalman += (wy - pitch_bias) * dt

#     # Update error covariance
#     P[0, 0] += q_angle
#     P[1, 1] += q_bias

#     # Measurement update
#     S = P[0, 0] + r_measure  # Measurement update (innovation covariance)
#     K = P[0, 0] / S  # Kalman Gain

#     # Update estimates with the new measurement
#     roll_kalman += K * (roll_acc - roll_kalman)
#     pitch_kalman += K * (pitch_acc - pitch_kalman)

#     # Update error covariance matrix
#     P[0, 0] -= K * P[0, 0]
#     P[1, 1] -= K * P[1, 1]

#     return roll_kalman, pitch_kalman

# def correct_gravity(ax, ay, az):
#     g = 9.81  # Gravity constant
#     roll_rad = np.radians(roll_estimated)
#     pitch_rad = np.radians(pitch_estimated)

#     gravity_x = g * np.sin(pitch_rad)
#     gravity_y = g * np.sin(roll_rad) * np.cos(pitch_rad)
#     gravity_z = g * np.cos(roll_rad) * np.cos(pitch_rad)

#     # Correct the accelerometer readings
#     ax_corrected = ax - gravity_x
#     ay_corrected = ay - gravity_y
#     az_corrected = az - gravity_z

#     return ax_corrected, ay_corrected, az_corrected

# # Initialize variables
# vx = 0.0
# vy = 0.0
# vz = 0.0
# dx = 0.0
# dy = 0.0
# dz = 0.0

# # Time step (seconds) based on the IMU sampling rate
# dt = 0.1  # For a 10Hz sampling rate

# def notification_handler(sender, data: bytearray):
#     global last_time, vx, vy, vz, dx, dy, dz

#     decoded = [int.from_bytes(data[i:i+2], byteorder='little', signed=True) for i in range(2, len(data), 2)]
#     ax    = decoded[0] * ACC_SCALE
#     ay    = decoded[1] * ACC_SCALE
#     az    = decoded[2] * ACC_SCALE
#     wx    = decoded[3] * GYRO_SCALE
#     wy    = decoded[4] * GYRO_SCALE
#     wz    = decoded[5] * GYRO_SCALE
#     roll  = decoded[6] * ORIENT_SCALE
#     pitch = decoded[7] * ORIENT_SCALE
#     yaw   = decoded[8] * ORIENT_SCALE

#     # Get current time and calculate the interval
#     current_time = time.time()
#     if last_time is not None:
#         dt = current_time - last_time
#         frequency = 1 / dt
#         if 100 < frequency < 300:  # Filter out unrealistic values
#             frequencies.append(frequency)
#             print(f"Sampling Frequency: {frequency:.2f} Hz")
#     last_time = current_time

#     # Apply filtering (optional)
#     # print(f"OLD ax: {ax:.3f}, ay: {ay:.3f}, az: {az:.3f}, wx: {wx:.3f}, wy: {wy:.3f}, wz: {wz:.3f}, roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}")
#     # ax, ay, az = lowpass_filter([ax, ay, az], cutoff=2, fs=10)
#     # ax, ay, az = highpass_filter([ax, ay, az])
#     # wx, wy, wz = lowpass_filter([wx, wy, wz], cutoff=1, fs=10)
#     # ax, ay, az = correct_gravity(ax, ay, az)

# #     roll = np.arctan2(ay, az) * (180 / np.pi)
# #     pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2)) * (180 / np.pi)

# #     roll, pitch, yaw = complementary_filter((ax, ay, az), (wx, wy, wz))
# #     roll, pitch = kalman_filter(roll, pitch, wx, wy)

# #    # Compensate for gravity (assuming z-axis is vertical)
# #     az -= GRAVITY  # Remove gravity from vertical acceleration

# #     # Integrate acceleration to get velocity
# #     vx += ax * dt
# #     vy += ay * dt
# #     vz += az * dt

# #     # Integrate velocity to get displacement
# #     dx += vx * dt
# #     dy += vy * dt
# #     dz += vz * dt

# #     print(f"NEW ax: {ax:.3f}, ay: {ay:.3f}, az: {az:.3f}, wx: {wx:.3f}, wy: {wy:.3f}, wz: {wz:.3f}, roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}")
# #     print(f"dx: {dx}, dy: {dy}, dz: {dz}")

# async def run(address, loop):
#     global loop_

#     async with BleakClient(address, loop=loop) as client:
#         await client.start_notify(UUID, notification_handler)

#         while client.is_connected and loop_:
#             # print(client.is_connected)
#             await asyncio.sleep(0.1)

def handler(signum, frame):
    global loop_
    loop_ = False


from pynput.keyboard import Listener, Key
import time

class ControllerKB:
    def __new__(cls, verbose=False):
        if not hasattr(cls, 'instance') or not cls.instance:
            cls.instance = super().__new__(cls)
            cls.instance.ControllerKB(verbose)

        return cls.instance

    def ControllerKB(self, verbose):
        self.verbose = verbose

        self.command = {'w': False, 's': False, 'a': False, 'd': False, 'p': False, 'shift': False, 'alt': False, 'esc': False, 'up': False, 'down': False, 'left': False, 'right': False}

    def readInput(self):
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        try:
            print(key)
            if key.char in self.command:
                self.command[key.char] = True
        except:
            if key == Key.esc:
                self.command['esc'] = True
                return False

            if key == key.shift:
                self.command['shift'] = True
            if key == key.alt:
                self.command['alt'] = True
            if key == key.up:
                self.command['up'] = True
            if key == key.down:
                self.command['down'] = True
            if key == key.left:
                self.command['left'] = True
            if key == key.right:
                self.command['right'] = True

    def on_release(self, key):
        try:
            if key.char in self.command:
                self.command[key.char] = False

        except:
            if key == key.shift:
                self.command['shift'] = False
            if key == key.alt:
                self.command['alt'] = False
            if key == key.up:
                self.command['up'] = False
            if key == key.down:
                self.command['down'] = False
            if key == key.left:
                self.command['left'] = False
            if key == key.right:
                self.command['right'] = False

    def getInput(self):
        return self.command


if __name__ == "__main__":
    # loop_ = True
    # signal.signal(signal.SIGINT, handler)

    # while loop_:
    #     try:
    #         loop = asyncio.get_event_loop()
    #         loop.run_until_complete(run(IMU_ADD, loop))
    #         loop.stop()
    #     except:
    #         print("fail")

    Target_Module = Module(file="./data/Target_Module.json")

    t = None

    cKB = ControllerKB()
    thread = threading.Thread(target=cKB.readInput, args=())
    thread.start()

    while not cKB.getInput()["esc"]:
        time.sleep(0.1)
        print(Target_Module["target"]["wrist"])

        old = Target_Module["target"]["wrist"]
        old2 = Target_Module["target"]["index"]

        if cKB.getInput()["w"]:
            old[0] += 1
        elif cKB.getInput()["s"]:
            old[0] -= 1
        elif cKB.getInput()["a"]:
            old[1] -= 1
        elif cKB.getInput()["d"]:
            old[1] += 1
        elif cKB.getInput()["shift"]:
            old[2] += 1
        elif cKB.getInput()["alt"]:
            old[2] -= 1

        if cKB.getInput()["up"]:
            old2[0] += 0.1
        elif cKB.getInput()["down"]:
            old2[0] -= 0.1
        elif cKB.getInput()["left"]:
            old2[1] -= 0.1
        elif cKB.getInput()["right"]:
            old2[1] += 0.1

        Target_Module["target"]["wrist"] = old
        Target_Module["target"]["index"] = old2

    Target_Module.stopModule()
