# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os
import numpy as np
import os
import tempfile
import pprint
import cv2
import time
import matplotlib.pyplot as plt

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()


t0 = time.time()
T = []
X = []
Y = []
Z = []
W = []
idx = 0

while True:
    idx = idx + 1
    state = client.getMultirotorState()
    responses = client.simGetImages([airsim.ImageRequest("high_res", airsim.ImageType.Scene, False, False)])
    response = responses[0]

    T.append(state.timestamp)
    X.append(state.kinematics_estimated.orientation.x_val)
    Y.append(state.kinematics_estimated.orientation.y_val)
    Z.append(state.kinematics_estimated.orientation.z_val)
    W.append(state.kinematics_estimated.orientation.w_val)

    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)

    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)

    # write to png
    airsim.write_png(os.path.normpath('../../Records Unreal/images/im' + str(idx) + '.png'), img_rgb)

    t = time.time()
    if (t-t0) > 10:
        break

plt.subplot(411)
plt.plot(T, X)
plt.subplot(412)
plt.plot(T, Y)
plt.subplot(413)
plt.plot(T, Z)
plt.subplot(414)
plt.plot(T, W)
plt.show()

print(idx)

# print(state.kinematics_estimated.position)
# print(state.kinematics_estimated.orientation)
# print(state.kinematics_estimated.linear_velocity)
# print(state.kinematics_estimated.angular_velocity)
# print(state.kinematics_estimated.linear_acceleration)

