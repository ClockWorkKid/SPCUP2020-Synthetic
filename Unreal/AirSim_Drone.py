# ready to run example: PythonClient/multirotor/hello_drone.py

import airsim
import numpy as np
import os
import glob
import time
import matplotlib.pyplot as plt
import pandas as pd

# ************************************** DIRECTORY HANDLE ************************************** #

dataset_path = 'D:/Sayeed/SPCUP2020/Records Unreal/'
name_prefix = 'data'
extension = '.csv'

os.chdir(dataset_path)
result = glob.glob('*{}'.format(extension))

last_file_idx = result[len(result)-1]
last_file_idx = last_file_idx.replace(name_prefix, '')
last_file_idx = last_file_idx.replace(extension, '')

new_dataset_idx = int(last_file_idx)+1
print("Dataset NO " + str(new_dataset_idx))

new_dataset_images_directory = dataset_path + name_prefix + str(new_dataset_idx)
new_dataset_csv_name = dataset_path + name_prefix + str(new_dataset_idx) + extension

print(new_dataset_images_directory)
print(new_dataset_csv_name)

if not os.path.exists(new_dataset_images_directory):
    os.makedirs(new_dataset_images_directory)

# ************************************** AIRSIM SIMULATION ************************************** #

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()


sim_seconds = 60

t0 = time.time()
T = []
accX = []
accY = []
accZ = []
angX = []
angY = []
angZ = []
oriX = []
oriY = []
oriZ = []
oriW = []
Cls = []
idx = 0

# Normal data acquisition
while True:
    state = client.getMultirotorState()
    responses = client.simGetImages([airsim.ImageRequest("high_res", airsim.ImageType.Scene, False, False)])
    response = responses[0]

    T.append(state.timestamp)
    accX.append(state.kinematics_estimated.linear_acceleration.x_val)
    accY.append(state.kinematics_estimated.linear_acceleration.y_val)
    accZ.append(state.kinematics_estimated.linear_acceleration.z_val)
    angX.append(state.kinematics_estimated.angular_velocity.x_val)
    angY.append(state.kinematics_estimated.angular_velocity.y_val)
    angZ.append(state.kinematics_estimated.angular_velocity.z_val)
    oriX.append(state.kinematics_estimated.orientation.x_val)
    oriY.append(state.kinematics_estimated.orientation.y_val)
    oriZ.append(state.kinematics_estimated.orientation.z_val)
    oriW.append(state.kinematics_estimated.orientation.w_val)
    Cls.append(0)

    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)
    # write to png
    airsim.write_png(os.path.normpath(new_dataset_images_directory + '/im' + str(idx) + '.png'), img_rgb)

    idx = idx + 1

    # stop recorder after set time
    t = time.time()
    if (t-t0) > sim_seconds:
        break

# save IMU data to csv
df = pd.DataFrame(list(zip(T, accX, accY, accZ, angX, angY, angZ, oriX, oriY, oriZ, oriW, Cls)),
                  columns=['Time', 'AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ',
                           'OriX', 'OriY', 'OriZ', 'OriW', 'class'])
df.to_csv(new_dataset_csv_name)

print("Recorded data " + str(idx) + " instances")
print("FPS " + str(idx/sim_seconds))

# ************************************** INDUCING ANOMALY ************************************** #

# print("Destabilizing drone")
# client.enableApiControl(True)
# client.moveByMotorPWMsAsync(127, -127, -127, -127, 3)



