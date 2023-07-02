# Camera Utils 2022

## camera.py
---
Module for search camera id and serial


## camera_list.py
---
List id and its serial of current camera

## camera_calbration.py
---
python3 camera_calibration.py (SERIAL) (POSITION)

- Press 'c' to capture and calculate the corners in the checkerboard

- Press 'c' to admit the corners and calculate the camera matrix. 
Press 's' to skip and retry the procedure

Calibrate camera and create yaml file CameraData.yaml which include camera matrix, distortion coefficients, serial in front_camera directory, or upward_camera directory, or current directory.

SERIAL can be obtained by camera_list.py, and the POSITION indicate the camera position (front or upward) of the camera.

## camera_test.py
---
Camera Node for test. Same with other camera nodes