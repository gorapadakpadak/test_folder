#!/home/snuzero/anaconda3/envs/tl/bin/python3
import numpy as np
import yaml
import sys
import camera
import cv2
import os

FPS = 60
SERIAL=sys.argv[1]
ID=camera.serial2id(SERIAL)
print(ID)
# cap = cv2.VideoCapture(int(ID))

# Initialize Webcam
exp, exp_flag = 15, 12
cap = cv2.VideoCapture(ID)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, FPS)
cap.set(cv2.CAP_PROP_GAIN, 0)
cap.set(cv2.CAP_PROP_BACKLIGHT, 0)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
cap.set(cv2.CAP_PROP_CONTRAST, 144)
cap.set(cv2.CAP_PROP_SATURATION, 176)
cap.set(cv2.CAP_PROP_SHARPNESS, 128)
cap.set(cv2.CAP_PROP_AUTO_WB, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, exp)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 0)
cap.set(cv2.CAP_PROP_PAN, 0)
cap.set(cv2.CAP_PROP_TILT, 0)
cap.set(cv2.CAP_PROP_ZOOM, 100)
os.system(f"v4l2-ctl -d {ID} -c power_line_frequency=2")
os.system(f"v4l2-ctl -d {ID} -c led1_mode=1")
os.system(f"v4l2-ctl -d {ID} -c led1_frequency=0")
os.system(f"v4l2-ctl -d {ID} -c exposure_auto_priority=1")


while True:
    ret, img = cap.read()

    # Process and Publish
    if ret:
        # Auto Exposure
        if exp_flag > 0:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, (320, 180), interpolation=cv2.INTER_AREA)
            gray = np.mean(gray)
            if 120 <= gray <= 140:
                exp_flag = 0
            elif gray < 120:
                exp += 1
                exp_flag -= 1
            else:
                exp -= 1
                exp_flag -= 1
            cap.set(cv2.CAP_PROP_EXPOSURE, exp)
        elif exp_flag == 0:
            if exp >= 25:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            exp_flag = -1

        img = img[:, 160:-160]
        img = cv2.resize(img, (640, 480), cv2.INTER_AREA)

        # Visualize
        cv2.imshow("camera_front", img)
        key=cv2.waitKey(1)
        if key==ord('s'): break

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((1, 8 * 6, 3), np.float32) # * 2
objp[0, :, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
objpoints = []
imgpoints = []
cnt=0

while True:
    ret, img = cap.read()
    if ret:
        img = img[:, 160:-160]
        img = cv2.resize(img, (640, 480), cv2.INTER_AREA)
        cv2.imshow("Checkerboard Image", img)

        key = cv2.waitKey(1)
        if key == ord("c"):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret2, corners = cv2.findChessboardCorners(gray, (8, 6), None)
            if ret2 == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (25, 25), (-1, -1), criteria)
                imgpoints.append(corners2)

                cv2.drawChessboardCorners(img, (8, 6), corners2, ret2)
                cv2.imshow("Corners", img)
                key = cv2.waitKey(0)
                if key == ord("c"):
                    cv2.destroyWindow("Corners")
                    cnt=cnt+1
                    print("capture "+ str(cnt))
                    continue

                else:  # skip
                    del objpoints[-1]
                    del imgpoints[-1]
                    cv2.destroyWindow("Corners")
                    print("skip")
            else:
                print("No CheckerBoard")

        elif key == ord("q"):
            break

cv2.destroyAllWindows()

if len(objpoints)>0:
    ret, CAM_MAT, DIST_MAT, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    print(f"{ret=}\n{CAM_MAT=}\n{DIST_MAT=}\n{rvecs=}\n{tvecs=}")
else:
    CAM_MAT=np.array(
            [[755,0,640],
            [0,755,480],
            [0,0,1]]
            )  # Camera Matrix
    DIST_MAT = np.array(
        [
            1.8381197097085436e-01,
            -4.1621081248642072e-01,
            1.4046580969321468e-04,
            -1.1921687101843280e-04,
            2.1254824941541825e-01,
        ]
)  # Distortion Matrix
cam_dat={'SERIAL':SERIAL,
        'CAM_MAT':CAM_MAT.tolist(),
        'DIST_MAT':DIST_MAT.tolist()} 

if sys.argv[2]=='frontleft':
    path='../front_camera/left/'
elif sys.argv[2]=='frontcenter':
    path='../front_camera/center/'
elif sys.argv[2]=='frontright':
    path='../front_camera/right/'
elif sys.argv[2]=='upward':
    path='../upward_camera/'
else:
    path='./'
    print("Save configure in current path")
    
with open(path+'CameraData.yaml', 'w') as f:
    yaml.dump(cam_dat, f)
