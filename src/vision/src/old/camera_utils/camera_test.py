#!/home/snuzero/anaconda3/envs/TL/bin/python3
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import os
import numpy as np
import cv2
from time import sleep, perf_counter
import copy

import yaml
from yaml.loader import SafeLoader

import camera

with open('CameraData.yaml', 'r') as f:
    data=list(yaml.load_all(f, Loader=SafeLoader))

FPS = 60
SERIAL = data[0]['SERIAL']
CAM_MAT = np.array(
        data[0]['CAM_MAT']
)  # Camera Matrix
DIST_MAT = np.array(
        data[0]['DIST_MAT']
)  # Distortion Matrix
ID = camera.serial2id(SERIAL)

def publisher():
    rospy.init_node("TestCameraImageNode")
    pub = rospy.Publisher("/test_camera_image", Image, queue_size=1)
    bridge = CvBridge()
    q = []

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

    try:
        while not rospy.is_shutdown():
            t0 = perf_counter()
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
                    
                h, w = img.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(CAM_MAT, DIST_MAT, (w, h), 1, (w,h))
                img = cv2.undistort(img, CAM_MAT, DIST_MAT, None, newcameramtx)
                img = img[:, 160:-160]
                img = cv2.resize(img, (640, 480), cv2.INTER_AREA)
                q.append(copy.deepcopy(img))

                # Visualize
                cv2.imshow("camera_test", img)
                cv2.waitKey(1)

            # Wait
            while True:
                diff = t0 + 1 / FPS - perf_counter()
                if diff <= 0:
                    break
                else:
                    sleep(diff / 2)

            if len(q) > 1:
                camera_test = bridge.cv2_to_imgmsg(q.pop(0), "passthrough")
                camera_test.header = Header()
                camera_test.header.stamp = rospy.Time.now()
                camera_test.header.frame_id = 'test_cam_imgplane'
                pub.publish(camera_test)

    except:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        print("Error!")
