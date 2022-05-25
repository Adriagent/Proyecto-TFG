import mediapipe as mp
import cv2
import numpy as np
from glob import glob
from scipy import linalg


def DrawLines(img, results):

    mp_pose = mp.solutions.mediapipe.python.solutions.pose

    if results.pose_landmarks:
            kypt_pos = []
            keypoint_list = [mp_pose.PoseLandmark.RIGHT_SHOULDER, mp_pose.PoseLandmark.RIGHT_ELBOW, mp_pose.PoseLandmark.RIGHT_WRIST,
                             mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.LEFT_ELBOW, mp_pose.PoseLandmark.LEFT_WRIST]

            for kpt in keypoint_list:
                x = int(results.pose_landmarks.landmark[kpt].x * img.shape[1])
                y = int(results.pose_landmarks.landmark[kpt].y * img.shape[0])
                kypt_pos.append([x, y])

            # Lines
            for i in range(2):
                cv2.line(img, kypt_pos[i], kypt_pos[i+1], (0,255,0), 3)
                cv2.line(img, kypt_pos[i+3], kypt_pos[i+4], (255,0,0), 3)

            for i in range(3):
                cv2.circle(img, kypt_pos[i], 6, (0,0,255), -1)
                cv2.circle(img, kypt_pos[i+3], 6, (0,0,255), -1)


def setup(devices):
    mtx = []
    dist = []
    for id in devices:

        if not glob("calibration_data/cam" + str(id) + ".npz"):
            print("[#] Could not find intrinsic params of camera:", id)
            exit()

        data = np.load("calibration_data/cam"+ str(id) + ".npz")
        
        mtx.append(data["mtx"])
        dist.append(data["dist"])

    if not glob("calibration_data/stereo.npz"):
        print("[#] Could not find stereo params")
        exit()

    data = np.load("calibration_data/stereo.npz")
    R = data["R"]
    T = data["T"]

    ### Calculamos la matriz de proyeccion:
    
    #RT matrix for C1 is identity.
    RT1 = np.concatenate([np.eye(3), [[0],[0],[0]]], axis = -1)
    P1 = mtx[0] @ RT1 #projection matrix for C1
 
    #RT matrix for C2 is the R and T obtained from stereo calibration.
    RT2 = np.concatenate([R, T], axis = -1)
    P2 = mtx[1] @ RT2 #projection matrix for C2

    return P1, P2


def find_devices(num_devices):
    
    devices = []
    n = 0
    while len(devices) < num_devices:
        cap = cv2.VideoCapture(n)
        
        if cap and cap.isOpened():
            devices.append(n)

        if n == 10: break

        n+=1

    return devices
