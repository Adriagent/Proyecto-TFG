import mediapipe as mp
import cv2
import numpy as np
import sys
import select

from time import time

from utils.communication import client_tcp
from utils.utils import find_devices, DrawLines, setup


def mediapipe_detection(P1, P2, devices):
    try:
        client = client_tcp('localhost', 50002)
    except ConnectionRefusedError:
        exit("[ERROR]: No se ha podido conectar con el servidor.")

    last = time()
    previous_frame_time = time()
    ready = False
    n = 0


    mp_pose = mp.solutions.mediapipe.python.solutions.pose
    pose0 = mp_pose.Pose()
    pose1 = mp_pose.Pose()

    pose_keypoints = {  
                        "L_wrist"       : mp_pose.PoseLandmark.LEFT_WRIST,
                        "R_wrist"       : mp_pose.PoseLandmark.RIGHT_WRIST,
                        "L_shoulder"    : mp_pose.PoseLandmark.LEFT_SHOULDER,
                        "R_shoulder"    : mp_pose.PoseLandmark.RIGHT_SHOULDER,
                    }

    points_3D = pose_keypoints.copy()

    cap0 = cv2.VideoCapture(devices[0])
    cap1 = cv2.VideoCapture(devices[1])

    while cap0.isOpened() and cap1.isOpened():
        ret0, image0 = cap0.read()
        ret1, image1 = cap1.read()

        if not ret0 or not ret1: break

        image0.flags.writeable = False
        image1.flags.writeable = False

        image0 =cv2.cvtColor(image0, cv2.COLOR_BGR2RGB)
        image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)

        results0 = pose0.process(image0)
        results1 = pose1.process(image1)

        image0.flags.writeable = True
        image1.flags.writeable = True

        image0 = cv2.cvtColor(image0, cv2.COLOR_RGB2BGR)
        image1 = cv2.cvtColor(image1, cv2.COLOR_RGB2BGR)


        # Obtenemos las coordenadas de los keypoints de la camara 0.        
        frame0_keypoints = []
        frame1_keypoints = []
        if results0.pose_landmarks and results1.pose_landmarks:
            for kpt in pose_keypoints:

                x0 = int(results0.pose_landmarks.landmark[pose_keypoints[kpt]].x * image0.shape[1])
                y0 = int(results0.pose_landmarks.landmark[pose_keypoints[kpt]].y * image0.shape[0])
                frame0_keypoints.append(np.array([x0,y0],dtype=float))

                x1 = int(results1.pose_landmarks.landmark[pose_keypoints[kpt]].x * image1.shape[1])
                y1 = int(results1.pose_landmarks.landmark[pose_keypoints[kpt]].y * image1.shape[0])
                frame1_keypoints.append(np.array([x1,y1],dtype=float))

        else: 
            frame0_keypoints = np.array([[-1, -1]]*len(pose_keypoints)) # if no keypoints are found, simply fill the frame data with [-1,-1] for each kpt
            frame1_keypoints = np.array([[-1, -1]]*len(pose_keypoints))


        # Tras esto ya tenemos las coordenadas para cada keypoint, solo queda calcular la posicion 3D de estos.
        for kpt, uv1, uv2 in zip(pose_keypoints, frame0_keypoints, frame1_keypoints):
            p3d = np.array([-1, -1, -1])

            if uv1[0] != -1 and uv2[0] != -1:
                p4d = cv2.triangulatePoints(P1, P2, uv1, uv2)  # calculate 3d position of keypoints.
                p3d = (p4d[:3, 0] / p4d[3, 0]).T

            points_3D[kpt] = p3d.copy()
        


        # Enviamos los puntos a la simulación.
        if client.is_open() and time() - last > 0.1:

            L_wrist = points_3D["L_wrist"] - points_3D["L_shoulder"] # Ponemos como origen el hombro izquierdo.
            R_wrist = points_3D["R_wrist"] - points_3D["R_shoulder"] # Ponemos como origen el hombro derecho.

            L_wrist = np.array([(-L_wrist[0]/80), -L_wrist[2]/100, -L_wrist[1]/100], dtype=float)# x, y, z
            R_wrist = np.array([-R_wrist[0]/15, R_wrist[2]/15, -R_wrist[1]/15], dtype=float).clip(0,0.05)

            L_wrist[0] = np.clip(L_wrist[0],-0.15, 0.2)     # x
            L_wrist[1] = np.clip(L_wrist[1],-0.15, 0.05)    # y 
            L_wrist[2] = np.clip(L_wrist[2],-0.05, 0.15)    # z

            print("muñeca izquierda (xzy) =", L_wrist)
            # print("muñeca derecha   =", R_wrist, "\n")

            DrawLines(image0, results0)
            DrawLines(image1, results1)

            image0 = cv2.flip(image0,1)
            image1 = cv2.flip(image1,1)

            fps = int(1/(time()-previous_frame_time))
            previous_frame_time = time()

            cv2.putText(image0, "FPS:"+str(fps), (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(image1, "FPS:"+str(fps), (7, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            cv2.imshow("cam0", image0)
            cv2.imshow("cam1", image1)
            
            if not ready:
                check = select.select([client.sock], [], [], 0.01)
                if check[0]:
                    client.sock.recv(1024).decode()
                    ready = True
            elif n < 10:
                    n+=1
            else:
                msg = np.concatenate((L_wrist, R_wrist), axis=None)
                client.enviar(msg)
            
            last = time()


        if cv2.waitKey(5) & 0xFF == 27:
            break



if __name__ == "__main__":

    if len(sys.argv) == 1:
        devices = find_devices(3)

        if len(devices) < 3: exit("[#]: Not enough cameras detected!")

        P1, P2 = setup(devices[1:]) # Obtenemos las matrices de proyeccion.

        mediapipe_detection(P1, P2, devices[1:])

    else:
        devices = [0, 2, 4]
        P1, P2 = setup(devices[1:]) # Obtenemos las matrices de proyeccion.

        devices = [0, "utils/cam1.avi", "utils/cam2.avi"]
        mediapipe_detection(P1, P2, devices[1:])

