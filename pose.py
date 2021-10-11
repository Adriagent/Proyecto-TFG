from cv2 import data
from numpy import int32
import rospy
from std_msgs.msg import String
import mediapipe as mp
import cv2
import math
from math import pi



def DrawLines(img, results):
    if results.pose_landmarks:
            # LEFT:
            Lx1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * width)
            Ly1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * height)
            Lz1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].z * width)

            Lx2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x * width)
            Ly2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y * height)
            Lz2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].z * width)

            Lx3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x * width)
            Ly3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y * height)
            Lz3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].z * width)

            # RIGHT:
            Rx1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x * width)
            Ry1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y * height)
            Rz1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].z * width)

            Rx2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].x * width)
            Ry2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].y * height)
            Rz2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].z * width)

            Rx3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x * width)
            Ry3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y * height)
            Rz3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].z * width)


            cv2.line(image, (Lx1,Ly1), (Lx2,Ly2), (0,255,0), 3)
            cv2.line(image, (Lx2,Ly2), (Lx3,Ly3), (0,255,0), 3)
            cv2.circle(image, (Lx1,Ly1), 6, (0,0,255), -1)
            cv2.circle(image, (Lx2,Ly2), 6, (0,0,255), -1)
            cv2.circle(image, (Lx3,Ly3), 6, (0,0,255), -1)

            cv2.line(image, (Rx1,Ry1), (Rx2,Ry2), (255,0,0), 3)
            cv2.line(image, (Rx2,Ry2), (Rx3,Ry3), (255,0,0), 3)
            cv2.circle(image, (Rx1,Ry1), 6, (0,0,255), -1)
            cv2.circle(image, (Rx2,Ry2), 6, (0,0,255), -1)
            cv2.circle(image, (Rx3,Ry3), 6, (0,0,255), -1)




mp_pose = mp.solutions.mediapipe.python.solutions.pose
# For webcam input:
cap = cv2.VideoCapture(0)

# Iniciamos nodo:
rospy.init_node('detectar_pose')
# Iniciamos topic al que enviar la posicion.
pub = rospy.Publisher('angulos_pose', String, queue_size=10)
rate = rospy.Rate(1)


with mp_pose.Pose(
    static_image_mode = False, # False por defecto.
    model_complexity = 1, # 1 por defecto.
    smooth_landmarks = True, # True por defecto.
    min_detection_confidence = 0.5, # 0.5 por defecto.
    min_tracking_confidence = 0.5 # 0.5 por defecto.
    ) as pose:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
			# If loading a video, use 'break' instead of 'continue'.
            continue
        
		# Flip the image horizontally for a later selfie-view display, and convert the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        height, width, _ = image.shape
		# To improve performance, optionally mark the image as not writeable to pass by reference.
        image.flags.writeable = False

        # Obtain the results of the detection.
        results = pose.process(image)

		# Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_world_landmarks:
            # LEFT:
            Lx1 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * width)
            Ly1 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * height)
            Lz1 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].z * width)

            Lx2 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x * width)
            Ly2 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y * height)
            Lz2 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].z * width)

            Lx3 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x * width)
            Ly3 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y * height)
            Lz3 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].z * width)

            # RIGHT:
            Rx1 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x * width)
            Ry1 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y * height)
            Rz1 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].z * width)

            Rx2 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].x * width)
            Ry2 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].y * height)
            Rz2 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].z * width)

            Rx3 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x * width)
            Ry3 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y * height)
            Rz3 = int(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].z * width)


            if not rospy.is_shutdown():
            # publicamos el mensaje

                Lx = Lx2 - Lx1
                Ly = abs(Ly2-height) - abs(Ly1-height)
                Lz = -(Lz2 - Lz1)

                theta = math.atan2(Ly,Lx)
                if theta > 0:
                    theta = pi - theta
                else:
                    theta = -pi - theta

                phi = math.atan2(Lx,Lz)

                a = [phi, theta]
                a = " ".join(str(x) for x in a)
                pub.publish(a)
                #rate.sleep()

            DrawLines(image, results)


        image.flags.writeable = False
        cv2.imshow("imagen", image)
        if cv2.waitKey(5) & 0xFF == 27: # If pulse ESC.
            break



cap.release()












