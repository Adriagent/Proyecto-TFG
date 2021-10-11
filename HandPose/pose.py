import mediapipe as mp
import cv2

#mp_pose = mp.solutions.pose # Es el conjunto de las herramientas de mediapipe.
mp_pose = mp.solutions.mediapipe.python.solutions.pose


# For webcam input:
cap = cv2.VideoCapture(0)


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

        if results.pose_landmarks:
            # LEFT:
            Lx1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * width)
            Ly1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * height)
            z1 = float(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].z)

            Lx2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x * width)
            Ly2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y * height)
            z2 = float(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].z)

            Lx3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x * width)
            Ly3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y * height)
            z3 = float(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].z)

            cv2.line(image, (Lx1,Ly1), (Lx2,Ly2), (0,255,0), 3)
            cv2.line(image, (Lx2,Ly2), (Lx3,Ly3), (0,255,0), 3)
            cv2.circle(image, (Lx1,Ly1), 6, (0,0,255), -1)
            cv2.circle(image, (Lx2,Ly2), 6, (0,0,255), -1)
            cv2.circle(image, (Lx3,Ly3), 6, (0,0,255), -1)


            # RIGHT:
            x1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x * width)
            y1 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y * height)
            x2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].x * width)
            y2 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].y * height)
            x3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x * width)
            y3 = int(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y * height)

            cv2.line(image, (x1,y1), (x2,y2), (255,0,0), 3)
            cv2.line(image, (x2,y2), (x3,y3), (255,0,0), 3)
            cv2.circle(image, (x1,y1), 6, (0,0,255), -1)
            cv2.circle(image, (x2,y2), 6, (0,0,255), -1)
            cv2.circle(image, (x3,y3), 6, (0,0,255), -1)

            #print(z1, z2, z3)

        cv2.imshow("imagen", image)
        if cv2.waitKey(5) & 0xFF == 27: # If pulse ESC.
            break



cap.release()












