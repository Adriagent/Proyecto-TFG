import mediapipe as mp
import cv2

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic # Es el conjunto de las herramientas de mediapipe.


# For webcam input:
cap = cv2.VideoCapture(0)

# 
with mp_holistic.Holistic(
    static_image_mode = False, # False por defecto.
    model_complexity = 1, # 1 por defecto.
    smooth_landmarks = True, # True por defecto.
    min_detection_confidence = 0.5, # 0.5 por defecto.
    min_tracking_confidence = 0.5 # 0.5 por defecto.
    ) as holistic:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
			# If loading a video, use 'break' instead of 'continue'.
            continue

		# Flip the image horizontally for a later selfie-view display, and convert the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

		# To improve performance, optionally mark the image as not writeable to pass by reference.
        image.flags.writeable = False

        # Obtain the results of the detection.
        results = holistic.process(image)

		# Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        mp_drawing.draw_landmarks(
            image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0,0,255)),    # Points
            mp_drawing.DrawingSpec(color=(255,0,0))     # Joints
        )

        mp_drawing.draw_landmarks(
            image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0,0,255)),    # Points
            mp_drawing.DrawingSpec(color=(0,255,0))     # Joints
        )

        mp_drawing.draw_landmarks(
            image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0,0,255)),    # Points
            mp_drawing.DrawingSpec(color=(0,255,0))     # Joints
        )

        cv2.imshow("imagen", image)
        if cv2.waitKey(5) & 0xFF == 27: # If pulse ESC.
            break


cap.release()












