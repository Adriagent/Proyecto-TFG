import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands # Nos permite detectar las manos.
mp_drawing = mp.solutions.drawing_utils # Nos permite dibujar la deteccion (21 puntos y sus conexiones).

# For webcam input:
cap = cv2.VideoCapture(0)

# min_detection_confidence: valor minimo de la detección para considerarla como exitosa.
# min_tracking_confidence: valor mínimo para que el rastreo de los 21 puntos se considere exitoso. 
# (si es menor a ese valor, se aplica el detector para volver a encontrarlas)
with mp_hands.Hands(max_num_hands = 2, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
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
		results = hands.process(image)

		# Draw the hand annotations on the image.
		image.flags.writeable = True
		image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		
		if results.multi_hand_landmarks:
			
			
			for (i, hand_landmarks) in enumerate(results.multi_hand_landmarks):

				POINTS_COLOR = mp_drawing.DrawingSpec(color=(0,0,255))
				JOINTS_COLOR = mp_drawing.DrawingSpec(color=(0,255,0))

				if results.multi_handedness[i].classification[0].label == "Right":
					POINTS_COLOR = mp_drawing.DrawingSpec(color=(0,0,255))
					JOINTS_COLOR = mp_drawing.DrawingSpec(color=(255,0,0))
			
				mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS, POINTS_COLOR, JOINTS_COLOR)

		cv2.imshow('MediaPipe Hands', image)

		if cv2.waitKey(5) & 0xFF == 27: # If pulse ESC.
			break


cap.release()