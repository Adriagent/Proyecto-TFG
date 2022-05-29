import cv2
from utils import find_devices

devices = find_devices(3)

# This will return video from the first webcam on your computer.
cap1 = cv2.VideoCapture(devices[1])
cap2 = cv2.VideoCapture(devices[2])

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out1 = cv2.VideoWriter('cam1.avi', fourcc, 20.0, (640, 480))
out2 = cv2.VideoWriter('cam2.avi', fourcc, 20.0, (640, 480))

# loop runs if capturing has been initialized.
while(True):
	# reads frames from a camera
	# ret checks return at each frame
	ret, frame1 = cap1.read()
	ret, frame2 = cap1.read()
	
	# output the frame
	out1.write(frame1)
	out2.write(frame2)
	
	# The original input frame is shown in the window
	cv2.imshow('cam1', frame1)
	cv2.imshow('cam2', frame2)

	
	# Wait for 'a' key to stop the program
	if cv2.waitKey(1) & 0xFF == ord('a'):
		break

# Close the window / Release webcam
cap1.release()
cap2.release()

# After we release our webcam, we also release the output
out1.release()
out2.release()

# De-allocate any associated memory usage
cv2.destroyAllWindows()
