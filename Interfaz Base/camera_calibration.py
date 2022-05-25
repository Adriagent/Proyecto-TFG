import numpy as np
import cv2
from utils import find_devices


def camera_calibration(cam_id, boardSize):

    print("[#] Move the pattern arround the camera and press 'f' to save a frame.")
    print("[#] Once you have finished, press 'esc' to stop saving samples.")

    image_number = 0

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
    objp = np.zeros((boardSize[0]*boardSize[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:boardSize[0],0:boardSize[1]].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # Iniciamos las camaras:
    cap = cv2.VideoCapture(cam_id)

    frame_shape = [480, 640]
    cap.set(3, frame_shape[1])
    cap.set(4, frame_shape[0])


    while(True):

        # Capture key press.
        k = cv2.waitKey(1)

        # Capture frame-by-frame
        frame_ret, frame = cap.read()

        if frame_ret:
            frame = cv2.flip(frame, 1)
            
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_gray.flags.writeable = False # La ponemos como solo lectrua para que se pase por referencia y vaya más rapido.

            # Fast seek of chess board corners.
            ret, frame_corners = cv2.findChessboardCorners(frame_gray, boardSize, cv2.CALIB_CB_FAST_CHECK)


            if k & 0xFF == ord('f'):
                k = -1

                if ret:
                    image_number += 1
                    print("\t- Board found: ", image_number)

                    # Find the chess board corners.
                    _, corners = cv2.findChessboardCorners(frame_gray, boardSize, None)

                    # Refining image points.
                    corners = cv2.cornerSubPix(frame_gray, corners, (11,11), (-1,-1), criteria)

                    # Add object points, image points (after refining them)
                    objpoints.append(objp)
                    imgpoints.append(corners)


            # Draw corner detection.
            
            cv2.drawChessboardCorners(frame, boardSize, frame_corners, ret)

            # Display the resulting frame
            cv2.imshow('camera_'+str(camera_id), frame)


        if k & 0xFF == ord('q') or k & 0xFF == 27:
            break

    # When everything done, release the capture
    cv2.destroyAllWindows()

    if not len(objpoints): return False, 0, 0

    # With the object points and image points we can get the camera matrix, distortion coefficients, rotation and translation vectors
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame_shape, None, None)


    print('- rmse camera '+str(camera_id)+":", ret)


    return True, mtx, dist


if __name__ == "__main__":

    boardSize = (9,6)
    devices = find_devices(3)
    print(devices)

    for camera_id in devices[1:]:

        ret, mtx, dist = camera_calibration(camera_id, boardSize)
        
        if ret:
            np.savez("calibration_data/cam"+str(camera_id)+".npz", mtx=mtx, dist=dist)
