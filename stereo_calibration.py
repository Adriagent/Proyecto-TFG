import numpy as np
import cv2
from glob import glob
from utils.utils import find_devices


def stereo_calibration(cam_id, boardSize, mtx0, dist0, mtx1, dist1):

    print("[#] Move the pattern arround the camera and press 'f' to save a frame.")
    print("[#] Once you have finished, press 'esc' to stop saving samples.")

    image_number = 0
    frame_shape = [480, 640]

    # termination criteria. change this if stereo calibration not good.
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
    objp = np.zeros((boardSize[0]*boardSize[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:boardSize[0],0:boardSize[1]].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space

    imgpoints0 = [] # 2d points in image plane.
    imgpoints1 = [] # 2d points in image plane.

    # Iniciamos las camaras:
    cap0 = cv2.VideoCapture(cam_id[0])
    cap1 = cv2.VideoCapture(cam_id[1])

    caps = [cap0, cap1]

    for camera in caps:
        camera.set(3, frame_shape[1])
        camera.set(4, frame_shape[0])
        
    


    while(True):

        # Capture key press.
        k = cv2.waitKey(1)

        # Capture frame-by-frame
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()

        if ret0 and ret1:

            frame0 = cv2.flip(frame0, 1)
            frame1 = cv2.flip(frame1, 1)
            
            frame0_gray = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)
            frame1_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

            frame0_gray.flags.writeable = False
            frame1_gray.flags.writeable = False

            # Fast seek of chess board corners.
            ret0, frame0_corners = cv2.findChessboardCorners(frame0_gray, boardSize, cv2.CALIB_CB_FAST_CHECK)
            ret1, frame1_corners = cv2.findChessboardCorners(frame1_gray, boardSize, cv2.CALIB_CB_FAST_CHECK)


            if k & 0xFF == ord('f'):
                k = -1

                if ret0 and ret1:
                    image_number += 1
                    print("\t- Board found: ", image_number)

                    # Find the chess board corners.
                    _, corners0 = cv2.findChessboardCorners(frame0_gray, boardSize, None)
                    _, corners1 = cv2.findChessboardCorners(frame1_gray, boardSize, None)

                    # Refining image points.
                    corners0 = cv2.cornerSubPix(frame0_gray, corners0, (11,11), (-1,-1), criteria)
                    corners1 = cv2.cornerSubPix(frame1_gray, corners1, (11,11), (-1,-1), criteria)

                    # Add object points, image points (after refining them)
                    objpoints.append(objp)
                    imgpoints0.append(corners0)
                    imgpoints1.append(corners1)


            # Draw corner detection.
            cv2.drawChessboardCorners(frame0, boardSize, frame0_corners, ret0)
            cv2.drawChessboardCorners(frame1, boardSize, frame1_corners, ret1)

            # Display the resulting frame
            cv2.imshow('img0', frame0)
            cv2.imshow('img1', frame1)


        if k & 0xFF == ord('q') or k & 0xFF == 27:
            break

    # When everything done, release the capture
    cv2.destroyAllWindows()

    if not len(objpoints): return False, 0, 0

    # Calculamos el stereo.
    ret, CM1, dist1, CM2, dist2, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints0, imgpoints1, mtx0, dist0,
    mtx1, dist1, frame_shape, criteria = criteria, flags = cv2.CALIB_FIX_INTRINSIC)

    print("Stereo RMSE:", ret)

    return True, R,T


if __name__ == "__main__":

    boardSize = (9,6)
    devices = find_devices(3)
    print(devices)

    if len(devices) < 3: exit("[#]: Not enough cameras detected!")

    
    # With the object points and image points we can get the camera matrix, distortion coefficients, rotation and translation vectors
    mtx = []
    dist = []
    for id in devices[1:]:

        if not glob("calibration_data/cam" + str(id) + ".npz"):
            print("[#] Could not find intrinsic params of camera:", id)
            exit()

        data = np.load("calibration_data/cam"+ str(id) + ".npz")
        
        mtx.append(data["mtx"])
        dist.append(data["dist"])


    ret, R, T = stereo_calibration(devices[1:], boardSize, mtx[0], dist[0], mtx[1], dist[1])
    
    if ret:
        np.savez("calibration_data/stereo.npz", R=R, T=T)


