import numpy as np
import cv2
import glob
import yaml

def save_images(n):
    camera = cv2.VideoCapture(0)
    i =0 
    while i<n: 
        raw_input('Press Enter to capture')
        return_value, image = camera.read()
        cv2.imwrite('opencv'+str(i)+'.jpg', image)
        i += 1

    del(camera)

def calibrate_camera():
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob('*.png')
    cnt = 0
    for fname in images:
        cnt += 1
        print('Image {}'.format(cnt))
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
        
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (7,6), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    # calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # transform the matrix and distortion to writable list
    data = {'camera_matrix': np.asarray(mtx).tolist(),
        'dist_coeff': np.asarray(dist).tolist()}

    # save to a file
    with open('calibration_matrix.yaml',"w") as f:
        yaml.dump(data,f)

    #  use calibration to undistort an image
    img = cv2.imread('opencv0.png')
    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    cv2.imwrite('undistorted.jpg',dst)

    cv2.destroyAllWindows()

# to save images
#save_images(10)

# load images and calibrate
calibrate_camera()


