#!/usr/bin/env python2.7

from __future__ import print_function
import multiprocessing
import roslib
import sys
import glob
import numpy as np
from numpy.linalg import lstsq
import rospy
import cv2 as cv
import sensor_msgs
import std_msgs
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

class FinishException(Exception):
    pass

class hand_eye(object):
    """
    Subscribes to the forward kinematics node and kinect camera topics - "k4a/rgb/image_raw" and "k4a/ir/image_raw"
    performs necessary calibrations and publishes - intrinsic and extrinsic camera parameters

    The algorithm used to solve is QR24 which was refered from the paper 
    (https://www.rob.uni-luebeck.de/publikationen_downloads/ermm_12.pdf-b46951f770e1e17036ee6b18fe686a1b.pdf)

    ...

    Attributes
    Contains necessary attributes to perform the calibration
    ----------
    total_imgs_cam_calib : int
        attribute which decides the number of images to consider for calibrating
    
    img_diff : int
        attribute to create gap between consequent images  
  

  """
    # criteria for the opencv library functions 
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # List to store transformation matrices generated
    pose_list = []
    camera_tf_list = []

    # Total number of images that will be read
    length = 10
    

    # Counting number of images done
    count_ir = 0
    count_rgb = 0
    
    # declaring objects necessary for publishing
    pub_X = rospy.Publisher("/hand_eye/X_mat", Float64MultiArray, queue_size=10)
    pub_Y = rospy.Publisher("/hand_eye/Y_mat", Float64MultiArray, queue_size=10)
    pub_ready_for_image = rospy.Publisher('/ready_for_image', Bool, queue_size=40)


    def __init__(self):
       self.bridge = CvBridge()   

    
    def sub_hand_eye(self):
        
        rospy.Subscriber("/k4a/rgb/image_raw", Image, self.rgb_callback)           
        rospy.Subscriber("/k4a/ir/image_raw", Image, self.ir_callback)


    """ function where data is read from ros, converted to image type """
    def rgb_callback(self, data):
        
        ready_for_image = rospy.wait_for_message("/ready_for_image", Bool)

        if ready_for_image.data:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
                print("Added a RGB Image")
                rospy.logwarn("Added a RGB Image")
                self.find_corner_rgb(cv_image)

            except CvBridgeError as e:
                print(e)


    def ir_callback(self, data):
        ready_for_image = rospy.wait_for_message("/ready_for_image", Bool)

        if ready_for_image.data:
            try:
                
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
                print("Added an IR Image")
                rospy.logwarn("Added an IR Image")
                self.pub_ready_for_image.publish(False)
                self.find_corner_ir(cv_image)

            except CvBridgeError as e:
                print(e)
  

    def pose_callback(self, msg):
        if len(self.pose_list) <= self.length:
            self.pose_mat = np.asarray(msg.data)
            self.pose_mat = np.resize(self.pose_mat, (4,4))
        
            self.pose_list.append(self.pose_mat)
            print("pose: ", self.pose_mat)
            

    """ Findng the corner points in the RGB images and storing corresponding image points"""
    def find_corner_rgb(self, imgs):
        i = 8           #Number of points to look for along x-axis
        j = 5           #Number of points to look for along Y-axis
        
        fx = 2048       #resizing factor(x-axis) for the image
        fy = 1536       #resizing factor(y-axis) for the image

        self.objpoints_rgb = []
        self.imgpoints_rgb = []

         # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp_rgb = np.zeros((8*5,3), np.float32)
        self.objp_rgb[:,:2] = np.mgrid[0:8,0:5].T.reshape(-1,2)
        
        img = cv.resize(imgs, (fx, fy))
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.img = img
        self.rgb_img_size = img.shape[::-1]
        ret, corners = cv.findChessboardCorners(img, (i,j), 
                                                flags=cv.CALIB_CB_ADAPTIVE_THRESH + 
                                                cv.CALIB_CB_FAST_CHECK +
                                                cv.CALIB_CB_NORMALIZE_IMAGE)

        # If corners found, add object points, image points (after refining them)
        if ret == True:
            self.pose_callback(fk)
            self.objpoints_rgb.append(self.objp_rgb)
            corners2 = cv.cornerSubPix(img, corners, (41,41), (-1,-1), self.criteria)
            self.imgpoints_rgb.append(corners2)
            self.cam_calibration_rgb()


    """ Findng the corner points in the IR images and storing corresponding image points"""               
    def find_corner_ir(self, imgs):
        fk =rospy.wait_for_message("/fk",Float64MultiArray)

        i = 8
        j = 5

        fx_dep = 640
        fy_dep = 576

        self.objpoints_ir = []
        self.imgpoints_ir = []

        self.objp_ir = np.zeros((8*5,3), np.float32)
        self.objp_ir[:,:2] = np.mgrid[0:8,0:5].T.reshape(-1,2)

        img = cv.resize(imgs, (fx_dep, fy_dep))
        self.ir_img_size = img.shape[::-1][1:3]
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img = cv.equalizeHist(img)
        ret, corners = cv.findChessboardCorners(img, (i,j), 
                                                flags=cv.CALIB_CB_ADAPTIVE_THRESH + 
                                                cv.CALIB_CB_FAST_CHECK +
                                                cv.CALIB_CB_NORMALIZE_IMAGE)

        # If corners found, add object points, image points (after refining them)
        if ret == True:
            self.objpoints_ir.append(self.objp_ir)
            corners2 = cv.cornerSubPix(img, corners, (11,11), (-1,-1), self.criteria)
            self.imgpoints_ir.append(corners2)
            self.pose_callback(fk)
            self.pub_ready_for_image.publish(False)
            self.cam_calibration_ir()


    """the detected corner points are processed in this function and the new camera matrix is obtained"""       
    def cam_calibration_rgb(self):
        ret, self.mtx_rgb, self.dist_rgb, self.rvecs_rgb, self.tvecs_rgb = cv.calibrateCamera(
                                                                                self.objpoints_rgb, 
                                                                                self.imgpoints_rgb, 
                                                                                self.rgb_img_size, 
                                                                                cameraMatrix= 0, 
                                                                                distCoeffs = 0, 
                                                                                flags = cv.CALIB_RATIONAL_MODEL)
        
        self.h, self.w = self.rgb_img_size
        self.newcameramtx_rgb, roi = cv.getOptimalNewCameraMatrix(self.mtx_rgb, 
                                                                  self.dist_rgb, 
                                                                  (self.w, self.h), 1, 
                                                                  (self.w, self.h))
        cam_tf = self.cal_tf_mat()      
       
        if len(self.camera_tf_list) <= self.length:
            self.camera_tf_list.append(cam_tf)
        
        elif len(self.camera_tf_list) >= self.length:
            print("Camera calibration done, beginning hand eye calculation...")
            rospy.logwarn("Camera calibration done, beginning hand eye calculation...")
            self.call_hand_eye()
        

    def cam_calibration_ir(self):
        ret, self.mtx_ir, self.dist_ir, self.rvecs_ir, self.tvecs_ir = cv.calibrateCamera(
                                                                        self.objpoints_ir, self.imgpoints_ir, 
                                                                        self.ir_img_size, cameraMatrix= 0, 
                                                                        distCoeffs = 0, 
                                                                        flags = cv.CALIB_RATIONAL_MODEL)
        
        h, w = self.ir_img_size
        self.newcameramtx_ir, roi = cv.getOptimalNewCameraMatrix(self.mtx_ir, 
                                                                 self.dist_ir, 
                                                                 (w,h), 1, 
                                                                 (w,h)  )


    """Calculating transformation matrix from the intrinsic values generated """
    def cal_tf_mat(self):
            
        a = np.asarray(self.rvecs_rgb)        
        a = np.resize(a, (3,1))

        rot, jac = cv.Rodrigues(a)

        cam_tf = np.eye(4, dtype=float)

        for i in range(3):
            for j in range(3):
                cam_tf[i][j] = rot[i][j]
        try:
            tvec_rgb = np.resize((np.asarray(self.tvecs_rgb)), (3,1)) 
            tvec_ir = np.resize((np.asarray(self.tvecs_ir)), (3,1))

            cam_tf[0][3] = tvec_rgb[0][0] 
            cam_tf[1][3] = tvec_rgb[1][0]
            cam_tf[2][3] = tvec_ir[2][0]
            return cam_tf
        
        except Exception:
            print("the cam calibration in class hand eye is not finished yet")


    def call_hand_eye(self):
        if len(self.pose_list) and len(self.camera_tf_list) == self.length:
            self.hand_eye_calibration()


    """Calculating the X and Y matrices using base to end effector tf matrix and camera tf matrix using QR24 algorithm """
    def hand_eye_calibration(self) :
        tf_b_ee = self.pose_list
        tf_cm_t = self.camera_tf_list

        size = len(tf_b_ee)
        A = np.zeros((12*size, 24), dtype=float)
        B = np.zeros((12*size, 1), dtype=float)
        
        E_12 = np.eye(12, dtype=float)
        E_12 = E_12 * -1
        
        try:

            for i in range(size):
                Mat_A = np.zeros((12,9), dtype=float)
                fk_tf = tf_b_ee[i]
                
                cam_tf = tf_cm_t[i]
                
                Mat_R = fk_tf[0:3, 0:3]
                fk_ts = -fk_tf[0:3, 3:4]
                Mat_N = np.linalg.inv(tf_cm_t[i])

                a_00 = Mat_R * Mat_N[0][0]
                a_01 = Mat_R * Mat_N[0][1]
                a_02 = Mat_R * Mat_N[0][2]
                a_03 = Mat_R * Mat_N[0][3]
                a_10 = Mat_R * Mat_N[1][0]
                a_11 = Mat_R * Mat_N[1][1]
                a_12 = Mat_R * Mat_N[1][2]
                a_13 = Mat_R * Mat_N[1][3]
                a_20 = Mat_R * Mat_N[2][0]
                a_21 = Mat_R * Mat_N[2][1]
                a_22 = Mat_R * Mat_N[2][2]
                a_23 = Mat_R * Mat_N[2][3]
                
                A_ = np.zeros((12,24), dtype=float)
                B_ = np.zeros((12,1), dtype = float)

                for j in range(len(a_00)):
                    for k in range(len(a_00)):
                        A_[j][k] = a_00[j][k]

                for j in range(len(a_10)):
                    for k in range(len(a_10)):
                        A_[j][k+3] = a_10[j][k]

                for j in range(len(a_20)):
                    for k in range(len(a_20)):
                        A_[j][k+6] = a_20[j][k]

                for j in range(len(a_01)):
                    for k in range(len(a_01)):
                        A_[j+3][k] = a_01[j][k]

                for j in range(len(a_11)):
                    for k in range(len(a_11)):
                        A_[j+3][k+3] = a_11[j][k]

                for j in range(len(a_21)):
                    for k in range(len(a_21)):
                        A_[j+3][k+6] = a_21[j][k]

                for j in range(len(a_02)):
                    for k in range(len(a_02)):
                        A_[j+6][k] = a_02[j][k]

                for j in range(len(a_12)):
                    for k in range(len(a_12)):
                        A_[j+6][k+3] = a_12[j][k]

                for j in range(len(a_22)):
                    for k in range(len(a_22)):
                        A_[j+6][k+6] = a_22[j][k]

                for j in range(len(a_03)):
                    for k in range(len(a_03)):
                        A_[j+9][k] = a_03[j][k]

                for j in range(len(a_13)):
                    for k in range(len(a_13)):
                        A_[j+9][k+3] = a_13[j][k]

                for j in range(len(a_23)):
                    for k in range(len(a_23)):
                        A_[j+9][k+6] = a_23[j][k]

                for j in range(len(Mat_R)):
                    for k in range(len(Mat_R)):
                        A_[9+j][9+k] = Mat_R[j][k]

                for j in range(len(E_12)):
                    for k in range(len(E_12)):
                        A_[j][k+12] = E_12[j][k]

                for j in range(12):
                    for k in range(24):
                        A[(i*12)+j][k] = A_[j][k]

                for j in range(len(fk_ts)):
                    B_[9+j] = fk_ts[j]

                for j in range(len(B_)):
                    B[(i*12)+j] = B_[j]
            
            
            # Solving Ax = yB, using numpy least square, 
            # here A & B are known, x & y are unknowns
            x = lstsq(A,B, rcond= None)
            
            x = np.asarray(x)
            
            x0 = x[0]
            x0 = x0.reshape(-1)
            x01 = np.zeros((3,1), dtype=float)
            x02 = np.zeros((3,1), dtype=float)

            x0 = np.resize(x0, (6, 4))

            x01[0][0] = x0[2][1]
            x01[1][0] = x0[2][2]
            x01[2][0] = x0[2][3]
            
            x02[0][0] = x0[5][1]
            x02[1][0] = x0[5][2]
            x02[2][0] = x0[5][3]


            # tf between camera and end effector
            X_mat = np.eye(4, dtype=float)

            # tf between camera and robot base
            Y_mat = np.eye(4, dtype=float)

            for i in range(3):
                for j in range(3):
                    X_mat[i][j] = x0[i][j]
            
            X_mat = X_mat.transpose()

            X_mat[0][3] = x01[0][0]
            X_mat[1][3] = x01[1][0]
            X_mat[2][3] = x01[2][0]


            for i in range(3):
                for j in range(3):
                    Y_mat[i][j] = x0[i+3][j]
            
            Y_mat = Y_mat.transpose()

            Y_mat[0][3] = x02[0][0]
            Y_mat[1][3] = x02[1][0]
            Y_mat[2][3] = x02[2][0]

            print("X_mat: ",X_mat)
            rospy.logwarn("X_mat: ", X_mat)

            
            print("Y_mat: ",Y_mat)
            rospy.logwarn("Y_mat: ", Y_mat)

            self.publish_mat(X_mat, Y_mat)
        
        except TypeError as te:
            print("Type error happened, usually happens when values are not stored while processing, so rerunning !! No need to worry")
            rospy.logwarn("Type error happened, usually happens when values are not stored while processing, so rerunning !! No need to worry")


    """Publishing the generated X and Y matrices"""    
    def publish_mat(self, X_mat, Y_mat):
        X_mat = X_mat.flatten()
        Y_mat = Y_mat.flatten()
        
        X_mat_data = Float64MultiArray()
        Y_mat_data = Float64MultiArray()
        
        X_mat_data.data = X_mat
        Y_mat_data.data = Y_mat

        self.pub_X.publish(X_mat_data)
        self.pub_Y.publish(Y_mat_data)
        sys.exit()

        print("publishing Hand eye results")
        rospy.logwarn("publishing Hand eye results")


#Beginning of the program        
if __name__ == '__main__':
    
    try:        
        he = hand_eye()
        rospy.init_node('hand_eye', anonymous=True)
        he.sub_hand_eye()
        rospy.spin()

    except SystemExit:
        print("Finished !")
    
     