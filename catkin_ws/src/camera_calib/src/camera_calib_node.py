#!/usr/bin/env python2.7

import multiprocessing
import roslib
import sys
import glob
import numpy as np
import rospy
import cv2 as cv
import sensor_msgs
import std_msgs
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from __future__ import print_function


class calibration(object):
  """
  Subscribes to the kinect camera topics - "k4a/rgb/image_raw" and "k4a/ir/image_raw"
  performs necessary calibrations and publishes - intrinsic and extrinsic camera parameters

  ...

  Attributes
  Contains necessary attributes to perform the calibration
  ----------
  total_imgs_cam_calib : int
    attribute which decides the number of images to consider for calibrating
  
  img_diff : int
    attribute to create gap between consequent images  
  

  """

  # Counting number of images done
  count_ir = 0
  count_rgb = 0

  # 3d point in real world space
  objpoints = [] 
  objpoints_rgb = []
  objpoints_ir = []
  
  # 2D points in image plane
  imgpoints_ir = []   
  imgpoints_rgb = [] 

  # The variable to decide the number of images to read for calibrating
  total_imgs_cam_calib = 25

  # criteria for the opencv library functions 
  criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
  
  # declaring objects necessary for publishing
  pub_dist_rgb = rospy.Publisher("/cam_calib/dist_rgb", Float64MultiArray, queue_size=10)
  pub_dist_ir = rospy.Publisher("/cam_calib/dist_ir", Float64MultiArray, queue_size=10)
  pub_rbg_cam = rospy.Publisher("/cam_calib/rgb_cam_mat", Float64MultiArray, queue_size=10)
  pub_ir_cam = rospy.Publisher("/cam_calib/ir_cam_mat", Float64MultiArray, queue_size=10)
  pub_tf = rospy.Publisher("/cam_calib/tf_mat", Float64MultiArray, queue_size=10)

  def __init__(self):

    self.bridge = CvBridge()
    self.objp_rgb = np.zeros((8*5,3), np.float32)
    self.objp_ir = np.zeros((8*5,3), np.float32)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    self.objp_rgb[:,:2] = np.mgrid[0:8,0:5].T.reshape(-1,2)
    self.objp_ir[:,:2] = np.mgrid[0:8,0:5].T.reshape(-1,2)  


  def subscription(self):
    """
        Subscribes to kinenct camera nodes of topics - "k4a/rgb/image_raw" and "k4a/ir/image_raw"
        and calls "rgb_callback" and "ir_callback" functions.

        Parameters
        ----------
        None

        Returns
        -------
        None

    """
    
    rospy.Subscriber("/k4a/rgb/image_raw", Image, self.rgb_callback)
    rospy.Subscriber("k4a/ir/image_raw", Image, self.ir_callback)
    
  # function where data is read from ros, converted to image type
  def rgb_callback(self, data):
    ready_for_image = rospy.wait_for_message("/ready_for_image", Bool)
    
    if ready_for_image.data:
      try:
        
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
        if self.count_rgb<= self.total_imgs_cam_calib:
          self.find_corner_rgb(cv_image)
      
      except CvBridgeError as e:
        print(e)

  def ir_callback(self, data):
    ready_for_image = rospy.wait_for_message("/ready_for_image", Bool)
    if ready_for_image.data:
      try:
        
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
        if self.count_rgb<= self.total_imgs_cam_calib:
          self.find_corner_ir(cv_image)

      except CvBridgeError as e:
        print(e)

  """ Findng the corner points in the RGB images and storing corresponding image points"""
  def find_corner_rgb(self, imgs):

    if self.count_rgb < self.total_imgs_cam_calib:

      i = 8       #Number of points to look for along x-axis
      j = 5       #Number of points to look for along Y-axis
      
      fx = 2048   #resizing factor(x-axis) for the image
      fy = 1536   #resizing factore(y-axis) for the image
      
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
        self.objpoints_rgb.append(self.objp_rgb)
        self.objpoints.append(self.objpoints_rgb)
        corners2 = cv.cornerSubPix(img, corners, (41,41), (-1,-1), self.criteria)
        self.imgpoints_rgb.append(corners2)
        self.count_rgb = self.count_rgb + 1
        rospy.logwarn("rgb: ",self.count_rgb)      
        print("rgb: ", self.count_rgb)
           
    else:     
      if self.count_rgb and self.count_ir >= self.total_imgs_cam_calib:
        self.check_calibration()


  """ Findng the corner points in the IR images and storing corresponding image points"""
  def find_corner_ir(self, imgs):
    
    if self.count_ir < self.total_imgs_cam_calib:  
      i = 8
      j = 5

      fx_dep = 640
      fy_dep = 576

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
        self.objpoints.append(self.objp_ir)
        corners2 = cv.cornerSubPix(img, corners, (41,41), (-1,-1), self.criteria)
        self.imgpoints_ir.append(corners2)
        self.count_ir = self.count_ir + 1
        print("ir: ", self.count_ir)
        rospy.logwarn("ir: ",self.count_rgb)  

    else:
      if self.count_rgb and self.count_ir >= self.total_imgs_cam_calib:
        self.check_calibration()

  """the detected corner points are processed in this function and the new camera matrix is obtained"""
  def cam_calibration_rgb(self):
    ret, self.mtx_rgb, self.dist_rgb, self.rvecs_rgb, self.tvecs_rgb = cv.calibrateCamera(
                                                                            self.objpoints_rgb, self.imgpoints_rgb, 
                                                                            self.rgb_img_size, cameraMatrix= 0, 
                                                                            distCoeffs = 0, flags = cv.CALIB_RATIONAL_MODEL)
    
    self.h, self.w = self.rgb_img_size
    self.newcameramtx_rgb, self.roi_rgb = cv.getOptimalNewCameraMatrix(self.mtx_rgb, 
                                                                       self.dist_rgb, (self.w, self.h), 
                                                                       1, (self.w, self.h))
    

  def cam_calibration_ir(self):
    ret, self.mtx_ir, self.dist_ir, self.rvecs_ir, self.tvecs_ir = cv.calibrateCamera(
                                                                      self.objpoints_ir, self.imgpoints_ir, 
                                                                      self.ir_img_size, cameraMatrix= 0, 
                                                                      distCoeffs = 0, 
                                                                      flags = cv.CALIB_RATIONAL_MODEL)
    
    h, w = self.ir_img_size
    self.newcameramtx_ir, self.roi_ir = cv.getOptimalNewCameraMatrix(self.mtx_ir, 
                                                                     self.dist_ir, (w,h), 
                                                                     1, (w,h))

  """Undistorting and finding the error """
  def undistort_error_rectification_rgb(self):
    try:
      # undistort
      mapx, mapy = cv.initUndistortRectifyMap(self.mtx_rgb, self.dist_rgb, None, self.newcameramtx_rgb, (self.w, self.h), 5)
      dst = cv.remap(self.img, mapx, mapy, cv.INTER_LINEAR)
      # crop the image
      x_new, y_new, width, height = self.roi_rgb
      dst = dst[y_new:y_new + height, x_new:x_new + width]
      cv.imwrite('calibresult.png', dst) 

      mean_error = 0
      for i in range(len(self.objpoints_rgb)):
        imgpoints2, _ = cv.projectPoints(self.objpoints_rgb[i], self.rvecs_rgb[i], self.tvecs_rgb[i], self.mtx_rgb, self.dist_rgb)
        error = cv.norm(self.imgpoints_rgb[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
        mean_error += error
      print( "total error rgb: {}".format(mean_error/len(self.objpoints_rgb)) )
    
    except Warning:
      print("values are not set yet, please wait, trying to retrieve and recalculate")
      rospy.logwarn("values are not set yet, please wait, trying to retrieve and recalculate")

  
  def undistort_error_rectification_ir(self):
    try:
      # undistort
      mapx, mapy = cv.initUndistortRectifyMap(self.mtx_ir, self.dist_ir, None, self.newcameramtx_ir, (self.w, self.h), 5)
      dst = cv.remap(self.img, mapx, mapy, cv.INTER_LINEAR)
      # crop the image
      x_new, y_new, width, height = self.roi_ir
      dst = dst[y_new:y_new + height, x_new:x_new + width]
      cv.imwrite('calibresult.png', dst) 

      mean_error = 0
      for i in range(len(self.objpoints_ir)):
        imgpoints2, _ = cv.projectPoints(self.objpoints_ir[i], self.rvecs_ir[i], self.tvecs_ir[i], self.mtx_ir, self.dist_ir)
        error = cv.norm(self.imgpoints_ir[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
        mean_error += error
      print( "total error ir camera: {}".format(mean_error/len(self.objpoints_ir)) )

    except Warning:
      print("values are not set yet, please wait, trying to retrieve and recalculate")
      rospy.logwarn("values are not set yet, please wait, trying to retrieve and recalculate")

  
  """Calculating the extrinsic parameters between RGB and IR camera (Depth) based on the intrinsic values obtained"""
  def stereoCalibration(self):
    
    try:
      retval, camera_rgb, distcof_rgb, camera_ir, distcof_ir, Cam_rot, cam_trans, ess, fund = cv.stereoCalibrate(
                                                                              self.objpoints_rgb, self.imgpoints_rgb,
                                                                              self.imgpoints_ir, self.newcameramtx_rgb, 
                                                                              self.dist_rgb, self.newcameramtx_ir, 
                                                                              self.dist_ir, self.rgb_img_size, 
                                                                              flags = cv.CALIB_FIX_INTRINSIC + cv.CALIB_RATIONAL_MODEL, 
                                                                              criteria= self.criteria)  
      
      # Data processing for publishing:
      camera_rgb = np.asarray(camera_rgb, dtype=float)
      camera_rgb = camera_rgb.flatten()
      camera_ir = np.asarray(camera_ir, dtype=float)
      camera_ir = camera_ir.flatten()
      distcof_rgb = np.asarray(distcof_rgb, dtype=float)
      distcof_rgb = distcof_rgb.flatten()
      distcof_ir = np.asarray(distcof_ir, dtype=float)
      distcof_ir = distcof_ir.flatten()
      cam_trans = np.asarray(cam_trans, dtype=float)
      cam_tf = np.eye(4, dtype=float)

      for i in range(3):
        for j in range(3):
          cam_tf[i][j] = Cam_rot[i][j]

      cam_tf[0][3] = cam_trans[0]
      cam_tf[1][3] = cam_trans[1]
      cam_tf[2][3] = cam_trans[2]
      cam_tf = cam_tf.flatten()

      # Calling publish function
      self.pub_data(distcof_rgb, distcof_ir, camera_rgb, camera_ir, cam_tf)
      
    except Exception:
      print("All data values are not available yet, please wait, retrieving data and trying again....")
      rospy.logwarn("All data values are not available yet, please wait, retrieving data and trying again....")

  """Function to publish necessary data""" 
  def pub_data(self, dist_rgb, dist_ir, cam_rgb, cam_ir, cam_tf):
  
    dist_rgb_data = Float64MultiArray()
    dist_ir_data = Float64MultiArray()
    cam_rgb_data = Float64MultiArray()
    cam_ir_data = Float64MultiArray()
    cam_tf_data = Float64MultiArray()

    dist_rgb_data.data = dist_rgb
    dist_ir_data.data = dist_ir
    cam_rgb_data.data = cam_rgb
    cam_ir_data.data = cam_ir
    cam_tf_data.data = cam_tf

    print("Publishing...")
    rospy.logwarn("Publishing....")

    self.pub_dist_rgb.publish(dist_rgb_data)
    self.pub_dist_ir.publish(dist_ir_data)
    self.pub_rbg_cam.publish(cam_rgb_data)
    self.pub_ir_cam.publish(cam_ir_data)
    self.pub_tf.publish(cam_tf_data)


  def shut(self):
    print("Shutting down")
    rospy.logwarn("Shutting down")

  def check_calibration(self):
    if self.count_rgb and self.count_ir >= self.total_imgs_cam_calib:
      rospy.logwarn("Beginnig rgb and ir camera calibration...")
      print("Beginnig rgb and ir camera calibration...")
      self.cam_calibration_rgb()
      self.cam_calibration_ir()

      rospy.logwarn("Calculating reprojection error...")
      print("Calculating reprojection error...")
      self.undistort_error_rectification_rgb()
      self.undistort_error_rectification_ir()
      
      rospy.logwarn("Beginning stereo calibration...")
      print("Beginning stereo calibration...")
      self.stereoCalibration()
    
# beginning of the program 
def main():

  ic = calibration()
  rospy.init_node('image_converter', anonymous=True)    
  ic.subscription()
  rospy.spin()
  cv.destroyAllWindows()
  
 

if __name__ == '__main__':
    main()
    
