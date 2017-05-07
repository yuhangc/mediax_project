#!/usr/bin/env python

import rospy
import cv2, sys
import numpy as np


class IntrinsicCalibrator:
    def __init__(self):
        self.n_images = rospy.get_param("~n_images", 30)
        print self.n_images

        # width and height of the board in # of cells
        self.board_w = rospy.get_param("~board_w", 9)
        self.board_h = rospy.get_param("~board_h", 6)

        # dimension of the cell in mm
        self.cell_size = rospy.get_param("~cell_size", 25)

        # image full resolution
        res_w = rospy.get_param("~resolution_w", 2080)
        res_h = rospy.get_param("~resolution_h", 1552)

        # scaling factor
        self.scaling = rospy.get_param("~scaling", 0.5)
        self.image_size = (int(res_w * self.scaling), int(res_h * self.scaling))

        # file path, prefix, extension
        self.image_path = rospy.get_param("~image_path", "calibration/")
        self.image_prefix = rospy.get_param("~image_prefix", "Calibration_Image")
        self.image_ext = rospy.get_param("~image_ext", ".png")

        # initialize variables
        self.opts = []
        self.ipts = []

    def image_collect(self, source, n_image):
        pass

    def image_processing(self):
        # Initializing variables
        n_cells = self.board_w * self.board_h
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

        # prepare object points based on the actual dimensions of the calibration board
        # like (0,0,0), (25,0,0), (50,0,0) ....,(200,125,0)
        objp = np.zeros((n_cells, 3), np.float32)
        objp[:, :2] = np.mgrid[0:(self.board_w * self.cell_size):self.cell_size,
                      0:(self.board_h * self.cell_size):self.cell_size].T.reshape(-1, 2)

        # Loop through the images.  Find checkerboard corners and save the data to ipts.
        for i in range(1, self.n_images + 1):
            # Loading images
            image_file = self.image_path + self.image_prefix + str(i) + self.image_ext
            rospy.loginfo("Loading..." + image_file)
            print image_file
            image = cv2.imread(image_file)

            # scale the image first
            image = cv2.resize(image, None, fx=self.scaling, fy=self.scaling, interpolation=cv2.INTER_CUBIC)

            # Converting to gray scale
            grey_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

            # Find chessboard corners
            found, corners = cv2.findChessboardCorners(grey_image, (self.board_w, self.board_h),
                                                       cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            print (found)

            if found:
                # Add the "true" checkerboard corners
                self.opts.append(objp)

                # Improve the accuracy of the checkerboard corners found in the image
                # and save them to the ipts variable.
                cv2.cornerSubPix(grey_image, corners, (20, 20), (-1, -1), criteria)
                self.ipts.append(corners)

                # Draw chessboard corners
                cv2.drawChessboardCorners(image, (self.board_w, self.board_h), corners, found)

                # Show the image with the chessboard corners overlaid.
                cv2.imshow("Corners", image)
                cv2.waitKey(0)

        cv2.destroyWindow("Corners")

        rospy.loginfo('Finished processes images.')

    def calibration_and_save(self):
        rospy.loginfo('Running Calibrations...')
        rospy.loginfo('')

        ret, intrinsic_matrix, distCoeff, rvecs, tvecs = \
            cv2.calibrateCamera(self.opts, self.ipts, self.image_size, None, None)

        # Save matrices
        print('Intrinsic Matrix: ')
        print(str(intrinsic_matrix))
        print(' ')
        print('Distortion Coefficients: ')
        print(str(distCoeff))
        print(' ')

        # Save data
        rospy.loginfo('Saving data file...')
        np.savez(self.image_path + 'calibration_data', distCoeff=distCoeff, intrinsic_matrix=intrinsic_matrix)
        rospy.loginfo('Calibration complete')

        # Calculate the total reprojection error.  The closer to zero the better.
        tot_error = 0
        for i in xrange(len(self.opts)):
            imgpoints2, _ = cv2.projectPoints(self.opts[i], rvecs[i], tvecs[i], intrinsic_matrix, distCoeff)
            error = cv2.norm(self.ipts[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            tot_error += error

        print "total reprojection error: ", tot_error / len(self.opts)

        # Undistort Images
        for i in range(1, self.n_images + 1):
            # Loading images
            image_file = self.image_path + self.image_prefix + str(i) + self.image_ext
            rospy.loginfo("Loading..." + image_file)
            image = cv2.imread(image_file)

            # scale the image first
            image = cv2.resize(image, None, fx=self.scaling, fy=self.scaling, interpolation=cv2.INTER_CUBIC)

            # undistort
            dst = cv2.undistort(image, intrinsic_matrix, distCoeff, None)

            cv2.imshow('Undisorted Image', dst)
            cv2.waitKey(0)

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("camera_calibration")
    calibrator = IntrinsicCalibrator()

    calibrator.image_processing()
    calibrator.calibration_and_save()
