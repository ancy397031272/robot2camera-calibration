"""A function to track a grid and display its origin on the screen
"""

# The MIT License (MIT)
#
# Copyright (c) 2016 GTRC.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import cv2
import numpy as np
import camera
import json

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


class GridLocation(object):
    """Gets the location of a grid in an image and builds display images.

    Attributes:
        space: A float describing the spacing of the grid in mm
        rows: An int describing the number of rows of interior corners on the
            grid being tracked.
        cols: An int describing the number of columns of interior corners on
            the grid being tracked.
        opencv_windows_open: A boolean, whether the openCV display windows are
            open
        image: numpy.ndarray of the undistorted image
        result_image: numpy.ndarray of the final image, which is undistorted,
            has grid corners drawn on it, and has the grid coordinates drawn on
            it.
        object_point: numpy.ndarray of the real world coordinates of the grid
            in the grid's own coordinate system.
        axis: numpy.ndarry of the axis line points to draw, relative to the
            grid origin in the grid's coordinate system.
        intrinsic: A numpy array of the camera intrinsic matrix
        distortion: A numpy array of the camera distortion parameters
    """

    def __init__(self, calibration, rows, cols, space, cam_name):
        """Initialize the GridLocation class.

        Reads in camera calibration info, sets up communications with the
        camera, and sets up the definition for an object point.

        Args:
            calibration (str): String of the file location of the camera .
                calibration data. The data should be stored as a JSON file with
                top level fields `intrinsic` which holds the intrinsic matrix
                as a list of lists and `distortion` which holds the distortion
                matrix as a list
            rows (int): The number of rows of interior corners on the grid
            cols (int): The number of columns of interior corners on the grid
            space (float): The spacing of corners on the grid

        Raises:
            ValueError: The number of rows and cols was the same
        """
        # From args:
        self.space = space
        if rows == cols:
            raise ValueError('The grid mus be asymmetric. Rows cannot equal '
                             'Columns')
        self.rows = rows
        self.cols = cols

        self.opencv_windows_open = False

        self.image = None
        self.result_image = None

        # Grid Info:
        self.object_point = np.zeros((self.cols * self.rows, 3), np.float32)
        self.object_point[:, :2] = (np.mgrid[
                                    0:(self.rows*self.space):self.space,
                                    0:(self.cols*self.space):self.space]
                                    .T.reshape(-1, 2))
        self.axis = np.float32([[3*self.space, 0, 0], [0, 3*self.space, 0],
                                [0, 0, -3*self.space]]).reshape(-1, 3)

        # Calibration Data setup:
        with open(calibration, 'r') as calibration_file:
            calibration_dictionary = json.load(calibration_file)
        self.intrinsic = np.asarray(calibration_dictionary['intrinsic'])
        self.distortion = np.asarray(calibration_dictionary['distortion'])

        # Camera
        self.cam = camera.Camera(cam_name, self.intrinsic, self.distortion)
        print("done with init")

    def __del__(self):
        """Destroy this instance of the GridLocation class

        Closes any open OpenCV windows and closes the communications with the
        camera.
        """
        cv2.destroyWindow('result')
        self.cam.__del__()

    def show_images(self):
        """Displays the images.

        If the windows have not yet been created, they are created. Note, there
        is a programmed 5 ms delay to allow the images to be shown.
        """
        # OpenCV window and image setup:
        if not self.opencv_windows_open:
            cv2.namedWindow('result', cv2.WINDOW_NORMAL)
            self.opencv_windows_open = True
            cv2.waitKey(1)
        if self.result_image is not None:
            cv2.imshow('result', self.result_image)
        cv2.waitKey(5)

    def get_cam2grid(self):
        """Extract grid information from image and generate result image.

        Extract translation and rotation of grid from camera. Draw grid corners
        on result image. Draw grid pose on result image. Return camera to grid
        transformation matrix.

        Returns: 6 member list, translation matrix

        Raises:
            RuntimeError: Could not find a grid
        """
        # Get new image
        self.image = self.cam.capture_image()

        # Find chessboard corners.
        re_projection_error, corners = cv2.findChessboardCorners(
            self.image, (self. rows, self.cols),
            flags=cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_ADAPTIVE_THRESH)

        if not re_projection_error:
            raise RuntimeError('unable to find grid')

        corners2 = cv2.cornerSubPix(self.image, corners, (11, 11),
                                    (-1, -1),
                                    criteria)
        if corners2 is None:
            corners2 = corners

        # Find the rotation and translation vectors.
        rvecs, tvecs, inliers = cv2.solvePnPRansac(self.object_point,
                                                   corners2,
                                                   self.intrinsic,
                                                   self.distortion)
        # project 3D points to image plane
        image_points, jac = cv2.projectPoints(self.axis, rvecs, tvecs,
                                              self.intrinsic,
                                              self.distortion)

        self.result_image = cv2.cvtColor(self.image,
                                         cv2.COLOR_GRAY2RGB)

        temp_image = cv2.drawChessboardCorners(self.result_image,
                                               (self.cols, self.rows),
                                               corners2,
                                               re_projection_error)
        # OpenCV 2 vs 3
        if temp_image is not None:
            self.result_image = temp_image

        self.result_image = draw_axes(self.result_image, corners2,
                                      image_points)

        return (np.concatenate((tvecs, rvecs), axis=0)).ravel().tolist()

    def __enter__(self):
        """Content manager entry point"""
        return self

    def __exit__(self, *_):
        """Content manager exit point"""
        self.__del__()


def draw_axes(image_raw, corners, image_points, label=''):
    """Draw axes on an image

    Draw axes which will be centered at the first corner and oriented by the
    image points. Basic code from: http://docs.opencv.org/3.0-beta/doc/
                                   py_tutorials/py_calib3d/py_pose/py_pose.html

    Args:
        image_raw (numpy.ndarray): The image on which to draw the axes
        corners (numpy.ndarray): An array of 2D points on the image in which
            the first point is the origin of the axes to draw
        image_points (np.array): 2D points on the image at the end of the three
            axes
        label (str): A string label to place near the coordinate frame

    Returns: numpy.ndarray Image with the axes drawn on it.

    """
    corners = np.rint(corners).astype('int')
    image_points = np.rint(image_points).astype('int')
    corner = tuple(corners[0].ravel())
    image = image_raw.copy()
    temp = cv2.arrowedLine(image, corner, tuple(image_points[0].ravel()),
                           (255, 0, 0), 5)
    if temp is not None:
        image = temp

    letters = np.array(image_points)
    letter_space = 30

    for row in range(letters.shape[0]):
        if letters[row][0][0] < corner[0]:
            letters[row][0][0] -= letter_space
        if letters[row][0][1] < corner[1]:
            letters[row][0][1] -= letter_space
        else:
            letters[row][0][1] += 1.5*letter_space

    temp = cv2.putText(image, "x", tuple(letters[0].ravel()),
                       cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 4)
    if temp is not None:
        image = temp

    temp = cv2.arrowedLine(image, corner, tuple(image_points[1].ravel()),
                           (0, 255, 0), 5)
    if temp is not None:
        image = temp

    temp = cv2.putText(image, "y", tuple(letters[1].ravel()),
                       cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)
    if temp is not None:
        image = temp

    temp = cv2.arrowedLine(image, corner, tuple(image_points[2].ravel()),
                           (0, 0, 255), 5)
    if temp is not None:
        image = temp

    temp = cv2.putText(image, "z", tuple(letters[2].ravel()),
                       cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
    if temp is not None:
        image = temp

    # put below the axes in the middle:
    temp = cv2.putText(image, label, corner,
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    if temp is not None:
        image = temp
    return image
