"""A file to visually check the result of camera coordinate to robot base transformation

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
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import cv2
import os
import numpy as np
import camera
import json
import track_grid


def check_transformation(Rotm, Tvec, ImageFolder, ResultFolder, rows, cols, space, intrinsic, distortion):
    """Plots transformed 3D world points onto camera image

            Args:
                Rotm (numpy.array): The rotation matrix from the robot base to camera coordinates
                Tvec (numpy.array): The translation vector from the robot base to camera coordinates
                Robot_Poses (str): Name of json file containing robot poses
                ImageFolder (str): The name of the folder to read images from
                ResultFolder (str): The name of the folder to save the output images in
                rows (int): The number of rows in the calibration grid
                cols (int): the number of columns in the calibration grid
                space (float): the calibration grid spacing in mm
                intrinsic (numpy.array): The intrinsic matrix of the camera
                distortion (numpy.array): The distortion matrix of the camera

            """
    if len(ResultFolder) and (ResultFolder[0] == '/' or ResultFolder[0] == '\\'):
        ResultFolder = ResultFolder[1:]
    if len(ResultFolder) and (ResultFolder[-1] == '/' or ResultFolder[-1] == '\\'):
        ResultFolder = ResultFolder[:-1]

    if len(ImageFolder) and (ImageFolder[0] == '/' or ImageFolder[0] == '\\'):
        ImageFolder = ImageFolder[1:]
    if len(ImageFolder) and (ImageFolder[-1] == '/' or ImageFolder[-1] == '\\'):
        ImageFolder = ImageFolder[:-1]

    target_directory = os.path.join(os.getcwd(), ImageFolder)
    directory_out = os.path.join(os.getcwd(), ResultFolder)
    file_names = os.listdir(target_directory)
    if not os.path.exists(directory_out):
        os.makedirs(directory_out)

    axis_length = .1
    axis = np.float32([[0,0,0],[axis_length, 0, 0], [0, axis_length, 0],
                                [0, 0, axis_length]]).reshape(-1, 3)

    number_found = 0
    image_points = np.zeros((Rotm.shape[0], 4, 1, 2))
    # Change rotation matrix into rotation vector
    for i in range(Rotm.shape[0]):
        rvec, jac = cv2.Rodrigues(Rotm[i])
        image_points[i], jac = cv2.projectPoints(axis, rvec, Tvec[i], intrinsic,
                                                 distortion)
    for image_file in file_names:
        image_file = os.path.join(target_directory, image_file)

        # Try to read in image as gray scale
        img = cv2.imread(image_file, 0)

        # If the image_file isn't an image, move on
        if img is not None:
            
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            for i in range(image_points.shape[0]):
                img = track_grid.draw_axes(image_raw=img, corners=image_points[i][0],
                                           image_points=image_points[i][1:])
            # for i in range(0,len(image_points)):
            #     cv2.circle(img,tuple(image_points[i]),3,[100,200,100])
            cv2.imwrite(os.path.join(ResultFolder, "result" + str(number_found) + ".jpg"), img)
            number_found += 1
            print("finished processing Image {}".format(number_found))
    print("Done processing all images")


# UR
# Rotm = np.matrix([[-.50634,.74773,-.42956],[.85721,.49064,-.15638],[.09383,-.44741,-.88939]])
# Tvec = np.array([.41163,.25832,1])

# Test
# Rotm = np.matrix([[1.000, 0.015, 0.001],[-0.015, 0.991, 0.131],[0.001, -0.131, 0.991]])
# Tvec = np.array([-0.355, 0.046, 2.189])
#
# Test2
# [0.977 -0.202 -0.063 0.047]
# [0.183 0.957 -0.223 0.081]
# [0.106 0.206 0.973 2.221]
# [0.000 0.000 0.000 1.000]
# Rotm = np.matrix([[0.977, -0.202, -0.063],[0.183, 0.957, -0.223],[0.106, 0.206, 0.973]])
# Tvec = np.array([0.047, 0.081, 2.221])

# with axis angle
#Rotm = np.matrix([[-0.0010,   -0.9984,   -0.0558],[0.9995,    0.0009,   -0.0328],[0.0328,   -0.0558,    0.9979]])
#Tvec = np.array([0.0554,   -0.2315,   -0.0225])

# manual guess
#Rotm = np.matrix([[-0.707,   0.707,   0.0],[0.0,   0.0,   -1.0],[-0.707,   -0.707,    0.0]])
#Tvec = np.array([0.1,   0.35,   1.0])

# measure manual guess
# Rotm = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
# Rotm = np.matrix([[-0.707,   0.707,   0.0],[0.0,   0.0,   -1.0],[-0.707,   -0.707,    0.0]])
# Tvec = np.array([0.2,   0.75,   2.0])

Rotm = np.array([
    [[-0.707, 0.707, 0.0], [0.0, 0.0, -1.0], [-0.707, -0.707, 0.0]],
    [[0.1419, -0.5214, -0.8412], [-0.8623, -0.4826, 0.1536 ], [-0.4861, 0.7036, -0.5181]]
])
Tvec = np.array([
    [0.2, 0.75, 2.0],
    [0.3517,   -0.1479,   1.2312]
])


ImageFolder = 'Images'
ResultFolder = 'Result'
rows = 7
cols = 8
space = 25.4
intrinsic = np.array([[2462.345193638386,0.0,1242.6269086495981],[0.0,2463.6133832534606,1014.3609261368764],
                     [0.0,0.0,1.0]])
distortion = np.array([[-0.3954032063765203,0.20971494160750948,0.0008056336338866635,9.237725225524615e-05,
                       -0.06042030845477194]])
check_transformation(Rotm,Tvec,ImageFolder,ResultFolder,rows,cols,space,intrinsic,distortion)