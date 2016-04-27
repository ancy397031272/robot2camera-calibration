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

    object_point = np.zeros((cols * rows, 3), np.float32)
    object_point[:, :2] = (np.mgrid[
                                0:(rows * space):space,
                                0:(cols * space):space]
                                .T.reshape(-1, 2))

    number_found = 0

    # Change rotation matrix into rotation vector
    Rvec, jac = cv2.Rodrigues(Rotm)

    for image_file in file_names:
        image_file = os.path.join(target_directory, image_file)

        # Try to read in image as gray scale
        img = cv2.imread(image_file, 0)

        # If the image_file isn't an image, move on
        if img is not None:
            image_points = cv2.projectPoints(object_point, Rvec, Tvec,
                                             intrinsic,
                                             distortion)
            for i in range(0,len(image_points)):
                cv2.circle(img,tuple(image_points[i]),3,[100,200,100])
            cv2.imwrite(os.path.join(ResultFolder, "result" + str(number_found) + ".jpg"), img)
            number_found += 1



Rotm = np.matrix([[-.50634,.74773,-.42956],[.85721,.49064,-.15638],[.09383,-.44741,-.88939]])
Tvec = np.array([.41163,.25832,1])
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