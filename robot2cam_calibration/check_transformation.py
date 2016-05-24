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
import re


def check_transformation(supermatrix, ImageFolder, ResultFolder, rows, cols, space, intrinsic, distortion):
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

    axis_length = 250
    axis = np.float32([[0,0,0],[axis_length, 0, 0], [0, axis_length, 0],
                                [0, 0, axis_length]]).reshape(-1, 3)

    number_found = 0


    # Change rotation matrix into rotation vector
    # for i in range(Rotm.shape[0]):
    #     rvec, jac = cv2.Rodrigues(Rotm[i])
    #     image_points[i], jac = cv2.projectPoints(axis, rvec, Tvec[i], intrinsic,
    #                                              distortion)
    for image_file in sort_nicely(file_names):
        image_file = os.path.join(target_directory, image_file)

        # Try to read in image as gray scale
        img = cv2.imread(image_file, 0)

        # If the image_file isn't an image, move on
        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            for j in range(supermatrix[number_found].shape[0]):
                cam2target = supermatrix[number_found][j]
                rvec, jac = cv2.Rodrigues(cam2target[0:3,0:3])
                image_points, jac = cv2.projectPoints(axis, rvec, cam2target[0:3,3],
                                                         intrinsic,
                                                         distortion)
                img = track_grid.draw_axes(image_raw=img, corners=image_points[0],
                                               image_points=image_points[1:])
            # for i in range(0,len(image_points)):
            #     cv2.circle(img,tuple(image_points[i]),3,[100,200,100])
            cv2.imwrite(os.path.join(ResultFolder, "result" + str(number_found) + ".jpg"), img)
            print("finished processing Image {}".format(image_file))
            number_found += 1
    print("Done processing all images")


# http://stackoverflow.com/questions/4623446/how-do-you-sort-files-numerically
def tryint(s):
    try:
        return int(s)
    except:
        return s

def alphanum_key(s):
    """ Turn a string into a list of string and number chunks.
        "z23a" -> ["z", 23, "a"]
    """
    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def sort_nicely(l):
    """ Sort the given list in the way that humans expect.
    """
    return sorted(l, key=alphanum_key)
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
# Rotm = np.array([
#     [[0.977, -0.202, -0.063],[0.183, 0.957, -0.223],[0.106, 0.206, 0.973]],
#     [[0.977, -0.202, -0.063],[0.183, 0.957, -0.223],[0.106, 0.206, 0.973]]
# ])
# Tvec = np.array([
#     [0.047, 0.081, 2.221],
#     [0.7170, 0.2065, 2.2937]
# ])

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

# Rotm = np.array([
#     [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
#     [[-0.707, 0.707, 0.0], [0.0, 0.0, -1.0], [-0.707, -0.707, 0.0]],
#     [[0.1419, -0.5214, -0.8412], [-0.8623, -0.4826, 0.1536 ], [-0.4861, 0.7036, -0.5181]],
#     [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
# ])
# Tvec = np.array([
#     [0, 0, 0],
#     [0.2, 0.75, 2.0],
#     [0.3517,   -0.1479,   1.2312],
#     [0.1517,   -0.8979,   -0.3688]
# ])

# row1: grid from track grid
# row2: door from EBT
# row3: approx tcp from mjs guess: row1*tcp
# row4: inverse transform to base via row3:
# row5: estimate from calibration software

# Rotm = np.array([
#     [[-0.00500405, 0.99998434, 0.0025046],
#      [-0.9306746, -0.00374094, -0.36582892],
#      [-0.36581382, -0.00416159, 0.93067875]],
#     [[0.977, -0.178, -0.121],
#      [0.136, 0.946, -0.294],
#      [0.167, 0.270, 0.948]],
#     [[- 0.00500405, -0.99998434, -0.00250460000000012],
#      [- 0.9306746, 0.00374093999999996, 0.36582892],
#      [- 0.36581382, 0.00416159000000011, -0.93067875]],
#     [[-0.659927435608819, 0.751329313735808, -0.0001893009399004],
#      [0.199371042478852, 0.174873921787055, -0.964194121183759],
#      [-0.72439420933552, -0.636335894927072, -0.265197405236951]],
#     np.eye(3)
# ])
# Tvec = np.array([
#     [151.48231455772967, -45.84279051621655, 1140.7020382437486],
#     [-38.0, -202.0, 2235.0],
#     [250.55, -218.392, 1072.610],
#     [75.79, 552.50, 1656.20],
#     [154.61, 156.377, 2032.33]
# ])

supermatrix2 = np.array([[
#     [[-1.25812084e-02, 9.90265918e-01, 1.38618629e-01, 1.77788714e+02],
#      [-9.57905818e-01, 2.78268728e-02, -2.85730834e-01, 5.11075809e+01],
#      [-2.86806830e-01, -1.36378430e-01, 9.48231389e-01, 1.10263438e+03],
#      [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],
#     [[-7.15156933e-01,   6.98541126e-01,  -2.43075326e-02,   1.78200000e+02],
#     [  4.12029080e-02,   7.41591789e-03,  -9.99123278e-01,   0.24300000e+02],
#     [ -6.97748437e-01,  -7.15531481e-01,  -3.40854708e-02,   1.51570000e+03],
#     [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]],
#     [[-.7152,.6985,-.0243,178.2],
#      [.0412,.0074,-.9991,724.3],
#      [-.6978,-.7155,-.0341,1515.7],
#      [0.,0.,0.,1.]],
#     [[-8.36255661e-01,   5.15571159e-01,   1.86715960e-01,   3.03700162e+02],
#     [ -7.09566325e-02,   2.35901562e-01,  -9.69182960e-01,  -1.63741761e+02],
#     [ -5.43729368e-01,  -8.23733472e-01,  -1.60690823e-01,   1.11456182e+03],
#     [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]],
#     [[ 4.99180183e-01,  -8.44908180e-01,  -1.92222039e-01,   4.40196637e+02],
#     [  2.37040237e-01,  -8.02218864e-02,   9.68181995e-01,  -1.68284775e+02],
#     [ -8.33445301e-01,  -5.28861623e-01,   1.60232059e-01,   1.28500766e+03],
#     [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]
#     [[-7.04033421e-01,   7.09692924e-01,  -2.59402374e-02,   1.51836794e+02],
#     [  3.29330487e-02,  -3.86083310e-03,  -9.99450103e-01,   7.55218885e+02],
#     [ -7.09402816e-01,  -7.04500567e-01,  -2.06541961e-02,   1.46384759e+03],
#     [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]],
#     [[ -3.65398965e-02,  -9.97544943e-01,  -5.97404705e-02,   2.81701946e+02],
#      [ -9.91340139e-01,   2.86375188e-02,   1.28158581e-01,  -1.38574698e+02],
#      [ -1.26133126e-01,   6.39060288e-02,  -9.89952753e-01,   1.07700385e+03],
#      [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]
        [[ -7.03205968e-01,   7.10505539e-01,  -2.61389626e-02,   1.54349198e+02],
         [  3.20817523e-02,  -5.01769281e-03,  -9.99472653e-01,   7.47408195e+02],
         [ -7.10262014e-01,  -7.03673718e-01,  -1.92657912e-02,   1.46924939e+03],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]],
        [[  2.42847887e-02,   9.97325536e-01,   6.89349222e-02,   1.80946705e+02],
         [ -9.91843649e-01,   3.26670859e-02,  -1.23203236e-01,   5.09704076e+01],
         [ -1.25125636e-01,  -6.53807002e-02,   9.89984313e-01,   1.11082496e+03],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]


]])



# grid2tcp = np.array(
#     [[1.0000,0,0,185.0000],
#      [0,-1.0000,-0.0000,100.0000],
#      [0,0.0000,-1.0000,0],
#      [0,0,0,1.0000]]
# )

# with open('../examples/gridFinding.json', 'r') as grid_json:
#     json_dictionary = json.load(grid_json)
#
# cam2grid = np.array(json_dictionary['camera2grid'])
# base2tcp = np.array(json_dictionary['tcp2robot'])
#
# cam2gridT = np.zeros((cam2grid.shape[0], 4, 4))
# base2tcpT = np.zeros((cam2grid.shape[0], 4, 4))
#
# for i in range(cam2grid.shape[0]):
#     [cam2gridRot, _] = cv2.Rodrigues((cam2grid[i][3:]))
#     translation = np.array([cam2grid[i][:3]])
#     cam2gridT[i] = np.concatenate((np.concatenate((cam2gridRot, translation.T), axis=1),np.array([[0,0,0,1]])),axis=0)
#
# for i in range(cam2grid.shape[0]):
#     [cam2tcpRot, _] = cv2.Rodrigues((base2tcp[i][3:]))
#     translation = np.array([base2tcp[i][:3]])*1000
#     base2tcpT[i] = np.concatenate((np.concatenate((cam2tcpRot, translation.T), axis=1),np.array([[0,0,0,1]])),axis=0)
#
# cam2tcpT = np.matmul(cam2gridT,grid2tcp)
#
# tcp2baseT = np.zeros(base2tcpT.shape)
# for i in range(base2tcpT.shape[0]):
#     tcp2baseT[i] = np.linalg.inv(base2tcpT[i])
#
# cam2baseT = np.matmul(cam2tcpT,tcp2baseT)
#
# supermatrix = np.zeros((cam2baseT.shape[0],3,4,4))
# supermatrix[:,0] = cam2baseT
# supermatrix[:,1] = cam2tcpT
# supermatrix[:,2] = cam2gridT

ImageFolder = 'Images1'
ResultFolder = 'Result'
rows = 7
cols = 8
space = 25.4
intrinsic = np.array([[2462.345193638386,0.0,1242.6269086495981],[0.0,2463.6133832534606,1014.3609261368764],
                     [0.0,0.0,1.0]])
distortion = np.array([[-0.3954032063765203,0.20971494160750948,0.0008056336338866635,9.237725225524615e-05,
                       -0.06042030845477194]])
check_transformation(supermatrix2,ImageFolder,ResultFolder,rows,cols,space,intrinsic,distortion)