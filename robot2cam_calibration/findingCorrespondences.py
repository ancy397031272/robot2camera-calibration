import numpy as np
import compute_transformations.py

intrinsic = np.matrix([[2462.345193638386, 0.0, 1242.6269086495981],[0.0, 2463.6133832534606, 1014.3609261368764],[0, 0, 1]])
distortion = np.array([-0.3954032063765203, 0.20971494160750948, 0.0008056336338866635, 9.237725225524615e-05, -0.06042030845477194])
correspondences = '../examples/correspondences_may10.json'
file_out = 'computed_transformations_may10'
#cam2rob_guess = np.array([0.,0.,0.,0.,0.,0.])
#tcp2target_guess = np.array([0.,0.,0.,0.,0.,0.])
cam2rob_guess = compute_transformations.mat2vector(np.matrix([[-.7152,.6985,-.0243,178.2],[.0412,.0074,-.9991,724.3],[-.6978,-.7155,-.0341,1515.7],[0.,0.,0.,1.]]))
tcp2target_guess = compute_transformations.mat2vector(np.matrix([[.0189,.9998,.0049,-206.5],[.9998,-.0189,-.0027,-71.1],[-.0026,.005,-1.,2.5],[0.,0.,0.,1.]]))
max_cam2rob_deviation = 2000
max_tcp2target_deviation = 500
compute_transformation(correspondences, file_out,max_cam2rob_deviation=max_cam2rob_deviation,
                                                                           max_tcp2target_deviation=max_tcp2target_deviation,intrinsic=intrinsic,distortion=distortion)
# guess = np.concatenate((cam2rob_guess, tcp2target_guess))
# with open(correspondences, 'r') as correspondences_file:
#     correspondences_dictionary = json.load(correspondences_file)
#     write_time = correspondences_dictionary['time']
#     tcp2robot = correspondences_dictionary['tcp2robot']  # nx6 array x,y,z,axis-angle
#     camera2grid = correspondences_dictionary['camera2grid']  # nx6 array x,y,z,axis-angle
# error_test = error(guess, tcp2robot, camera2grid, intrinsic, distortion)
# print(error_test)