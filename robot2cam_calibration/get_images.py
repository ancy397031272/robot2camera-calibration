"""A file to move a UR robot with a grid and save images and robot pose for
future processing.

todo: generalize this to any robot type
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

import argparse
import datetime
import os

import robot2cam_calibration.track_grid as ci
import ur_cb2.cb2_robot as cb2_robot
import json
import time
import numpy as np
import camera
import cv2


def main():
    """
    Exposes :py:func:`get_images_poses` to the commandline. Run with arg
    `-h` for more info.
    """
    # Parse in arguments
    parser = argparse.ArgumentParser(
        description="Get images from camera and poses from UR Robot")

    parser.add_argument("--samples", type=str,
                        help='The filename for the file containing the list of'
                             'points which the robot should move through. This'
                             'file can be generated using the `cb2-record`'
                             'command from the ur_cb2 package.',
                        required=True)

    parser.add_argument("--camera", type=str,
                        help="The name of the camera to be used."
                             "Valid options are:"
                             "- `flycap`",
                        default="flycap")

    parser.add_argument("--address", type=str,
                        help="The address of the robot in form: `###.###.###`",
                        required=True)

    parser.add_argument("--port", type=int,
                        help="The port of the robot", default=30003)

    parser.add_argument("--out_folder", type=str,
                        help="Folder to save output to",
                        default="result")

    parser.add_argument("--out_file", type=str,
                        help="File to save output to",
                        default="correspondences.json")

    args = parser.parse_args()

    get_images_poses(
        robot_samples=args.samples,
        cam_name=args.camera,
        robot_address=args.address,
        robot_port=args.port,
        folder_out=args.out_folder,
        file_out=args.out_file
    )


def get_images_poses(robot_samples, cam_name,
                     robot_address, robot_port, folder_out, file_out):
    """
    Gets images from a cam_name and and poses of a UR Robot.
    Relies on pre-trained points to direct robot motion. Generates a json file
    with the pose of the robot and filename of the corresponding image.

    Args:
        robot_samples (str): The filename for the file containing the list of
                             points which the robot should move through. This
                             file can be generated using the `cb2-record`
                             command from the ur_cb2 package.
        cam_name (str): The name of the cam_name to be used.
                      Valid options are:
                        - `flycap`
        robot_address (str): The address of the robot in form: `###.###.###`
        robot_port (int): The port of the robot
        folder_out (str): The folder in which to save the data.
        file_out (str): The file in which to save all of the generated data.
    """
    with open(robot_samples, 'r') as f:
        data = json.load(f)
        write_time = data['time']
        points = data['points']
        print('read in {} points, written at: {}'.format(len(points.keys()),
                                                         write_time))

    tcp2robot = []
    im_num = 0

    if not os.path.isdir(folder_out):
        os.mkdir(folder_out)

    with cb2_robot.URRobot(robot_address, robot_port) as robot:
        with camera.Camera(cam_name) as cam:
            for number in sorted([int(x) for x in points.keys()]):
                robot.add_goal(cb2_robot.Goal(points[str(number)]['joint'],
                                              False, 'joint'))
                # TODO: this appears to skip the first point!
                robot.move_on_stop()
                print('Beginning move: {}'.format(number))

                while not (robot.at_goal() and robot.is_stopped()):
                    time.sleep(.25)
                time.sleep(.25)  # let everything settle

                with robot.receiver.lock:
                    tcp2robot.append(robot.receiver.position)
                cv2.imwrite(os.path.join(folder_out, str(im_num) + '.png'),
                            cam.capture_raw())

                im_num += 1

    print(np.asarray(tcp2robot))
    # `[x,y,z,<rotation vector>]` where `<rotation vector>` is a three element
    # vector representing the an axis about which to rotate (`<x,y,z>`) in
    # radians equal to the magnitude of the vector.

    json_dict = {"time": str(datetime.datetime.now()),
                 "tcp2robot": tcp2robot}
    with open(
            os.path.join(folder_out,
                         os.path.splitext(file_out)[0] + '.json'), 'w') as \
            result_json_file:
        json.dump(json_dict, result_json_file, indent=4)

if __name__ == '__main__':
    main()
