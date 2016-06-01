"""A file to move a UR robot with a grid and save correspondences between
camera and robot pose for future processing.

todo: generalize this to other robot types
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


def main():
    """
    Exposes :py:func:`get_correspondences` to the commandline. Run with arg
    `-h` for more info.
    """
    # Parse in arguments
    parser = argparse.ArgumentParser(
        description="Get correspondences between camera and UR Robot",
        epilog="Gets correspondences between a camera and UR Robot with a "
               "grid attached. Relies on pre-trained points to direct "
               "robot motion. Will try to find the grid 5 times per "
               "position. Generates a json file with all of the data "
               "needed to then calculate the tool offset and the camera "
               "to robot transformation.")

    parser.add_argument("--samples", type=str,
                        help='The filename for the file containing the list of'
                             'points which the robot should move through. This'
                             'file can be generated using the `cb2-record`'
                             'command from the ur_cb2 package.',
                        required=True)

    parser.add_argument("-s", "--spacing", type=float,
                        help="The grid spacing in mm.", required=True)

    parser.add_argument("-c", "--columns", type=int,
                        help="the number of inner corners horizontally",
                        required=True)

    parser.add_argument("-r", "--rows", type=int,
                        help="the number of inner corners vertically",
                        required=True)

    parser.add_argument("--calibration", type=str,
                        help="The filename of the camera calibration "
                             "information. This file can be generated using "
                             "the`calibrate-camera` command from the "
                             "camera-calibration toolbox.",
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

    parser.add_argument("--out", type=str,
                        help="File to save output to",
                        default="correspondences.json")

    args = parser.parse_args()

    get_correspondences(
        robot_samples=args.samples,
        calibration=args.calibration,
        rows=args.rows,
        cols=args.columns,
        spacing=args.spacing,
        camera=args.camera,
        robot_address=args.address,
        robot_port=args.port,
        file_out=args.out
    )


def get_correspondences(robot_samples, calibration, rows, cols, spacing,
                        camera, robot_address, robot_port, file_out):
    """
    Gets correspondences between a camera and UR Robot with a grid attached.
    Relies on pre-trained points to direct robot motion. Will try to find the
    grid 5 times per position. Generates a json file with all of the data
    needed o then calculate the tool offset and the camera to robot
    transformation.

    Args:
        robot_samples (str): The filename for the file containing the list of
                             points which the robot should move through. This
                             file can be generated using the `cb2-record`
                             command from the ur_cb2 package.
        calibration (str): The filename of the camera calibration information.
                           This file can be generated using the
                           `calibrate-camera` command from the
                           camera-calibration toolbox.
        rows (int): The number of rows on the grid which is attached to the
                    robot
        cols (int): The number of columns on the grid which is attached to the
                    robot
        spacing (float): The spacing in mm between grid corners on the grid
                         which is attached to the robot
        camera (str): The name of the camera to be used.
                      Valid options are:
                        - `flycap`
        robot_address (str): The address of the robot in form: `###.###.###`
        robot_port (int): The port of the robot
        file_out (str): The file in which to save all of the generated data.
    """
    with open(robot_samples, 'r') as f:
        data = json.load(f)
        write_time = data['time']
        points = data['points']
        print('read in {} points, written at: {}'.format(len(points.keys()),
                                                         write_time))

    camera2grid = []
    tcp2robot = []

    with ci.GridLocation(calibration, rows, cols, spacing, camera) as calib:
        with cb2_robot.URRobot(robot_address, robot_port) as robot:
            for number in sorted([int(x) for x in points.keys()]):
                robot.add_goal(cb2_robot.Goal(points[str(number)]['joint'],
                                              False, 'joint'))
                # TODO: this appears to skip the first point!
                robot.move_on_stop()
                print('Beginning move: {}'.format(number))

                while not (robot.at_goal() and robot.is_stopped()):
                    time.sleep(.25)
                time.sleep(.25)  # let everything settle
                print("reached goal")
                go_on = 0
                while go_on <= 5:
                    try:
                        camera2grid.append(calib.get_cam2grid())
                        calib.show_images()
                        with robot.receiver.lock:
                            tcp2robot.append(robot.receiver.position)
                        print("got the grid")
                        go_on = 6
                    except RuntimeError as e:
                        print("something went wrong: {}".format(e))
                        go_on += 1
    tcp2robot = np.array(tcp2robot)
    tcp2robot[:, 0:3] = tcp2robot[:, 0:3] * 1000
    tcp2robot = tcp2robot.tolist()

    print(np.asarray(tcp2robot))  # Axis-Angle [x,y,z,ax,ay,az]
    print(np.asarray(camera2grid))
    json_dict = {"grid": {"rows": rows,
                          "cols": cols,
                          "spacing": spacing},
                 "time": str(datetime.datetime.now()),
                 "calibration": calibration,
                 "tcp2robot": tcp2robot,
                 "camera2grid": camera2grid}
    with open(os.path.splitext(file_out)[0] + '.json', 'w') as \
            result_json_file:
        json.dump(json_dict, result_json_file, indent=4)

if __name__ == '__main__':
    main()
