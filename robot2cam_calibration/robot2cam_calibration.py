"""A script to calibrate a robot to a camera.

Basic Usage: Store points using cb2_store_points.py (cb2-record from the
terminal). Run this script, with commandline args.

Copyright (c) 2016 GTRC. All rights reserved.
"""

import argparse
import ur_cb2.cb2_robot as cb2_robot
import json
import time
import flycapture2 as fc2
import numpy as np
import cv2


def main():
    # Parse in arguments
    parser = argparse.ArgumentParser(
        description='Save Points',
        epilog="This software is designed to move a cb2 robot to points which "
               "have been previously saved.")

    parser.add_argument("-f", "--file", metavar="file", type=str,
                        help='The file to read data from.',
                        default="cb2points.json")

    parser.add_argument("--ip", metavar="ip", type=str,
                        help='IP address of the robot', default="192.168.1.100")

    parser.add_argument("--port", metavar="port", type=int,
                        help='IP port on the robot', default=30003)

    parser.add_argument("--cam", metavar="camera calibration", type=string,
                        help='The file describing the camera calibration. This '
                             'should be a json file with ',
                        requried=True)

    args = parser.parse_args()
    host = args.ip    # The remote host
    port = args.port  # The same port as used by the server

def calibrate(file, host, port, cam):
    """Calibrate a robot to a camera.

    Currently this assumest that you are using a CB2

    Args:
        file:
        host:
        port:
        cam:

    Returns:

    """
    with open(file, 'r') as f:
        data = json.load(f)
    write_time = data['time']
    points = data['points']
    print 'read in {} points, written at: {}'.format(len(points.keys()),
                                                     write_time)

    print "\n\nlibrary version: {}\n".format(fc2.get_library_version())
    c = fc2.Context()
    print "Number of Cameras: {}\n".format(c.get_num_of_cameras())
    c.connect(*c.get_camera_from_index(0))
    print "Camera Info: {}\n".format(c.get_camera_info())
    m, f = c.get_video_mode_and_frame_rate()
    print "Video Mode: {}\nFrame Rate:{}\n".format(m, f)
    print "Frame Rate Property Info: {}\n".format(c.get_property_info(fc2.FRAME_RATE))
    p = c.get_property(fc2.FRAME_RATE)
    print "Frame Rate Property: {}\n".format(p)
    c.set_property(**p)
    c.start_capture()
    im = fc2.Image()
    cv2.namedWindow('raw', cv2.WINDOW_NORMAL)
    cv2.namedWindow('undistorted', cv2.WINDOW_NORMAL)
    cv2.namedWindow('grid', cv2.WINDOW_NORMAL)
    frame = 0

    with cb2_robot.URRobot(host, port) as robot:
        for number in sorted([int(x) for x in points.keys()]):
            robot.add_goal(cb2_robot.Goal(points[str(number)]['joint'], False,
                                          'joint'))
            # TODO: this appears to skip the first point!
            robot.move_on_stop()
            print 'Beginning move: {}'.format(number)
            while not (robot.is_stopped() and robot.at_goal()):
                time.sleep(.1)
            time.sleep(.1)  # make sure that everything is settled for a crisp
                            # shot
            a = np.array(c.retrieve_buffer(im))
            cv2.imshow('raw', a)
            dst = cv2.undistort(a, mtx, dist, None)
            cv2.imshow('undistorted', dst)
            print "captured image {}".format(frame)
            frame += 1

    c.stop_capture()
    c.disconnect()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
