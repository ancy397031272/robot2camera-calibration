"""A function to move around a robot and capture images from it.

This assumes that you are using a UR CB2 robot and a Flycapture2 camera. This
could certainly be made better by generalizing to other platforms.

Copyright (c) 2016 GTRC.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import cv2
import flycapture2 as fc2
import ur_cb2
import numpy as np


class Calibrator(object):

    def __init__(self, calibration, ):
        print "\n\nlibrary version: {}\n".format(fc2.get_library_version())
        self.context = fc2.Context()
        print "Number of Cameras: {}\n".format(self.context.get_num_of_cameras())
        self.context.connect(*self.context.get_camera_from_index(0))
        print "Camera Info: {}\n".format(c.get_camera_info())
        m, f = self.context.get_video_mode_and_frame_rate()
        print "Video Mode: {}\nFrame Rate:{}\n".format(m, f)
        print "Frame Rate Property Info: {}\n".format(
            self.context.get_property_info(fc2.FRAME_RATE))
        p = self.context.get_property(fc2.FRAME_RATE)
        print "Frame Rate Property: {}\n".format(p)
        self.context.set_property(**p)
        self.context.start_capture()
        self.fc2_image = fc2.Image()
        cv2.namedWindow('raw', cv2.WINDOW_NORMAL)
        cv2.namedWindow('undistort', cv2.WINDOW_NORMAL)
        cv2.namedWindow('grid', cv2.WINDOW_NORMAL)
        frame = 0
        self.raw_image = None
        self.undistort_image = None
        self.grid_image = None

    def capture_image(self):
        """Capture an image"""
        self.raw_image = np.array(self.context.retrieve_buffer(self.fc2_image))

    def show_images(self):
        cv2.imshow('raw',self.raw_image)
        cv2.imshow('')