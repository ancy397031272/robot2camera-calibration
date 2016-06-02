"""A file to wrap up various cameras into a common class.

Currently this houses:
    - Flycapture2

TODO: Add some more:
    - webcam
    - folder of images
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
import threading

import numpy as np
import cv2


class Camera(object):
    """Wraps various camera and image capture technologies.

    Supported Cameras:
    
    - Flycapture2 devices : `flycap`, `flycap2`, `flycapture`, `flycapture2`

    Attributes:
        intrinsic: A numpy array of the camera intrinsic matrix
        distortion: A numpy array of the camera distortion parameters
        cam: A camera or other image acquisition device, which this class wraps.
    """
    def __init__(self, name, intrinsic=None, distortion=None):
        """Sets up camera acquisition and reads calibration data.

        Args:
            name (str): Name of the camera to use
            intrinsic (numpy.ndarray): The camera intrinsic matrix
            distortion (numpy.ndarray): The camera distortion parameters

        Raises:
            NotImplementedError: The camera type selected is not yet implemented
            Value Error: The value entered for the camera is not valid.
        """
        if name.lower() in ('flycap', 'flycap2', 'flycapture', 'flycapture2'):
            self.cam = FlyCap2()
        elif name.lower() in ('wc', 'webcam'):
            raise NotImplementedError
        elif name.lower() in ('file', 'files', 'folder', 'pictures', 'picture',
                              'image', 'images'):
            raise NotImplementedError
        else:
            raise ValueError('unknown camera type')

        self.intrinsic = intrinsic
        self.distortion = distortion

    def capture_image(self):
        """Capture an and rectify an image.

        Returns: The newly captured image, rectified, as a numpy array.

        Raises:
            RuntimeError: It was not possible to capture and rectify an image
        """
        raw_image = self.cam.capture_image()
        if self.intrinsic is not None and self.distortion is not None:
            rectified_image = cv2.undistort(raw_image, self.intrinsic,
                                            self.distortion)
        else:
            rectified_image = raw_image
        if rectified_image is None:
            raise RuntimeError("Unable to capture and rectify image")
        return rectified_image

    def capture_raw(self):
        """ Return the raw (still distorted) image.

        Returns: The most recent raw image as a numpy array
        """
        return self.cam.capture_image()

    def __del__(self):
        self.cam.__del__()

    def __enter__(self):
        """Enters the camera from a with statement"""
        return self

    def __exit__(self, *_):
        """Exits at the end of a context manager statement by destructing."""
        self.__del__()


class FlyCap2(object):
    """A wrapper to capture images from a flycapture2 camera

    Attributes:
        context: flycapture2.Context of the camera context
        fc2_image: flycapture2.Image which represents the image buffer of the
            camera images
    """
    def __init__(self):
        """Setup the communications with the flycapture2 device."""
        import flycapture2 as fc2

        # FlyCapture Info printing and setup:
        print "\n\nlibrary version: {}\n".format(fc2.get_library_version())
        self.context = fc2.Context()
        print "Number of Cameras: {}\n".format(
            self.context.get_num_of_cameras())
        self.context.connect(*self.context.get_camera_from_index(0))
        print "Camera Info: {}\n".format(self.context.get_camera_info())
        m, f = self.context.get_video_mode_and_frame_rate()
        print "Video Mode: {}\nFrame Rate:{}\n".format(m, f)
        print "Frame Rate Property Info: {}\n".format(
            self.context.get_property_info(fc2.FRAME_RATE))
        p = self.context.get_property(fc2.FRAME_RATE)
        print "Frame Rate Property: {}\n".format(p)
        self.context.set_property(**p)
        self.context.start_capture()
        self.fc2_image = fc2.Image()
        print "done with flycap2 setup"
        self.cam_on = True

        self.image = None
        cv2.namedWindow('raw', cv2.WINDOW_NORMAL)
        cv2.waitKey(5)

        self.__acquisition_thread = None
        self.lock = threading.Lock()

        self.run = True
        self.__acquisition_thread = threading.Thread(group=None,
                                                     target=self.acquire,
                                                     name='acquisition_thread',
                                                     args=(),
                                                     kwargs={})
        self.__acquisition_thread.start()

    def __del__(self):
        """Shutdown the communications with the flycapture2 device."""
        self.stop()
        if self.cam_on:
            print("flycap cam already disconnected")
        else:
            self.context.stop_capture()
            self.context.disconnect()
        cv2.destroyWindow('raw')

    def capture_image(self):
        """Return the latest image from the camera

        Returns: np.array of the latest image from the camera.
        """
        with self.lock:
            return np.copy(self.image)

    def stop(self):
        if self.__acquisition_thread is not None:
            if self.__acquisition_thread.is_alive():
                print("shutting down acquisition thread")
                self.run = False
                self.__acquisition_thread.join()
                if self.__acquisition_thread.is_alive():
                    print('failed to shutdown auxiliary thread')
                else:
                    print('shutdown auxiliary thread')
            else:
                print('auxiliary thread already shutdown')
        else:
            print('no auxiliary threads exist')

    def acquire(self):
        while self.run:
            with self.lock:
                self.image = np.array(self.context.retrieve_buffer(self.fc2_image))
            cv2.imshow('raw', self.image)
            cv2.waitKey(5)


