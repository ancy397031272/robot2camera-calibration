robot2camera-calibration
========================

.. image:: https://readthedocs.org/projects/robot2camera-calibration/badge/?version=latest
   :target: http://robot2camera-calibration.readthedocs.io/en/latest/?badge=latest
   :alt: Documentation Status

This library was designed to help us calibrate cameras to robots and to
calibrate difficult end of arm objects (like say a car door, which isn't
really a tool) to the robot flange. It uses error minimization to
achieve this. The error comes from the euclidean and angular difference
between a measured value and an estimate.

We have built the library to handle two primary use cases which we have:

1. Calibrating a UR robot to a flycapture2 enabled camera
2. Calibrating an odd object (currently a door) mounted on the end of a
   KUKA arm.

It would be great to have the system expanded in the future.

Installing:
-----------
You have two options:

1. ``pip install robot2cam_calibration``
2. Clone the repository and run ``python setup.py install``
