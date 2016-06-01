# robot2camera-calibration

This library was designed to help us calibrate cameras to robots and to
calibrate difficult end of arm objects (like say a car door, which isn't really
a tool) to the robot flange. It uses error minimization to achieve this. The
error comes from the euclidean and angular difference between a measured value
and an estimate.

We have built the library to handle two primary use cases which we have:

1. Calibrating a UR robot to a flycapture2 enabled camera
2. Calibrating an odd object (currently a door) mounted on the end of a KUKA
   arm.

It would be great to have the system expanded in the future. 

Further Instructions:
[Calibrating UR arm](examples/KUKA with EBT/calibrate_check KUKA with EBT.md)
[Gathering data to calibrate KUKA](examples/KUKA with EBT/gather_data/HowToGatherData.md)
[Calibrating KUKA arm](examples/KUKA with EBT/calibrate_check KUKA with EBT.md)
