Gather Data from KUKA arm using EBT
===================================

.. note:: This README is housed in `/examples/KUKA_with_EBT/gather_data`

It is possible to calibrate a camera to a robot and calibrate the tool
offset at once using Edge Based Tracking. You will need to have `Edge
Based
Tracking <https://github.com/CognitiveRobotics/Peugeot/tree/TRL5>`__ and
our `KUKA-RSI <https://github.gatech.edu/msobrepera3/KUKA-RSI>`__
interface.

You will need to setup your KUKA to talk to the KUKA-RSI interface. Then
load a program with a series of diversified points onto the KUKA (like
those in the tp\_files directory) which talks over the RSI interface and
stops for a few seconds at each point.

You then need to setup EBT to track your object with the network
interface on.

Run The KUKA-RSI interface, then then EBT, then start the program on the
KUKA, then run ``python RSI_EBT_LOG.py``. Allow the program on the KUKA
to run. Then stop the RSI\_EBT\_LOG, EBT, and KUKA-RSI interface. A log
file should now exist in the ``gather_data`` folder (like
``2016-5-31_15-38-12.txt``). I have also run the routine without EBT
running and captured images everytime the robot stops for
demonstrationand validation purposes, they are stored in ``Images``.
Note, they aren't undistorted, so not perfect.

The next step is to extract the useful data from the log file. This is
done using ``extract_for_calib.m`` within matlab. Pass in the log file,
a series of figures showing the results will generated as well as a json
file (ex: ``2016-5-31_15-38-12.json``.

You know have everything you need to run the calibration routines!!.
