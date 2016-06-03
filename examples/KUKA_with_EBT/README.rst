Calibrate and Verify KUKA arm
=============================

.. note:: This README is housed in `/examples/KUKA_with_EBT`

In order to calibrate a KUKA robot using EBT, you will first have to
gather the appropriate data. For this, read ``HowToGatherData.md`` in
the ``gather_data`` directory.

First install the software by running "sudo python setup.py install"
(or install in a virtual environment)

Once you have gathered the appropriate data, edit the
``calibrate+check`` script as needed and run it. Or run
robot2cam-compute and robot2cam-check from the commandline. Images
showing the calibration results should be placed into ``output_images``
and a file detailing the calibration results should be stored as
``transformation.json``
