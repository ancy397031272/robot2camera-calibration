Calibrate UR Arm
================

.. note:: This README is housed in `/examples/UR_with_Grid`

It is very easy to calibrate a UR Robot to a point grey camera using a
grid. Hopefully, in the future, this can be extended to other hardware
as well.

In order to do this, you will will need to have
`pyflycapture2 <https://github.com/jordens/pyflycapture2>`__ as well as
our `ur-cb2 <https://pypi.python.org/pypi/ur_cb2>`__ package.

The first step is to attach an asymetric grid to the flange of the UR.
Then save a series of robot poses in which the camera can see the grid
using ``cb2-record``. This will generate a series of points such as
``cb2points.json``.

You then need to run the ur through the points and capture data. This is
done using the ``robot2cam-record-ur`` script.

.. note:: We have seen problems with pyflycapture2 when using a virtual environment

If you would like to have images to use for validation, those can be gathered 
using the ``robot2cam-images-ur`` command. You will then need to use
``robot2cam-compute`` to calculate the transformations. Finally, you can
optionally use ``robot2cam-check`` to visualize the results. All of this
is packages into ``calibrate_ur``.
