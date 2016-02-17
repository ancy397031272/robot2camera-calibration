import robot2cam_calibration.track_grid as ci

with ci.GridLocation("result.json", 7, 8, 25.4) as calib:
    while True:
        try:
            calib.capture_image()
            calib.get_cam2grid()
            calib.show_images()
        except RuntimeError:
            print "something went wrong"
