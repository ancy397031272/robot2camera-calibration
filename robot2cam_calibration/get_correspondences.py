import robot2cam_calibration.track_grid as ci
import ur_cb2.cb2_robot as cb2_robot
import json
import time
import numpy as np

with open('cb2points.json', 'r') as f:
    data = json.load(f)
    write_time = data['time']
    points = data['points']
    print 'read in {} points, written at: {}'.format(len(points.keys()),
                                                     write_time)


camera2grid = []
tcp2robot = []

with ci.GridLocation("result.json", 7, 8, 25.4, 'flycap') as calib:
    with cb2_robot.URRobot("192.168.1.100", 30003) as robot:
        for number in sorted([int(x) for x in points.keys()]):
            robot.add_goal(cb2_robot.Goal(points[str(number)]['joint'], False,
                                          'joint'))
            # TODO: this appears to skip the first point!
            robot.move_on_stop()
            print 'Beginning move: {}'.format(number)

            while not (robot.at_goal() and robot.is_stopped()):
                time.sleep(.25)
            time.sleep(.25)  # let everything settle

            go_on = 0
            while go_on <= 5:
                try:
                    camera2grid.append(calib.get_cam2grid())
                    calib.show_images()
                    with robot.receiver.lock:
                        tcp2robot.append(robot.receiver.position)
                    print "got the grid"
                    go_on = 6
                except RuntimeError, e:
                    print "something went wrong: {}".format(e)
                    go_on += 1

print np.asarray(tcp2robot) # Axis-Angle [x,y,z,ax,ay,az]
print np.asarray(camera2grid)
