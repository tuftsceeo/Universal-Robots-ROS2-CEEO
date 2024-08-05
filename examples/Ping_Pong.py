from myur import MyUR3e

from sklearn.multioutput import MultiOutputRegressor

robot = MyUR3e()

calibration_points = [  [-0.15,0.1,0.1,0,0,0],
                        [0.15,0.1,0.1,0,0,0],
                        [0.15,0.3,0.1,0,0,0],
                        [-0.15,0.3,0.1,0,0,0],
                        [-0.15,0.1,0.4,0,0,0],
                        [0.15,0.1,0.4,0,0,0],
                        [0.15,0.3,0.4,0,0,0],
                        [-0.15,0.3,0.4,0,0,0],]

calibration_data = []

def track_raw():
    # get live tracking data
    # x & y location, contour size
    return [x,y,c]

def calibrate():
    X = []
    Y = []
    for point in calibration_points:
        robot.move_global(point,time=3)
        
        X.append(track_raw)
        Y.append(point[0:3])

    model = MultiOutputRegressor.fit(X,Y)
    return model

def track(model):
    return model.predict(track_raw())

def roll(target):
    # define threshold value
    # define p,i,d values, should be same for x and y
    while track() not in target +- threshold:
        # calculate angle for x and angle for y
        angle = [0,0,0,x,y,0]
        robot.move_global(angle,wait=False)


def bounce():
    # hard coded start sequence to get ball in the air

    while ascending:
        # track vertical velocity to estimate flight time
        # track horizontal velocity to estimate landing position
        # fit tracked values to a parabolic curve?
        if confidence > threshold or descending: ascending = false

    robot.move_global(landing_pos,wait=False)

    when descending:
        
'''
# PARALLEL CODING VERSION

Cam Node:
    Job: Retreives cam data and uses model to find position
    PUB: position

Tracking Node:
    Job: reads position data to find flight characteristics
    SUB: position
    PUB: time of flight, landing position, confidence

Robot Node:
    Job: moves arm to landing position and strikes
    SUB: flight data
    CLIENT: joint trajectory controller
'''
    
