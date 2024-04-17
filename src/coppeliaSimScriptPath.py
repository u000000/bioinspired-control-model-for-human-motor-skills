#python

def sysCall_init():
    sim = require('sim')
    ctrlPts = []

    with open('/home/u000000/2_Semester/bioinspired-control-model-for-human-motor-skills/src/line.csv', 'r') as file:
        for line in file:
            point = line.split(',')
            row = [float(point[0]), float(point[1]), float(point[2]), float(point[3]), float(point[4]), float(point[5]), float(point[6])]
            ctrlPts.append(row)

def sysCall_actuation():
    # put your actuation code here
    pass

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
