#python
import math

def sysCall_init():
    sim = require('sim')

    self.joint1 = sim.getObject('/joint1')
    self.joint2 = sim.getObject('/joint2')


def sysCall_actuation():
    # put your actuation code here

    theta1 = sim.getJointPosition(self.joint1)
    theta2 = sim.getJointPosition(self.joint2)
    print('theta1 = ' + str(theta1) + ' theta2 = ' + str(theta2))

    # moving accordingly to theta values
    with open('/home/u000000/2_Semester/bioinspired-control-model-for-human-motor-skills/src/jointAngles.csv', 'r') as file:
        for line in file:
            theta1_deg = float(line.split(',')[0])
            theta2_deg = float(line.split(',')[1])
            theta1_rad = math.radians(theta1_deg)
            theta2_rad = math.radians(theta2_deg)
            sim.setJointTargetPosition(self.joint1, theta1_rad)
            sim.setJointTargetPosition(self.joint2, theta2_rad)

            #wait till target position is reached
            while True:
                if sim.getJointPosition(self.joint1) == theta1_rad and sim.getJointPosition(self.joint2) == theta2_rad:
                    break
                sim.switchThread()

    pass

def sysCall_sensing():
    # put your sensing code here

    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
