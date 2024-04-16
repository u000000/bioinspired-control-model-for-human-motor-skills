#python
import math

def sysCall_init():
    sim = require('sim')

    self.joint1 = sim.getObject('/joint1')
    self.joint2 = sim.getObject('/joint2')
    self.jointAngles = []
    self.i = 0

    with open('/home/u000000/2_Semester/bioinspired-control-model-for-human-motor-skills/src/jointAngles.csv', 'r') as file:
        for line in file:
            pos = line.split(',')
            posJoint1 = math.radians(float(pos[0]))
            posJoint2 = math.radians(float(pos[1]))
            self.jointAngles.append((posJoint1, posJoint2))


def sysCall_actuation():
    # put your actuation code here
    # theta1 = sim.getJointPosition(self.joint1)
    # theta2 = sim.getJointPosition(self.joint2)
    # print('theta1 = ' + str(theta1) + ' theta2 = ' + str(theta2))

    # moving accordingly to theta values
    pos = self.jointAngles[self.i]
    sim.setJointTargetPosition(self.joint1, pos[0])
    sim.setJointTargetPosition(self.joint2, pos[1])
    self.i += 1

    pass

def sysCall_sensing():
    # put your sensing code here

    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
