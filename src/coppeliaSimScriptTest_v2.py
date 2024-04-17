#python
import math

def sysCall_init():
    sim = require('sim')

    self.joint1 = sim.getObject('/joint1')
    self.joint2 = sim.getObject('/joint2')
    self.jointAngles = []
    self.i = 0
    self.waitUntil = 0
    
    self.reachedTarget1 = 0
    self.reachedTarget2 = 0
    self.reachedTargetPrev1 = 0
    self.reachedTargetPrev2 = 0

    with open('/home/u000000/2_Semester/bioinspired-control-model-for-human-motor-skills/src/jointAngles.csv', 'r') as file:
        for line in file:
            pos = line.split(',')
            posJoint1 = math.radians(float(pos[0]))
            posJoint2 = math.radians(float(pos[1]))
            self.jointAngles.append((posJoint1, posJoint2))

    print('jointAngles = ' + str(self.jointAngles))

    self.reachedTargetPrev1 = self.jointAngles[0][0]
    self.reachedTarget1 = self.jointAngles[0][0]
    self.reachedTargetPrev2 = self.jointAngles[0][1]
    self.reachedTarget2 = self.jointAngles[0][1]

def sysCall_actuation():

    # moving accordingly to theta values
    pos = self.jointAngles[self.i]
    print('pos = ' + str((pos)))
    sim.setJointTargetPosition(self.joint1, pos[0])
    sim.setJointTargetPosition(self.joint2, pos[1])
    tolerance = 0.25*math.pi/180
    if [abs(sim.getJointPosition(self.joint1)-sim.getJointTargetPosition(self.joint1)) <= tolerance and abs(sim.getJointPosition(self.joint2)-sim.getJointTargetPosition(self.joint2)) <= tolerance]:
        if self.i < len(self.jointAngles):
            self.i += 1
    else:
        return

def sysCall_sensing():
    # put your sensing code here

    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
