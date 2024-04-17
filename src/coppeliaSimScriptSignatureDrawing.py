#python
import math

def sysCall_init():
    sim = require('sim')

    drawOriginalSignature()

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
    self.reachedTargetPrev2 = self.jointAngles[0][0]
    self.reachedTarget2 = self.jointAngles[0][0]

    print('reachedTargetPrev1 = ' + str(self.reachedTargetPrev1))
    print('reachedTarget1 = ' + str(self.reachedTarget1))
    print('reachedTargetPrev2 = ' + str(self.reachedTargetPrev2))
    print('reachedTarget2 = ' + str(self.reachedTarget2))

def sysCall_actuation():
    # put your actuation code here
    # if self.waitUntil > sim.getSimulationTime():
    #     print(self.waitUntil)
    #     return

    tolerance = math.radians(1)

    self.reachedTargetPrev1 = self.reachedTarget1
    self.reachedTarget1 = abs(sim.getJointPosition(self.joint1)-self.jointAngles[self.i][0]) <= tolerance
    # print('reachedTarget1 = ' + str(self.reachedTarget1))
    
    self.reachedTargetPrev2 = self.reachedTarget2
    self.reachedTarget2 = abs(sim.getJointPosition(self.joint2)-self.jointAngles[self.i][0]) <= tolerance
    # print('reachedTarget2 = ' + str(self.reachedTarget2))
    
    if not self.reachedTarget1 and not self.reachedTarget2:
        # target not reached: do nothing
        sim.setJointTargetPosition(self.joint1, self.jointAngles[self.i][0])
        sim.setJointTargetPosition(self.joint2, self.jointAngles[self.i][1])
        pass
    elif not self.reachedTargetPrev1 and not self.reachedTargetPrev2:
        # target just reached, wait 1s
        self.waitUntil = sim.getSimulationTime() + 1
    else:
        # target reached, wait done, select next target
        if self.i < len(self.jointAngles)-1:
            self.i += 1
            sim.setJointTargetPosition(self.joint1, self.jointAngles[self.i][0])
            sim.setJointTargetPosition(self.joint2, self.jointAngles[self.i][1])

def sysCall_sensing():
    # put your sensing code here

    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

def drawOriginalSignature():
    drawingObject = sim.addDrawingObject(sim.drawing_spherepts, 0.005, 0, -1, 1000, [0, 0, 1])

    with open('/home/u000000/2_Semester/bioinspired-control-model-for-human-motor-skills/src/line.csv', 'r') as file:
        for line in file:
            point = line.split(',')
            sim.addDrawingObjectItem(drawingObject, [0,int(point[0])/300,int(point[1])/300])