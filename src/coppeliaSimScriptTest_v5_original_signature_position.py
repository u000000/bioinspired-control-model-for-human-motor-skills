#python
import math

def sysCall_init():
    sim = require('sim')

    self.joint1 = sim.getObject('/joint1')
    self.joint2 = sim.getObject('/joint2')
    self.tip = sim.getObject('/tip')

    self.jointAngles = []
    self.i = 0
    self.j = 0
    self.waitUntil = 0

    self.reachedTarget1 = 0
    self.reachedTarget2 = 0

    # variables to display an orignal signature
    self.coppeliaFirstPoint = 0
    self.coppeliaLastPoint = 0

    with open('/home/u000000/Desktop/github_2_Semester/bioinspired-control-model-for-human-motor-skills/src/jointAngles.csv', 'r') as file:
        for line in file:
            pos = line.split(',')
            posJoint1 = float(pos[1])
            posJoint2 = float(pos[2])
            self.jointAngles.append((posJoint1, posJoint2))
        file.close()
    # print('jointAngles = ' + str(self.jointAngles))

    self.reachedTarget1 = self.jointAngles[0][0]
    self.reachedTarget2 = self.jointAngles[0][0]

    # print('reachedTarget1 = ' + str(self.reachedTarget1))
    # print('reachedTarget2 = ' + str(self.reachedTarget2))

def sysCall_actuation():
    tolerance = math.radians(1)

    self.reachedTarget1 = abs(sim.getJointPosition(self.joint1)-self.jointAngles[self.i][0]) <= tolerance
    # print('reachedTarget1 = ' + str(self.reachedTarget1))

    self.reachedTarget2 = abs(sim.getJointPosition(self.joint2)-self.jointAngles[self.i][0]) <= tolerance
    # print('reachedTarget2 = ' + str(self.reachedTarget2))

    if not self.reachedTarget1 and not self.reachedTarget2:
        # target not reached: do nothing
        sim.setJointTargetPosition(self.joint1, self.jointAngles[self.i][0])
        sim.setJointTargetPosition(self.joint2, self.jointAngles[self.i][1])
        pass
    else:
        # target reached, select next target
        if self.i < len(self.jointAngles)-1:
            if self.i == 0:
                self.coppeliaFirstPoint = sim.getObjectPosition(self.tip)

            self.i += 1
            sim.setJointTargetPosition(self.joint1, self.jointAngles[self.i][0])
            sim.setJointTargetPosition(self.joint2, self.jointAngles[self.i][1])
        elif self.i == len(self.jointAngles)-1 and self.j == 0:
            self.coppeliaLastPoint = sim.getObjectPosition(self.tip)
            drawOriginalSignature()
            self.j = 1

def sysCall_sensing():
    # put your sensing code here

    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

def calculateTransformation(coppelia_first, coppelia_last, drawing_first, drawing_last):
    x1_c, y1_c = coppelia_first
    x2_c, y2_c = coppelia_last
    x1_d, y1_d = drawing_first
    x2_d, y2_d = drawing_last

    tx = x1_c - x1_d
    ty = y1_c - y1_d

    sx = (x2_c - x1_c) / (x2_d - x1_d)
    sy = (y2_c - y1_c) / (y2_d - y1_d)

    return tx, ty, sx, sy, x1_c, y1_c, x1_d, y1_d

def transform(x, y, transformation):
    tx, ty, sx, sy, x1_c, y1_c, x1_d, y1_d = transformation

    x_new = sx * (x - x1_d) + tx
    y_new = sy * (y - y1_d) + ty

    return x_new, y_new


def drawOriginalSignature():
    drawingObject = sim.addDrawingObject(sim.drawing_spherepts, 0.005, 0, -1, 1000, [0, 0, 1])

    with open('/home/u000000/Desktop/github_2_Semester/bioinspired-control-model-for-human-motor-skills/src/line.csv', 'r') as file:
        drawingFirstPoint = file.readline().strip().split(',')
        drawingLastPoint = ''
        for line in file:
            drawingLastPoint = line.strip().split(',')
        file.close()

    drawingFirstPoint = (float(drawingFirstPoint[0]), float(drawingFirstPoint[1]))
    drawingLastPoint = (float(drawingLastPoint[0]), float(drawingLastPoint[1]))

    print('drawingFirstPoint = ' + str(drawingFirstPoint))
    print('drawingLastPoint = ' + str(drawingLastPoint))

    self.coppeliaFirstPoint = self.coppeliaFirstPoint[:2]
    self.coppeliaLastPoint = self.coppeliaLastPoint[:2]
    
    print('coppeliaFirstPoint = ' + str(self.coppeliaFirstPoint))
    print('coppeliaLastPoint = ' + str(self.coppeliaLastPoint))
    
    transformation_data = calculateTransformation(self.coppeliaFirstPoint, self.coppeliaLastPoint, drawingFirstPoint, drawingLastPoint)

    with open('/home/u000000/Desktop/github_2_Semester/bioinspired-control-model-for-human-motor-skills/src/line.csv', 'r') as file:
        for line in file:
            point = line.strip().split(',')
            point_draw = transform(int(point[0]), int(point[1]), transformation_data)
            print('point_draw = ' + str(point_draw))
            sim.addDrawingObjectItem(drawingObject, [0,-point_draw[0],point_draw[1]])
        file.close()