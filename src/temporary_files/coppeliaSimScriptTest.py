#python
import math

def sysCall_init():
    sim = require('sim')

    self.joint1 = sim.getObject('/joint1')
    self.joint2 = sim.getObject('/joint2')


def sysCall_actuation():
    # put your actuation code here
    T_last_inserted = 0
    theta1 = sim.getJointPosition(self.joint1)
    theta2 = sim.getJointPosition(self.joint2)
    #print('theta1 = ' + str(theta1) + ' theta2 = ' + str(theta2))

    # moving accordingly to theta values
    with open('/home/u000000/2_Semester/bioinspired-control-model-for-human-motor-skills/src/jointAngles.csv', 'r') as file:
        for line in file:
            theta1_deg = float(line.split(',')[0])
            theta2_deg = float(line.split(',')[1])
            theta1_rad = math.radians(theta1_deg)
            theta2_rad = math.radians(theta2_deg)
            sim.setJointTargetPosition(self.joint1, theta1_rad)
            sim.setJointTargetPosition(self.joint2, theta2_rad)
            
            joint1TargetPosition = sim.getJointTargetPosition(self.joint1)
            joint2TargetPosition = sim.getJointTargetPosition(self.joint2)
            print('joint1 = ' + str(joint1TargetPosition))
            print('joint2 = ' + str(joint2TargetPosition))
            
            # wait till target position is reached
            #if sim.getJointPosition(self.joint1) == theta1_rad and sim.getJointPosition(self.joint2) == theta2_rad:
            #    sim.wait(10.0, False)
            
                
    pass

def sysCall_sensing():
    # put your sensing code here

    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
