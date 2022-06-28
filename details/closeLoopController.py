import Sofa
from splib3.numerics import Quat
from splib3.constants import Key

import numpy as np
import math
import serial
import time
import csv

class EffectorController(Sofa.Core.Controller):
    """The goal of this controller is to :
       - control the orientation of the goal 
    """

    def __init__(self, *args, serialport=None, servomotors=None, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "InverseController"
        self.targetGoal = args[1]
                
    def onKeypressedEvent(self, event):
        key = event['key']

        positionRigid = np.array(self.targetGoal.goalMO.position.value)
        position = positionRigid[0][0:3]
        quat = Quat(positionRigid[0][3:7])
        angles = quat.getEulerAngles( axes='sxyz')

        # rotate around x
        if key == Key.uparrow:
            self.index = 0

        # rotate around z
        if key == Key.rightarrow:
            self.index = 2

        # increase or decrease angle of 0.1 rad (5.7deg)
        if key == Key.plus :
            angles[self.index] += 0.1
        if key == Key.minus :
            angles[self.index] -= 0.1 
        
        new_quat = Quat.createFromEuler(angles)
        self.targetGoal.goalMO.position.value = [list(position) +[new_quat.take(0),new_quat.take(1),new_quat.take(2),new_quat.take(3)]]

class OpenLoopController(Sofa.Core.Controller):
    """The goal of this controller is to :
       - control the orientation of the goal 
    """

    def __init__(self, *args, serialport=None, servomotors=None, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "InverseController"
        self.targetGoal = args[1]
        self.arduino = args[2]

        self.time = time.time()
        self.t = [0]
        self.step = 0
        self.file = open('openLoop.csv', 'w')
        writer = csv.writer(self.file)
        writer.writerow(['time','x_input','x_tracking','z_input','z_tracking'])
        self.input = []
        with open("closeLoop.csv", 'r') as file:
            csvreader = csv.reader(file)
            header = next(csvreader)
            for row in csvreader:
                self.input.append([float(row[1]),float(row[4])])

    def onKeypressedEvent(self, event):
        key = event['key']

        positionRigid = np.array(self.targetGoal.goalMO.position.value)
        position = positionRigid[0][0:3]
        quat = Quat(positionRigid[0][3:7])
        angles = quat.getEulerAngles( axes='sxyz')

        new_quat = Quat.createFromEuler(angles)
        self.targetGoal.goalMO.position.value = [list(position) +[new_quat.take(0),new_quat.take(1),new_quat.take(2),new_quat.take(3)]]
    
    def onAnimateBeginEvent(self, e):
        # get time
        t2 = time.time()
        self.dt = t2-self.time
        self.time = t2
        self.t.append(self.t[-1]+self.dt)
        
        if self.arduino.state == "init":
            return

        if self.arduino.state == "no-comm":
            return

        # put same orientation as closeLoop
        if(self.arduino.activate):
            if self.step < len(self.input):
                print(self.step)
                positionRigid = np.array(self.targetGoal.goalMO.position.value)
                position = positionRigid[0][0:3]
                quat = Quat(positionRigid[0][3:7])
                angles = quat.getEulerAngles( axes='sxyz')

                new_quat = Quat.createFromEuler([self.input[self.step][0],angles[1],self.input[self.step][1]])
                self.targetGoal.goalMO.position.value = [list(position) +[new_quat.take(0),new_quat.take(1),new_quat.take(2),new_quat.take(3)]]
            
                # get sensor value
                self.tracking = self.arduino.sensor

                row = [self.t[-1],self.input[self.step][0],self.tracking[0],self.input[self.step][1],self.tracking[1]]
                # create the csv writer
                writer = csv.writer(self.file)
                writer.writerow(row)

                self.step +=1
        else :
            print('Simulation done')


class CloseLoopController(Sofa.Core.Controller):
    """The goal of this controller it to :
        - add a gain
        - add an integrator
        - add an antiwindup
    """

    def __init__(self, *args,dt, serialport=None, servomotors=None, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "InverseController"
        self.nodeGoal = args[1]
        self.targetGoal = args[2]
        self.arduino = args[3]

        # parameters for the Proportionnal controller
        self.time = time.time()
        # self.dt = dt
        self.ki = 2
        self.kp = 0.2

        self.t = [0]
        self.x_output =[0]
        self.z_output = [0]

        # anti windup parameter:
        self.windupmax = 0.5
        self.kb = 0.98

        self.input = [0,0]
        self.output = [0,0]
        self.tracking = [0,0]

        self.pi_term = [0,0]

        self.file = open('closeLoop.csv', 'w')
        writer = csv.writer(self.file)
        writer.writerow(['time','x_input','x_output','x_tracking','z_input','z_output','z_tracking'])
    ########################################
    # Proportionnal controller functions
    ########################################

    def proportionnal(self):
        epsilon = [self.input[i]-self.tracking[i] for i in range(2)]

        self.proportionnal_term = [epsilon[i]*self.kp for i in range(2)]

    def antiwindup(self):
        # saturation
        windup_error = [0,0]
        for i in range(2):
            if self.output[i] >= self.windupmax:
                pi_sat = self.windupmax
            elif self.output[i] <= -self.windupmax:
                pi_sat = -self.windupmax
            else :
                pi_sat = self.output[i]
        
            # anti windup error 
            windup_error[i] = self.kb*(pi_sat-self.output[i])
        
        return windup_error

    def integrator(self):
        epsilon = [self.input[i]-self.tracking[i] for i in range(2)]
        # print("error : ",epsilon)

        windup_error = self.antiwindup()
        self.pi_term = [epsilon[i]*self.ki+windup_error[i] for i in range(2)]
        self.integrator_tem = [self.output[i]+self.pi_term[i]*self.dt for i in range(2)]
        self.output = [self.proportionnal_term[i] + self.integrator_tem[i] for i in range(2)]
        # print("output =",self.output)

    ########################################
    # other functions
    ########################################
    
    def onAnimateBeginEvent(self, e):
        # get real time step
        t2 = time.time()
        self.dt = t2-self.time
        self.time = t2
        self.t.append(self.t[-1]+self.dt)

        print(f'dt = {self.dt}')

        # get new input
        q = Quat(self.targetGoal.goalMO.position.value[0][3:7])
        angles_target = q.getEulerAngles(axes = 'sxyz')
        self.input = [angles_target[0],angles_target[2]]

        # get sensor value
        self.tracking = self.arduino.sensor

        # get new ouput value
        self.proportionnal()
        self.integrator()
        
        # write output in node goal
        positionRigid = np.array(self.nodeGoal.goalMO.position.value)
        position = positionRigid[0][0:3]
        quat = Quat(positionRigid[0][3:7])
        angles = quat.getEulerAngles( axes='sxyz')
        new_quat = Quat.createFromEuler([self.output[0],angles[1],self.output[1]])
        self.nodeGoal.goalMO.position.value = [list(position) +[new_quat.take(0),new_quat.take(1),new_quat.take(2),new_quat.take(3)]]

        self.x_output.append(self.output[0])
        self.z_output.append(self.output[1])

        row = [self.t[-1],self.input[0],self.output[0],self.tracking[0],self.input[1],self.output[1],self.tracking[1]]

        # create the csv writer
        writer = csv.writer(self.file)

        # write a row to the csv file
        writer.writerow(row)


class InverseController(Sofa.Core.Controller):
    """This controller has two role:
       - if user press I inverse kinematics is started
       - if state is in comm, send and receive data from arduino
       - state is change by DirectController
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.serialObj = serial.Serial("/dev/ttyACM0", 115200,timeout=0.05)
        self.nodeTripod = args[1]
        self.nodesInverseComponents = args[2]
        self.state = "init"
        self.sensor = [0,0]
        self.oldMotorValues = [0,0,0]

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.I:
            for i in range(3):
                self.nodeTripod.actuatedarms[i].ServoMotor.Articulation.RestShapeSpringsForceField.stiffness.value = [0.]
            self.activate = True
            for node in self.nodesInverseComponents:
                node.activated = bool(self.activate)
                node.init()

    def onAnimateBeginEvent(self,e):

        # read serial port
        currentLine = self.serialObj.readline()   
        try:
            DecodedAndSplit = currentLine.decode().split(',')
            FloatValues = []
            for String in DecodedAndSplit[:4]:
                FloatValues.append(float(String))
            anglesIMU = Quat(FloatValues).getEulerAngles()
            if anglesIMU[0]<0:
                anglesIMU[0]= anglesIMU[0]+math.pi  
            else :
                anglesIMU[0] = anglesIMU[0]-math.pi  
            self.sensor = [anglesIMU[1],anglesIMU[0]]
        except:
            print("Error while decoding/writing IMU data")

        if self.state == "init":
            return

        if self.state == "no-comm":
            return

        # write convert angles in byte
        if(self.activate):

            Angles = [0]*3;
            for i in range(3):
                Angles[i] = self.nodeTripod.actuatedarms[i].ServoMotor.Articulation.dofs.position[0][0];
       
            AnglesOut = []
            
            for i in range(3):
                # Conversion of the angle values from radians to degrees
                angleDegree = Angles[i]*360/(2.0*math.pi)
                angleByte = int(math.floor(angleDegree)) + 179

                # Limitation of the angular position's command
                if angleByte < 60:
                    angleByte = 60
                if angleByte > 180:
                    angleByte = 180

                # Filling the list of the 3 angle values
                AnglesOut.append(angleByte)
        
        # write to serial port
        String = str(AnglesOut[0]) + ' ' + str(AnglesOut[1]) + ' ' + str(AnglesOut[2]) + '\n'
        ByteString = String.encode('ASCII')
        print("Sending to the motors: {}".format(ByteString))
        self.serialObj.write(ByteString)
