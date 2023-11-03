"""basic_drive_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, InertialUnit
from numpy import *
from Rollover import Rollover
from realtime_plotter import RealTimePlot

#plot = RealTimePlot(x_label='time',y_label='rollrate',marker='k')

# create the Robot instance.
robot = Robot()
yawCorr = Rollover()


# CONTROL PARAMETERS
lastControlTime = 0
dTcontrol = 0.02
gainmtx = loadtxt('razorgains.txt')
lqrspeeds = gainmtx[:,0]
lqrgains = gainmtx[:,1:]
T = 0

# SIMULATION SETUP
driveVelocity = 3.0
stepRollVal = 0.1
stepTime = 1


recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('./webots_data.txt','w')
    f.write("# time, goalRoll, Torque, speed, roll, rollrate, steer, steerrate, intE\r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
motor.setPosition(float('inf'))

imu = robot.getDevice('imu')
imu.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)
steergyro = robot.getDevice('steergyro')
steergyro.enable(timestep)

steersensor = robot.getDevice('steer_angle')
steersensor.enable(timestep)

simtime = 0.0
yawRate = 0
oldYaw = 0

rollInt = 0
inteYawRate = 0
oldRoll = 0

steerangle = 0
oldsteer = 0
steerRate = 0

oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()

firstLoop = True

#set the simulation forward speed and calculate rear wheel omega
Rrw = 0.15875
driveOmega = driveVelocity/Rrw


def clamp(val,min,max):
    assert min<max
    if(val<min):
        val=min
    elif val>max:
        val=max
    return val

def setDriveMotorTorque(self,motor,command,omega):
    kt = 0.86/13.5 #published for MY1016-C1 at 24V max torque .86 Nm, max curr 13.5A
    Vbatt = 24.0 #total voltage
    """! sets motor torque in simulation based on a physic-based model (not for user use)"""
    Vcommand = clamp(command,-Vbatt,Vbatt)
    #this uses a physics-based model to calculate torque based on motor params.
    torque = self.kt/(self.R)*(Vcommand-self.kt*omega)
    #set motor force
    motor.setTorque(torque)

def find_nearest_index(array, value):
    array = asarray(array)
    idx = (abs(array - value)).argmin()
    return idx

def getCurrentGains(speed):
    #find the speed in the gain array closest to ours
    idx = find_nearest_index(lqrspeeds,speed)
    #find the gain set at this index
    Klqr = lqrgains[idx,:]
    return Klqr

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    simtime+=timestep/1000.0
    if(firstLoop):
        oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
        oldYaw = yawCorr.update(oldYaw)
        oldsteer = steersensor.getValue()
        firstLoop=False
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    #get current fwd speed
    U = gps.getSpeed()
    #get IMU values and process to get yaw, roll rates
    #read IMU value
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()
    #rollInt += (timestep/1000.0)*rpy[0]
    yaw = rpy[2]
    yaw = yawCorr.update(yaw)
    yawRate = gyros[2]#(yaw-oldYaw)/(timestep/1000.0)
    # print("yaw/old: "+str(yaw)+","+str(oldYaw))
    oldYaw = yaw
    roll = rpy[0]
    rollRate = gyros[0]#(roll-oldRoll)/(timestep/1000.0)
    rollRate_bad = (roll-oldRoll)/(timestep/1000.0)
    oldRoll = roll

    #now get steer values and calculate steer rate.
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    steerangle = -steersensor.getValue()
    steergyros = steergyro.getValues()
    steerRate = -steergyros[2]#(steerangle-oldsteer)/(timestep/1000.0)
    steerRate_bad = (steerangle-oldsteer)/(timestep/1000.0)
    oldsteer = steerangle


    # Enter here functions to send actuator commands, like:
    motor.setVelocity(driveOmega)

    #print("goalYawRate/yawRate: "+str(goalYawRate)+","+str(yawRate))

    # eYawRate = (goalYawRate - yawRate);
    # inteYawRate +=(timestep/1000.0)*eYawRate;
    # #set roll angle based on yaw rate.
    # #ay = U * yawRate

    #print("speed, accel (g): "+str(U)+","+str(U*yawRate/9.81))
    # goalRoll = -arctan(U*goalYawRate/9.81)#eYawRate*-.1 - 1*inteYawRate
    #if(simtime>stepTime):
        #goalRoll = stepRollVal #set an arb number for now, to compare with papadop
    #else:
        #goalRoll = 0

    #goalRoll = 0.05
    goalRoll = 0

    if((simtime-lastControlTime)>dTcontrol):

        eRoll = goalRoll - roll
        rollInt = rollInt + eRoll*(timestep/1000.0)

        #LQR gains from papadop:
        Klqr = getCurrentGains(driveVelocity)
        #Klqr = array([-9.54924635, 12.44287136, -1.02011267,  1.06339454, 10.        ])
        #below are gains for ballast of 30kg 0.5m up

        T = Klqr[0]*(eRoll) - Klqr[1]*steerangle - Klqr[2]*rollRate - Klqr[3]*steerRate - 0*Klqr[4]*rollInt
        #print("rate = "+str(rollRate)+", bad: "+str(rollRate_bad))
        T = -T
        Tlim = 1.5916
        if(T>Tlim):
            T = Tlim
        elif(T<-Tlim):
            T = -Tlim
        
        print("Torque: "+str(T))
        steer.setControlPID(0.0001,0,0)
        steer.setPosition(float('inf'))
        # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
        steer.setTorque(T)
        lastControlTime = simtime
    if(recordData):
        f.write(str(simtime)+","+str(goalRoll)+","+str(T)+","+str(U)+","+str(roll)+","+str(steerangle)+","+str(rollRate)+","+str(steerRate)+","+str(rollInt)+"\r\n")

# Enter here exit cleanup code.
