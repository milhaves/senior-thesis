from controller import Robot, Motor, PositionSensor, InertialUnit
from numpy import *
# from Rollover import Rollover

# create the Robot instance.
robot = Robot()
# yawCorr = Rollover()

# CONTROL PARAMETERS
lastControlTime = 0
dTcontrol = 0.0005
# gainmtx = loadtxt('gains.txt')
# lqrspeeds = gainmtx[:,0]
# lqrgains = gainmtx[:,1]
T = 0

# SIMULATION SETUP
stepRollVal = 0.1
stepTime = 1

recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('./pendulum_data.txt','w')
    f.write("# time, goalRoll, leanTorque, roll, rollrate, lean, leanrate, intE\r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#motor = robot.getDevice('drive_motor')
pendulum = robot.getDevice('balance_motor')
#motor.setPosition(float('inf'))

imu = robot.getDevice('imu')
imu.enable(timestep)

pendulumsensor = robot.getDevice('pendulum_position')
pendulumsensor.enable(timestep)

simtime = 0.0
# yawRate = 0
# oldYaw = 0

rollInt = 0
# inteYawRate = 0
oldRoll = 0

# steerangle = 0
# oldsteer = 0
# steerRate = 0

oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()

firstLoop = True

def clamp(val,min,max):
    assert min<max
    if(val<min):
        val=min
    elif val>max:
        val=max
    return val

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

while robot.step(timestep) != -1:
    simtime+=timestep/1000.0
    if(firstLoop):
        oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
        # oldYaw = yawCorr.update(oldYaw)
        oldlean = pendulumsensor.getValue()
        firstLoop=False
    #get IMU values and process to get yaw, roll rates
    #read IMU value
    rpy = imu.getRollPitchYaw()
    #rollInt += (timestep/1000.0)*rpy[0]
    # yaw = rpy[2]
    # yaw = yawCorr.update(yaw)
    # yawRate = (yaw-oldYaw)/(timestep/1000.0)
    # print("yaw/old: "+str(yaw)+","+str(oldYaw))
    # oldYaw = yaw
    roll = -rpy[0]
    rollRate = (roll-oldRoll)/(timestep/1000.0)
    rollRate_bad = (roll-oldRoll)/(timestep/1000.0)
    oldRoll = roll

    leanangle = -pendulumsensor.getValue()
    leanrate = (leanangle-oldlean)/(timestep/1000.0)
    oldlean = leanangle

    goalRoll = 0

    if((simtime-lastControlTime)>dTcontrol):

        eRoll = goalRoll - roll
        rollInt = rollInt + eRoll*(timestep/1000.0)

        print("eRoll: "+str(eRoll))
        print("leanangle: "+str(leanangle))
        print("rollRate: "+str(rollRate))
        print("leanRate: "+str(leanrate))
        print("rollInt: "+str(rollInt))

        K = array([-662.734025559258,-233.28228440665075,-277.78471581784527,-107.52438353053273,-31.62277660165455])
        T = K[0]*eRoll - K[1]*leanangle - K[2]*rollRate - K[3]*leanrate - K[4]*rollInt
        #print("rate = "+str(rollRate)+", bad: "+str(rollRate_bad))
        T = -T
        # Tlim = 1.5916
        # if(T>Tlim):
        #     T = Tlim
        # elif(T<-Tlim):
        #     T = -Tlim

        print("Torque: "+str(T))
        print("-------------------------------")
        pendulum.setControlPID(0.0001,0,0)
        pendulum.setPosition(float('inf'))
        # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
        pendulum.setTorque(T)
        lastControlTime = simtime
    if(recordData):
        f.write(str(simtime)+","+str(goalRoll)+","+str(T)+","+str(roll)+","+str(leanangle)+","+str(rollRate)+","+str(leanrate)+","+str(rollInt)+"\r\n")

# Enter here exit cleanup code.
