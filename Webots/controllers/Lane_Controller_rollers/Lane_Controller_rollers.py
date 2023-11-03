"""basic_drive_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, InertialUnit, Display
from numpy import *
from Rollover import Rollover
from realtime_plotter import RealTimePlot
from LaneEstimator import LaneEstimator

#plot = RealTimePlot(x_label='time',y_label='rollrate',marker='k')

# create the Robot instance.
robot = Robot()
yawCorr = Rollover()

tqplot = RealTimePlot(x_label="Time (s)",y_label="Steer Torque (Nm)",ylim=2,delaycounts=10)
rollplot = RealTimePlot(x_label="Time (s)",y_label="Roll (rad)",ylim = 0.3,delaycounts = 10)
laneplot = RealTimePlot(x_label="Time (s)",y_label="lane position (m)",ylim = 0.3,delaycounts = 10)
# CONTROL PARAMETERS
lastControlTime = 0
dTcontrol = 0.02
gainmtx = loadtxt('razorgains_lane.txt')
lqrspeeds = gainmtx[:,0]
lqrgains = gainmtx[:,1:]
T = 0
fricComp = 0.2
eLane = 0
eLaneInt = 0
useEstimator = True
useIntegral = 1.0


#introduce lane estimator
laneest = LaneEstimator()

# SIMULATION SETUP
driveVelocity = 3.0
stepLaneVal = 0
stepTime = 1
impulseDone = False
impulseTorque = 10
impulseTime = 3


recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('./webots_data.txt','w')
    f.write("# time, goalRoll, Torque, speed, roll, rollrate, steer, steerrate, intERoll, yaw, y, intELane\r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
#tqdisplay = robot.getDevice('torque_display')
#rolldisplay = robot.getDevice('roll_display')
#lanedisplay = robot.getDevice('lane_display')

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
    xyz = gps.getValues()
    curr_y = -xyz[1]
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

    ### NOW UPDATE LANE ESTIMATOR
    yarwrate_est,yaw_est,yvel_est,y_est = laneest.update(U,steerangle,timestep/1000.0)

    # Enter here functions to send actuator commands, like:
    motor.setVelocity(driveOmega)

    #allow for step in desired lane pos
    if(simtime>stepTime):
        goalLane = stepLaneVal #set an arb number for now, to compare with papadop
    else:
        goalLane = 0
    #always set goal roll to 0
    goalRoll = 0



    if((simtime-lastControlTime)>dTcontrol):

        eRoll = goalRoll - roll
        rollInt = rollInt + eRoll*(dTcontrol)

        if(useEstimator):
            eLane = goalLane - y_est #estimate of integral of lane error
        else:
            eLane = goalLane-curr_y
        eLaneInt += eLane*(dTcontrol)

        #LQR gains from papadop:
        Klqr = getCurrentGains(driveVelocity)

        if(useEstimator):
            T = Klqr[0]*(eRoll) - Klqr[1]*steerangle - Klqr[2]*rollRate - Klqr[3]*steerRate - useIntegral*Klqr[4]*rollInt - Klqr[5]*yaw_est - Klqr[6]*y_est - useIntegral*Klqr[7]*eLaneInt
        else:
            T = Klqr[0]*(eRoll) - Klqr[1]*steerangle - Klqr[2]*rollRate - Klqr[3]*steerRate - useIntegral*Klqr[4]*rollInt - Klqr[5]*yaw - Klqr[6]*curr_y - useIntegral*Klqr[7]*eLaneInt

        #print("rate = "+str(rollRate)+", bad: "+str(rollRate_bad))
        T = -T

        if(T<0):
            T-=fricComp
        elif(T>0):
            T+=fricComp

        Tlim = 1.5916
        if(T>Tlim):
            T = Tlim
        elif(T<-Tlim):
            T = -Tlim



        #udpate roll plot
        #rollplot.update(simtime,roll)
        #update ttorque plot
        #tqplot.update(simtime,T)
        #update lane plot
        #laneplot.update(simtime,yaw_est)

        #if(rollplot.newPlot):
            #height,width,depth = rollplot.imgsize
            #ir = rolldisplay.imageNew(rollplot.img_array.tolist(), Display.RGBA,width,height)
            #rolldisplay.imagePaste(ir, 0, 0, False)
            #rolldisplay.imageDelete(ir)
        #print(width,height,depth)
        #if(tqplot.newPlot):
            #height,width,depth = tqplot.imgsize
            #itq = tqdisplay.imageNew(tqplot.img_array.tolist(), Display.RGBA,width,height)
            #tqdisplay.imagePaste(itq, 0, 0, False)
            #tqdisplay.imageDelete(itq)
        #print(width,height,depth)
        #if(laneplot.newPlot):
            #height,width,depth = laneplot.imgsize
            #ilane = lanedisplay.imageNew(laneplot.img_array.tolist(), Display.RGBA,width,height)
            #lanedisplay.imagePaste(ilane, 0, 0, False)
            #lanedisplay.imageDelete(ilane)


        ## IF we are at impulseTime, add impulse
        if(simtime>impulseTime and not impulseDone):
            T +=impulseTorque
            impulseDone = True



        print("Time: "+str(simtime)+", Torque: "+str(T)+", y: "+str(curr_y))
        steer.setControlPID(0.0001,0,0)
        steer.setPosition(float('inf'))
        # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
        steer.setTorque(T)
        lastControlTime = simtime
    if(recordData):
        f.write(str(simtime)+","+str(goalLane)+","+str(T)+","+str(U)+","+str(roll)+","+str(steerangle)+","+str(rollRate)+","+str(steerRate)+","+str(rollInt)+","+str(yaw_est)+","+str(y_est)+","+str(eLaneInt)+"\r\n")

# Enter here exit cleanup code.
