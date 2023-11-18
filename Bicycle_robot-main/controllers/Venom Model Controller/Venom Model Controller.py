# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor, InertialUnit
from numpy import *
#from Rollover import Rollover
#from realtime_plotter import RealTimePlot

#plot = RealTimePlot(x_label='time',y_label='rollrate',marker='k')

# create the Robot instance.
robot = Robot()
#yawCorr = Rollover()

lastControlTime = 0
dTcontrol = 0.0005
T = 0

recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('Venom_model_data.txt','w')
    f.write("# time, goalRoll, leanTorque, roll, rollrate, lean, leanrate, intE\r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
# motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
# motor.setPosition(float('inf'))
pendulum = robot.getDevice('balance_motor')

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
pendulumsensor = robot.getDevice('pendulum_position')
pendulumsensor.enable(timestep)


simtime = 0.0
# yawRate = 0
# oldYaw = 0

rollInt = 0
# inteYawRate = 0
oldRoll = 0

# eLane = 0
# eLaneInt = 0

# steerangle = 0
# oldsteer = 0
# steerRate = 0

oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()

firstLoop = True

#set the simulation forward speed and calculate rear wheel omega
# driveVelocity= 3.75#3.95#28.95
# Rrw = 0.3
# driveOmega = driveVelocity/Rrw

# d_roll=0
# k_roll=13;
# k_rollRate=0;
# error = 0
# dedt = 0
# error_old = 0
#
# #k for roll control
# k=8;

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    simtime+=timestep/1000.0
    if(firstLoop):
        oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
        # oldYaw = yawCorr.update(oldYaw)
        # oldsteer = steersensor.getValue()
        oldlean = pendulumsensor.getValue()
        firstLoop=False
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #val = ds.getValue()
    #get current fwd speed
    # U = gps.getSpeed()
    # xyz = gps.getValues()
    # curr_y = -xyz[1]
    #get IMU values and process to get yaw, roll rates
    #read IMU value
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()
    # rollInt += (timestep/1000.0)*rpy[0]
    # yaw = -rpy[2]
    # yaw = yawCorr.update(yaw)
    # yawRate = gyros[2]
    # (yaw-oldYaw)/(timestep/1000.0)
    #print("yaw/old: "+str(yaw)+","+str(oldYaw))

    # oldYaw = yaw
    roll = rpy[0]
    rollRate = (roll-oldRoll)/(timestep/1000.0)
    oldRoll = roll

    #now get steer values and calculate steer rate.
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    # steerangle = -steersensor.getValue()
    # steergyros = steergyro.getValues()
    # steerRate = -steergyros[2]#(steerangle-oldsteer)/(timestep/1000.0)
    # oldsteer = steerangle

    leanangle = -pendulumsensor.getValue()
    leanrate = (leanangle-oldlean)/(timestep/1000.0)
    oldlean = leanangle

    # Enter here functions to send actuator commands, like:
    #motor.setAvailableTorque(10)
    # motor.setVelocity(driveOmega)

   #roll control
    # if(simtime>3):
    #    goalRoll = 0.1
    # else:
    #    goalRoll = 0


    #goalRoll=0;
    # steer.setControlPID(100,0,0)
    #
    # delta = k*(goalRoll - roll)
    # steer.setPosition(delta)



 #coding for T control
   # if(simtime>3):
       #T = 0.008
    #else:
       #T = 0
   # error = d_roll-roll
    #dedt = (error-error_old)/(timestep/1000.0)
    #error_old = error
    #T=k_roll*error+k_rollRate*dedt;
    #steer.setTorque(T)



    # print(simtime,roll)



    #steer.setPosition()
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    #plot.update(simtime,rollRate)
    #ang = -.2*rpy[0] #+ .50*(rpy[2]-pi)
    #steer.setPosition(ang)
    # time, goalRoll, Torque, speed, roll, rollrate, pitch, pitchrate, intE
    goalRoll = 0

    if((simtime-lastControlTime)>dTcontrol):

        eRoll = goalRoll - roll
        rollInt = rollInt + eRoll*(timestep/1000.0)

        print("eRoll: "+str(eRoll))
        print("leanangle: "+str(leanangle))
        print("rollRate: "+str(rollRate))
        print("leanRate: "+str(leanrate))
        print("rollInt: "+str(rollInt))

        K = array([-9025.138601568087,-1439.5933074607906,-2929.7186992458846,-510.2754217196313,-31.622776595861943])
        T = K[0]*eRoll - K[1]*leanangle - K[2]*rollRate - K[3]*leanrate - K[4]*rollInt
        #print("rate = "+str(rollRate)+", bad: "+str(rollRate_bad))
        # T = -T
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
        steer.setPosition(0)
        lastControlTime = simtime

    if(recordData):
        f.write(str(simtime)+","+str(goalRoll)+","+str(T)+","+str(roll)+","+str(leanangle)+","+str(rollRate)+","+str(leanrate)+","+str(rollInt)+"\r\n")

# Enter here exit cleanup code.
