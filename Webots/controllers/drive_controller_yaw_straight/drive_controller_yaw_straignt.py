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

recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('../../../python_model/webots_data.txt','w')
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
driveVelocity = 6.25
Rrw = 0.15875
driveOmega = driveVelocity/Rrw

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
    if(simtime>3):
        goalRoll = .3 #set an arb number for now, to compare with papadop
    else:
        goalRoll = 0

    #goalRoll = 0.05

    eRoll = goalRoll - roll
    rollInt = rollInt + eRoll*(timestep/1000.0)

    #LQR gains from papadop:
    #Klqr = array([-86.8123537,  133.97971569, -14.86411525,   9.73292061, 100.])
    # Klqr = array([-77.94868821, 120.93186311, -13.20931288,   9.72035992, 100.        ])
    Klqr = array([-9.54924635, 12.44287136, -1.02011267,  1.06339454, 10.        ])
    #below are gains for ballast of 30kg 0.5m up
    #Klqr = array([-82.34489608,  95.15268744, -18.50359612,   9.37877004, 100.        ])
    #implement the LQR: subtract when it's a state, add when it's an error.
    T = Klqr[0]*(eRoll) - Klqr[1]*steerangle - Klqr[2]*rollRate - Klqr[3]*steerRate - Klqr[4]*rollInt
    #print("Control Torque: "+"{:2f}".format(T)+", Speed "+"{:2f}".format(U)+", steer: "+"{:2f}".format(steerangle))
    #print("IntE: "+"{:2f}".format(rollInt))
    #print("Terms: "+"{:2f}".format(Klqr[0]*(eRoll))+", "+"{:2f}".format(-Klqr[1]*steerangle)+", "+"{:2f}".format(-Klqr[2]*rollRate)+", "+"{:2f}".format(-Klqr[3]*steerRate)+", "+"{:2f}".format(-Klqr[4]*rollInt))
    #print("Steer: "+"{:2f}".format(steerangle)+", roll: "+"{:2f}".format(roll))
    #print("Torque: "+"{:2f}".format(T))
    print("rate = "+str(rollRate)+", bad: "+str(rollRate_bad))
    steer.setControlPID(0.0001,0,0)
    steer.setPosition(float('inf'))
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    #steer.setTorque(-T)
    #plot.update(simtime,rollRate)
    #ang = -.2*rpy[0] #+ .50*(rpy[2]-pi)
    #steer.setPosition(ang)
    # time, goalRoll, Torque, speed, roll, rollrate, pitch, pitchrate, intE
    if(recordData):
        f.write(str(simtime)+","+str(goalRoll)+","+str(T)+","+str(U)+","+str(roll)+","+str(steerangle)+","+str(rollRate)+","+str(steerRate)+","+str(rollInt)+"\r\n")

# Enter here exit cleanup code.
