from controller import Supervisor
from controller import Node

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
mc_node = robot.getFromDef('MOTORCYCLE')
fw_node = robot.getFromDef('front_wheel_solid')
rw_node = robot.getFromDef('rear_wheel')
com=mc_node.getCenterOfMass()
psn=mc_node.getPosition()
com_ego=[com[i]-psn[i] for i in range(3)]
#print(com)
#print(com_ego)
i = 0


initVel = 6.25
#print(mc_node.getCenterOfMass())

while robot.step(TIME_STEP) != -1:
  if (i < 10):
    mc_node.setVelocity([initVel,0,0,0,0,0])
    #fw_node.setVelocity([0,0,0,0,rw_node.getVelocity()[4],0])
    #if(i==1):
        #print(mc_node.getCenterOfMass())
  i += 1
