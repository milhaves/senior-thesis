from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control

g = 9.81

def getModelMDK ():
  g = 9.81
  m1 = 1
  m2 = 1
  l1 = 1
  lc1 = 0.5
  l2 = 2
  lc2 = 1
  I1 = .083
  I2 = .33

  M11 = I1+I2+(m2*l1*l1)+(m2*lc2*lc2)+(m1*lc1*lc1)+(2*m2*l1*lc2) #unsure about the 2
  M12 = I2+(m2*l1*lc2)+(m2*lc2*lc2)
  M21 = I2+(m2*l1*lc2)+(m2*lc2*lc2)
  M22 = I2+(m2*l1*lc2)
  D11 = 0
  D12 = 0
  D21 = 0
  D22 = 0
  K11 = -g*m1*lc1-g*m2*l1-g*m2*lc2
  K12 = -g*m2*lc2
  K21 = -g*m2*lc2
  K22 = -g*m2*lc2

  M = array([[M11,M12],[M21,M22]])
  D = array([[D11,D12],[D21,D22]])
  K = array([[K11,K12],[K21,K22]])

  return M,D,K,eye(2)

def getModelSS():
    M,D,K,F = getModelMDK()
    # Ass = hstack((zeros((2,2)),eye(2,2)))
    # Ass = vstack((Ass,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))
    # Ass = hstack((Ass,zeros((4,1))))
    # Ass = vstack((Ass,array([-1, 0, 0, 0, 0])))
    # Bss = vstack((zeros((2,2)),dot(linalg.inv(M),F)))
    # # Bss = vstack((0,0,dot(linalg.inv(M),F),0))
    # print("######### Bss #############")
    # print(Bss)
    # Bss = vstack((Bss[:,1]))
    # Bss_control = vstack((Bss,array([0]))) #uncomment from here up
    #Bss_control = vstack((vstack(Bss[:,1]),array([1])))
    # Bss_control = vstack((array([0,0]),Bss,array([0])))
    # print("######### Bss_control #############")
    # print(Bss_control)

    # Ass = array([[0,0,1,0],[0,0,0,1],[12.49,-12.54,0,0],[-14.49,29.36,0,0]])
    # Bss_control = array([[0],[0],[-2.98],[5.98]])

    #---------------------------------------------------------------------------------------------------------

    M,D,K,F = getModelMDK()
    Ass = hstack((zeros((2,2)),eye(2,2)))
    Ass = vstack((Ass,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))
    # Ass = hstack((Ass,zeros((4,1))))
    # Ass = vstack((Ass,array([-1, 0, 0, 0, 0])))
    Bss = vstack((zeros((2,2)),dot(linalg.inv(M),F)))
    # Bss = vstack((0,0,dot(linalg.inv(M),F),0))
    # print("######### Bss #############")
    # print(Bss)
    Bss = vstack((Bss[:,1]))
    # Bss_control = vstack((Bss,array([0]))) #uncomment from here up

    #---------------------------------------------------------------------------------------------------------

    # Css = array([[0,0,0,1,0]])
    Css = eye(4)
    print("######### Css #############")
    print(Css)

    Dss = 0
    print("######### Dss #############")
    print(Dss)

    sys = control.StateSpace(Ass,Bss,Css,Dss)
    return sys,Ass,Bss,Css,Dss

def getClosedLoopPendulums(sys,Klqr):
    Acl = sys.A - dot(sys.B,Klqr)
    Bcl = dot(sys.B,Klqr)
    Bcl = vstack(Bcl[:,2])
    Bcl = sys.B
    # Bcl = array([[0],[0],[0],[0],[1]])

    Ccl = vstack((eye(4),Klqr))
    Dcl = 0

    print("####### CLOSED LOOP DYNAMICS #######")
    print("A: "+str(Acl))
    print("B: "+str(Bcl))
    print("C: "+str(Ccl))
    print("D: "+str(Dcl))

    print("####### CLOSED LOOP EIGS: for ROLL control #######")
    eigs,vecs = linalg.eig(Acl)
    print(eigs)
    syscl  = control.ss(Acl,Bcl,Ccl,Dcl)
    return syscl, eigs

def getRollLQRPendulums():
    sys,Ass,Bss,Css,Dss = getModelSS()
    Q = eye(4)
    # # Q[0,0] = Q[0,0]*1000
    # Q[1,1] = 0
    # Q[2,2] = Q[2,2]*10
    # Q[3,3] = 0
    # # Penalize Q[4,4] more to make it faster
    # Q[4,4] = Q[4,4]*10000
    # Q[4,4] = 0

    R = 1

    Klqr,Slqr,Elqr = control.lqr(sys,Q,R)
    print("######### LQR GAINS for ROLL control #############")
    # print(Klqr)
    # print("K = array([" + str(Klqr[0,0]) + "," + str(Klqr[0,1]) + "," + str(Klqr[0,2]) + "," + str(Klqr[0,3]) + "," + str(Klqr[0,4]) + "])")
    print("K = array([" + str(Klqr[0,0]) + "," + str(Klqr[0,1]) + "," + str(Klqr[0,2]) + "," + str(Klqr[0,3]) + "])")
    return sys,Klqr

def designClosedLoopPendulums():
    sys,Klqr = getRollLQRPendulums()
    syscl,eigs = getClosedLoopPendulums(sys,Klqr)
    return syscl,Klqr

def main():
    #velocity = 1 #rad/s
    data = loadtxt('pendulum_data3.txt', delimiter = ",")
    timeData = data[:,0]
    torqueData = data[:,2]
    rollData = data[:,3]
    leanData = data[:,4]

    sys,Ass,Bss,Css,Dss = getModelSS()
    syscl,Klqr = designClosedLoopPendulums()

    print("######### syscl #############")
    print(syscl)

    #goalAccel = 1 #rad/sec^2

    goalRoll = 0 #rad

    tsim = linspace(0,10,1000)

    xdesired = zeros((len(tsim),1))

    xdesired[:,0] = goalRoll

    import control.matlab as cnt
    # ycl,tsim_out,xcl = cnt.lsim(syscl,xdesired,tsim,[0.1,0,0,0,0])
    ycl,tsim_out,xcl = cnt.lsim(syscl,xdesired,tsim,[0.05,-0.05,0,0])

    print("######### ycl #############")
    print(ycl)

    figure()
    subplot(3,1,1)
    title("Closed Loop Step Response: Desired Roll = "+"{:.2f}".format(goalRoll*180/pi)+" degrees")
    # plot(tsim,goalRoll*ones((len(tsim),1)),'k--',tsim,ycl[:,0],'k',timeData,rollData,'r')
    plot(tsim,goalRoll*ones((len(tsim),1)),'k--',tsim,ycl[:,0],'k')
    xlabel('Time (s)')
    ylabel('Roll Angle (rad)')
    legend(['desired','modeled','simulated'])
    subplot(3,1,2)
    # plot(tsim,ycl[:,1],'k',timeData,leanData,'r')
    plot(tsim,ycl[:,1],'k')
    ylabel('Lean Angle (rad)')
    subplot(3,1,3)
    # plot(tsim,ycl[:,5],'k',timeData,torqueData,'r')
    plot(tsim,ycl[:,4],'k')
    # ylim(-100,100)
    xlabel('Time (s)')
    ylabel('Hinge Torque (Nm)')
    show()

if __name__ == '__main__':
    main()
