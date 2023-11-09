from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control

g = 9.81

def getModelMDK ():
  g = -9.81
  m1 = 1
  m2 = 1
  l1 = 1
  l2 = 1

  M11 = m1*l1*l1+m2*l1*l1
  M12 = 2*m2*l1*l2
  M21 = 2*m2*l1*l2
  M22 = m2*l2*l2
  D11 = 0
  D12 = 0
  D21 = 0
  D22 = 0
  K11 = -g*m1*l1-g*m2*l1-g*m2*l2
  K12 = -g*m2*l2
  K21 = -g*m2*l2
  K22 = -g*m2*l2

  M = array([[M11,M12],[M21,M22]])
  D = array([[D11,D12],[D21,D22]])
  K = array([[K11,K12],[K21,K22]])

  return M,D,K,eye(2)

def getModelSS():
    M,D,K,F = getModelMDK()
    Ass = hstack((zeros((2,2)),eye(2,2)))
    Ass = vstack((Ass,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))
    Ass = hstack((Ass,zeros((4,1))))
    Ass = vstack((Ass,array([-1, 0, 0, 0, 0])))
    Bss = vstack((zeros((2,2)),dot(linalg.inv(M),F)))
    #Bss = vstack(Bss[:,1])
    #Bss_control = vstack((vstack(Bss[:,1]),array([1])))
    Bss_temp = hstack((Bss,vstack(array([0,0,0,0]))))
    Bss_control = vstack((Bss_temp,array([0,0,1])))
    print("######### Bss #############")
    print(Bss_control)

    Css = array([[0,0,0,1,0]]) #replace with identity matrix or first two ones
    print("######### Css #############")
    print(Css)

    Dss = zeros((1,3))
    print("######### Dss #############")
    print(Dss)

    sys = control.StateSpace(Ass,Bss_control,Css,Dss)
    return sys,Ass,Bss_control,Css,Dss

def getClosedLoopPendulums(sys,Klqr):
    Acl = sys.A - dot(sys.B,Klqr)
    Bcl = dot(sys.B,Klqr)
    # Bcl = vstack(Bcl[:,2])
    # Bcl = sys.B

    Ccl = vstack((eye(5),Klqr))
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
    Q = eye(5)*1000
    #Q[0,0] = 0
    Q[1,1] = 0
    #Q[4,4] = 0

    R = eye(3)

    Klqr,Slqr,Elqr = control.lqr(sys,Q,R)
    print("######### LQR GAINS for ROLL control #############")
    print(Klqr)
    return sys,Klqr

def designClosedLoopPendulums():
    sys,Klqr = getRollLQRPendulums()
    syscl,eigs = getClosedLoopPendulums(sys,Klqr)
    return syscl,Klqr

def main():
    #velocity = 1 #rad/s
    sys,Ass,Bss,Css,Dss = getModelSS()
    syscl,Klqr = designClosedLoopPendulums()

    print("######### syscl #############")
    print(syscl)

    #goalAccel = 1 #rad/sec^2

    goalRoll = 0.25 #rad

    tsim = linspace(0,10,1000)

    xdesired = zeros((len(tsim),5))

    xdesired[:,0] = goalRoll

    import control.matlab as cnt
    ycl,tsim_out,xcl = cnt.lsim(syscl,xdesired,tsim)

    print("######### ycl #############")
    print(ycl)

    figure()
    subplot(3,1,1)
    title("Closed Loop Step Response: Desired Roll = "+"{:.2f}".format(goalRoll*180/pi)+" degrees")
    plot(tsim,goalRoll*ones((len(tsim),1)),'k--',tsim,ycl[:,0],'k')
    xlabel('Time (s)')
    ylabel('Roll Angle (rad)')
    legend(['desired','actual'])
    subplot(3,1,2)
    plot(tsim,ycl[:,1],'k')
    ylabel('Lean Angle (rad)')
    subplot(3,1,3)
    plot(tsim,ycl[:,5],'k')
    xlabel('Time (s)')
    ylabel('Hinge Torque (Nm)')
    show()

if __name__ == '__main__':
    main()
