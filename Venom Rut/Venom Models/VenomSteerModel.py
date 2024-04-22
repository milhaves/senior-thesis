from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control
import control.matlab as cnt

g = 9.81

def getModelMDK(U):
    g = 9.81 #m/s/s, gravitational constant
    params=array([.451,.99,.0526,.506,22.717,.9246,.515,5.105,.2413,4.278,.2413,7.1,.125,.2,1.345])
    a ,b ,c,hrf,mrf,xff,zff,mff,Rfw,mfw,Rrw,mrw,Jyyf,Jyyr,lam = params
    mr = mrw+mrf #mass of rear frame includes rear wheel
    hr = (mrf*hrf+mrw*Rrw)/(mr) #CG height of rear frame includes rear wheel
    Sf = Jyyf/Rfw
    Sr = Jyyr/Rrw
    St = Sf+Sr
    mf = mff+mfw #total mass of front frame includes front wheel
    xf = (mff*xff+b*mfw)/mf# total front frame x position including front wheel
    hf = (zff*mff+Rfw*mfw)/mf #total front frame mass height including front wheel
    u = hf*cos(lam)-(b+c-xf)*sin(lam) #perpendicular distance from steer axis to front frame CG (positive if CG in front of steer axis)

    #build terms for the MDK model based on Eqn32
    M11 = mr*hr**2 + mf*hf**2
    M12 = -mf*hf*u- c*sin(lam)/b*(mf*xf*hf+mr*hr*a)
    M21 = -mf*hf*u- c*sin(lam)/b*(mf*xf*hf+mr*hr*a)
    M22 = mf*(u**2+2*c*sin(lam)/b*xf*u)+c**2*sin(lam)**2/b**2*(mr*a**2+mf*xf**2)
    D11 = 0
    D12 = U*sin(lam)/b*(-mr*hr*a - mf*xf*hf)-U*c*sin(lam)/b*(mf*hf+mr*hr)-U*c*sin(lam)/b*(Jyyf/Rfw+Jyyr/Rrw)-U*Jyyf/Rfw*sin(lam)
    D21 = U*c*sin(lam)/b*(Jyyf/Rfw+Jyyr/Rrw)+Jyyf/Rfw*U*sin(lam)
    D22 = U*sin(lam)/b*mf*xf*u + U*c*sin(lam)**2/b**2*(mr*a**2+mf*xf**2)+U*c*sin(lam)/b*mf*u+U*c**2*sin(lam)**2/b**2*(mf*xf+mr*a)
    K11 = -(mr*g*hr+mf*g*hf)
    K12 = mf*g*u+g*c*sin(lam)/b*(mf*xf+mr*a)-U**2*sin(lam)/b*(mf*hf+mr*hr)-U**2*sin(lam)/b*(Jyyf/Rfw+Jyyr/Rrw)
    K21 = mf*g*u + g*c*sin(lam)/b*(mr*a+mf*xf)
    K22 = -mf*g*u-g*c*sin(lam)*cos(lam)/b*(mr*a+mf*xf) + U**2*sin(lam)/b*mf*u+U**2*c*sin(lam)**2/b**2*(mf*xf+mr*a) + Jyyf/Rfw*U**2*sin(lam)/b

    M = array([[M11, M12],[M21, M22]])
    D = array([[D11, D12],[D21, D22]])
    K = array([[K11, K12],[K21, K22]])

    return M,D,K,eye(2)


def getModelSS(v):
    #first get the model in MDK form
    M,D,K,F = getModelMDK(v)
    A = hstack((zeros((2,2)),eye(2,2)))#first create the 'top half' of A matrix
    A = vstack((A,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))# now fill in bottom half
    B = vstack((zeros((2,2)),dot(linalg.inv(M),F)))#now stack zeros on top of our M^-1F term
    #trim B so it only takes steer.
    B = vstack(B[:,1])
    C = array([[1,0,0,0],[0,1,0,0]]) #choose the 'outputs' as just roll and steer
    D = 0 #with two possible inputs, two possible outputs, there are four terms in the D matrix when output eqn is y = Cx+Du
    sys = control.StateSpace(A,B,C,D)
    return sys,A,B,C,D

def getClosedLoopVenom(sys,Klqr):
    """
    syscl, eigs = getClosedLoopRazor(sys,Klqr)

    Given a set of state feedback gains and a matrix of gains (presumably from LQR,
    but could be any gains), return the closed loop system. This function can be called
    to do an eigenvalue study by designing ONE set of gains and testing them at different speeds

    """
    #create a closed loop feedback system with this state feedback CONTROLLER
    Acl = sys.A - dot(sys.B,Klqr)
    # Bcl = dot(sys.B,Klqr)
    #only keep the first column of this, since we only care about roll as an input
    # Bcl = Bcl[:,0]

    # Bcl = array([[0],[0],[0],[0],[1]])
    Bcl = array([[1],[0],[0],[0]])

    Ccl = vstack((eye(4),Klqr))
    Dcl = 0

    print("####### CLOSED LOOP DYNAMICS [roll steer rollrate steerrate int_error] #########")
    print("A: "+str(Acl))
    print("B: "+str(Bcl))
    print("C: "+str(Ccl))
    print("D: "+str(Dcl))

    print("####### CLOSED LOOP EIGS: for ROLL control ######")
    eigs,vecs = linalg.eig(Acl)
    print(eigs)
    syscl = control.ss(Acl,Bcl,Ccl,Dcl)
    return syscl, eigs

def getRollLQRVenom(velocity,R=0.001):
    sys,A,B,C,D = getModelSS(velocity)
    Q = eye(4)/100.0
    Q[1,1] = 0
    # Q[4,4] =1
    #set up LQR weight on input matrix, which is
    #R = array([[1,0],[0,.01]])
    # R = .01
    Klqr, Slqr, Elqr = control.lqr(sys, Q, R)
    print("######### LQR GAINS for ROLL control #############")
    print("K = array([" + str(Klqr[0,0]) + "," + str(Klqr[0,1]) + "," + str(Klqr[0,2]) + "," + str(Klqr[0,3]) + "])")
    return sys,Klqr

def designClosedLoopVenom(velocity):
    """
    syscl, Klqr = designClosedLoopRazor(velocity)
    This is a wrapper that builds and designs a closed-loop LQR for the motorcycle.
    The input to the closed loop system is a desired roll angle

    """
    sys,Klqr = getRollLQRVenom(velocity)
    syscl,eigs = getClosedLoopVenom(sys,Klqr)
    return syscl,Klqr,eigs

def main():
    velocity = 1.75
    syscl,Klqr,eigs = designClosedLoopVenom(velocity)
    print('######### Eigs #########')
    print(eigs)

    goalAccel = -0.1*g #goal lateral acceleration is U^2/R
    goalRadius = velocity**2/goalAccel
    goalYawRate = velocity/goalRadius

    #now what does that mean for roll?
    goalRoll = -arctan(velocity*goalYawRate/9.81)
    # goalRoll = 0
    # #what does it mean for goal steering? delta = L/R, ignoring trig stuff.
    # goalSteer = w/goalRadius
    tsim = linspace(0,20,1000)

    xdesired = zeros((len(tsim),1))

    xdesired[:,0] = goalRoll


    ycl,tsim_out,xcl = cnt.lsim(syscl,xdesired,tsim)
    #compute steer torque

    ######### Virtual Mass Spring Damper Model #########
    m1 = 39.2
    m2 = 15
    l1 = 0.6858
    lc1 = 0.441
    l2 = 0.5
    lc2 = l2
    # I1 = 0
    # I2 = 0
    I1 = (1/3)*m1*lc1*lc1
    I2 = (1/3)*m2*(l1+lc2)**2

    #smallest eigenvalues at 1.75 m/s
    s1 = -10000000
    for index1 in eigs:
        if index1 > s1:
            s1 = index1
    s2 = -10000000
    for index2 in eigs:
        if (index2>s2) and (index2<s1):
            s2 = index2
    print('######### s1 #########')
    print(s1)
    print('######### s2 #########')
    print(s2)

    # J = ((1/3)*m1*lc1*lc1)+((1/3)*m2*((l1+lc2)**2))
    J = I1+I2
    # bvirtual = (-1/2)*J*(s1+s2) #virtual damper
    # Kvirtual = (-3*bvirtual**2-8*bvirtual*J*s1-4*J**2*s1**2)/(4*J) #virtual spring
    bvirtual = -1*J*(s1+s2)
    Kvirtual = J*s1*s2
    mtot = m1+m2

    square = (ycl[-1,0]/goalRoll)*s1*s2 #steady state gain

    print('######### J #########')
    print(J)
    print('######### bvirtual #########')
    print(bvirtual)
    print('######### Kvirtual #########')
    print(Kvirtual)

    tsimVirt = linspace(0,20,1000)
    xVirt = zeros((len(tsimVirt),1))
    # xdotVirt = zeros((len(tsimVirt),1))
    # xddotVirt = zeros((len(tsimVirt),1))
    # print(xVirt)


    # for w in range(1,len(tsimVirt)):
    #     xdotVirt = G/(xVirt[w-1]**2+(bvirtual/J)*xVirt[w-1]+(Kvirtual/J))
    #     xVirt[w] = xdotVirt*(tsimVirt[w]-tsimVirt[w-1])+xVirt[w-1]

    P = control.TransferFunction(square,[1,bvirtual/J,Kvirtual/J])

    # s = control.TransferFunction.s

    # P = G/(s**2+(bvirtual/J)*s+(Kvirtual/J))

    # xVirt, tsimVirt = step(P, tsimVirt, 0)
    xVirt,tsimVirt,something = cnt.lsim(P,xdesired,tsim)

    figure()

    # subplot(3,1,1)
    title("Closed Loop Step Response: Desired Roll = "+"{:.2f}".format(goalRoll)+" radians")
    # plot(tsim,goalRoll*ones((len(tsim),1)),'k--',tsim,ycl[:,0],'k')
    plot(tsim,goalRoll*ones((len(tsim),1)),'k--',tsim,ycl[:,0],'k',tsimVirt,xVirt,'r')
    xlabel('Time (s)')
    ylabel('Roll Angle (rad)')
    legend(['Desired Roll Angle','Bike Model Steer Response','VMSD Response'])
    # subplot(3,1,2)
    # plot(tsim,ycl[:,1],'k')
    # ylabel('Steer Angle (rad)')
    # subplot(3,1,3)
    # plot(tsim,ycl[:,4],'k')
    # xlabel('Time (s)')
    # ylabel('Steer Torque (Nm)')
    show()


if __name__ == '__main__':
    main()
