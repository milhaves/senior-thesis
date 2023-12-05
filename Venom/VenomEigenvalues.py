from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control

def getModelMDK (U,printmodel=False):
    a = 0.474 #.451#meters, distance from rear axle to CG in x direction
    b = .99#meters, wheelbase of bike
    c = .0526#.08#meters, trail
    hrf = 0.776#.506#meters, rear frame CG height
    mrf = 22.717#kg, rear frame mass inc. rider
    xff = .9246#position of front frame CG
    yff = 0#y-position of the front frame CG
    zff = .515#height of the front frame CG
    mff = 5.105 #kg, fork mass
    Rfw = .2413 #m, radius of the front wheel
    mfw = 4.278 #kg, wheel mass
    Rrw = .2413 # radius of real wheel
    mrw = 7.1 #mass of rear wheel

    mp = 15 #kg, mass of pendulum
    hp = 0.5 + 0.6858 #m, height of pendulum mass from the ground

    mr = mrw+mrf+mp #mass of rear frame includes rear wheel
    hr = (mrf*hrf+mrw*Rrw+mp*hp)/(mr) #CG height of rear frame includes rear wheel
    Jyyf = mfw*Rfw**2
    Jyyr = mrw*Rrw**2
    Sf = Jyyf/Rfw
    Sr = Jyyr/Rrw
    St = Sf+Sr
    mf = mff+mfw #total mass of front frame includes front wheel
    xf = (mff*xff+b*mfw)/mf# total front frame x position including front wheel
    hf = (zff*mff+Rfw*mfw)/mf #total front frame mass height including front wheel
    lam = 1.345 #radians, angle from negative x axis to steering axis
    g = 9.81 #m/s/s, gravitational constant
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
    C = array([[1,0,0,0],[0,1,0,0]]) #choose the 'outputs' as just roll and steer
    D = zeros((2,2)) #with two possible inputs, two possible outputs, there are four terms in the D matrix when output eqn is y = Cx+Du
    sys = control.StateSpace(A,B,C,D)
    return sys

def main():

    # #pick a velocity
    # velocity = 10 #m/s
    # #get the state space model
    # sys = getModelSS(velocity)
    # #check the eigenvalues. All real parts must be <0 for stability.
    # eigs,vecs = linalg.eig(sys.A)
    # print("Eigenvalues at "+str(velocity)+" m/s are:")
    # print(eigs)

###########################################################################################

    #create a vector of velocities to investigate
    vvec = arange(.01,10,.01)
    #create a second copy of this vector that is four rows. This will make plotting easier
    vvec2 = zeros((4,len(vvec)))

    #our model is fourth order, so will have 4 eigenvalues. each can have a real and/or imaginary part.
    eigs_re = zeros((4,len(vvec)))
    eigs_im = zeros((4,len(vvec)))

    for k in range(0,len(vvec)):
        #get current velocity
        v = vvec[k]
        #get state space model at this speed
        sys= getModelSS(v)
        #get eigenvalues at this speed
        eigs,vecs = linalg.eig(sys.A)
        #get real parts and place in proper matrix for storage
        eigs_re[:,k] = real(eigs)
        #get imaginary parts and place in proper matrix for storage
        eigs_im[:,k] = imag(eigs)
        #fill up velocity vector corresponding with each eigenvalue
        vvec2[:,k] = [v,v,v,v]

    #creat a figure for us to plot the eigenvalues.
    figure()
    plot(vvec,eigs_re[0,:],'k.',vvec,eigs_im[0,:],'k')
    xlabel('Speed (m/s)')
    ylabel('Eig (1/s)')
    legend(['Real Part','Imaginary part'])
    plot(vvec,eigs_re[1,:],'k.',vvec,abs(eigs_im[1,:]),'k')
    plot(vvec,eigs_re[2,:],'k.',vvec,abs(eigs_im[2,:]),'k')
    plot(vvec,eigs_re[3,:],'k.',vvec,abs(eigs_im[3,:]),'k')
    #limit our view so we can see what's happening
    axis([0,10,-30,30])
    show()

if __name__ == '__main__':
    main()
