from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control

def getModelMDK (U,printmodel=False):
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

def getModelSS(v):
    M,D,K,F = getModelMDK(v)
    A = hstack((zeros((2,2)),eye(2,2)))
    A = vstack((A,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))
    B = vstack((zeros((2,2)),dot(linalg.inv(M),F)))
    C = array([[1,0,0,0],[0,1,0,0]]) #I think I have to change something here to get theta1 and theta2
    D = zeros((2,2))
    sys = control.StateSpace(A,B,C,D)
    return sys

def getClosedLoopPendulums(sys,Klqr):
    Acl = sys.A - dot(sys.B,Klqr)
    Bcl = dot(sys.B,Klqr)
    Bcl = Bcl[:,1]

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

def designClosedLoopPendulums(velocity): #Since the model is stationary, what would be the input here? I'm assuming zero
    sys,Klqr = getRollLQRPendulums(velocity)
    syscl,eigs = getClosedLoopPendulums(sys,Klqr)
    return syscl,Klqr

def main():
    syscl
