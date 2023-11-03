from numpy import *



class LaneEstimator:
    def __init__(self,lamda=1.16,wheelbase =.767):
        """
        This class estimates lateral position based on speed and steer angle.
        In the future, it may use a Kalman Filter, GPS, and yaw rate measurements to assist.
        Currently, it simply integrates estimates of yaw rate that come from steering and fwd speed.
        """
        self.yaw_est = 0
        self.y_est = 0
        self.lamda = lamda
        self.wheelbase = wheelbase

    def update(self,U,delta,dt):
        #first calculate yaw rate based on steer, lamda, wheelbase
        #note that this is DIFFERENT than the yaw rate you'd measure at the imu
        #this is the yaw rate measured about ground plane normal!!
        #it assumes no tire slip.
        yawrate = U*sin(self.lamda)*delta/self.wheelbase
        #now compute global Y velocity
        yvel = sin(self.yaw_est)*U
        #now integrate yaw angle using Euler integration
        self.yaw_est += dt*yawrate
        #now integrate y position using Euler integration
        self.y_est += dt*yvel
        #now return the state estimates
        return yawrate,self.yaw_est,yvel,self.y_est
