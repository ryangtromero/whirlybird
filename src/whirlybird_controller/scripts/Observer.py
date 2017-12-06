#!/usr/bin/env python
# license removed for brevity


# This file is a basic structure to write a observer that
# communicates with ROS. It will be the students responsibility
# tune the gains and fill in the missing information

# As an example this file contains PID gains, but other
# observers use different types of gains so the class
# will need to be modified to accomodate those changes

import rospy
import time
import numpy as np
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from numpy import *             # Grab all of the NumPy functions
from matplotlib.pyplot import * # Grab MATLAB plotting functions
from control.matlab import *    # MATLAB-like functions



class Observer():

    def __init__(self):

        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')

        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        self.d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        self.km = self.param['km']
        t_r_theta = self.param['t_r_theta']
        zeta_theta = self.param['zeta_theta']
        t_r_phi = self.param['t_r_phi']
        zeta_phi = self.param['zeta_phi']
        self.t_r_psi = self.param['t_r_psi']
        self.zeta_psi = self.param['t_r_psi']
        self.kd_theta = 2.2*zeta_theta/(t_r_theta*1.152)
        self.kp_theta = 2.2**2/(t_r_theta**2*1.152)
        self.kp_phi = (2.2/t_r_phi)**2*Jx
        self.kd_phi = (4.4*zeta_phi*Jx)/t_r_phi


        # Roll Gains
        self.P_phi = 0.0
        self.I_phi = 0.0
        self.D_phi = 0.0
        self.Int_phi = 0.0
        self.prev_phi_error = 0.0
        self.prev_phi = 0.0

        # Pitch Gains
        self.theta_r = 0.0
        self.P_theta = 0.0
        self.I_theta = 0.0
        self.D_theta = 0.0
        self.prev_theta = 0.0
        self.prev_theta_error = 0.0


        # Yaw Gains
        self.psi_r = 0.0
        self.P_psi = 0.0
        self.I_psi = 0.0
        self.D_psi = 0.0
        self.prev_psi = 0.0
        self.prev_psi_error = 0.0

        #Observer Param
        self.x_hat_lon = np.zeros((2,1))
        self.x_hat_lat = np.zeros((4,1))
        self.prev_F = 0.0;
        self.prev_tau = 0.0;
        self.F = 0.0;
        self.tau = 0.0;

        self.prev_time = rospy.Time.now()

        self.Fe = 0.0 #Note this is not the correct value for Fe, you will have to find that yourself

        self.observer_sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        self.plant_sub_ = rospy.Subscriber('command', Command, self.commandCallback, queue_size=5)
        self.observer_pub_ = rospy.Publisher('x_hat', Twist, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def commandCallback(self, msg):
        ul = msg.left_motor
        ur = msg.right_motor
        self.F = self.km*ul + self.km*ur
        self.tau = self.km*self.d*ul - self.km*self.d*ur

    def whirlybirdCallback(self, msg):
        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']
        ki_theta = self.param['kitheta']
        ki_psi = self.param['kipsi']
        kd_theta = self.kd_theta
        kp_theta = self.kp_theta
        kd_phi = self.kd_phi
        kp_phi = self.kp_phi

        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw



        #Calculate yaw gains
        t_r_psi = self.t_r_psi
        zeta_psi = self.zeta_psi
        Fe = (m1*l1-m2*l2)*g*np.cos(theta)/l1
        Fe_new = (m1*l1-m2*l2)*g*np.cos(0)/l1
        b_psi = (l1*Fe_new)/(m1*l1**2+m2*l2**2+Jz)
        kp_psi = (2.2/t_r_psi)**2/b_psi
        kd_psi = (4.4*zeta_psi)/(b_psi*t_r_psi)

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        theta_dot = (theta-self.prev_theta)/dt
        psi_dot = (psi-self.prev_psi)/dt
        phi_dot = (phi-self.prev_phi)/dt


        #################################
        #Calculate K_r and K matrices
        wn_theta = 2.2/1*5
        wn_phi = 2.2/.25*5
        wn_psi = 2.2/2.5*5
        zeta_theta = 0.8
        zeta_phi = 0.9
        zeta_psi = 0.8

        #syms theta_eq
        theta_eq = 0

        Fe = (m1*l1-m2*l2)*g/l1

        # lateral dynamics
        A_lat = matrix([[0, 0, 1, 0], \
        [0, 0, 0, 1], \
        [0, 0, 0, 0], \
        [l1*Fe/(m1*l1**2+m2*l2**2+Jz), 0, 0, 0]])

        B_lat = matrix([0, 0, 1/Jx, 0]).transpose()
        Cr_lat = matrix([[1, 0, 0, 0],[0, 1, 0, 0]] )

        C_lat = concatenate((B_lat, A_lat*B_lat, A_lat**2*B_lat, A_lat**3*B_lat),axis=1)

        #print rank(C_lat)

        # longitudinal
        A_lon = matrix([[0, 1], \
        [(m1*l1-m2*l2)*g*sin(theta_eq)/(m1*l1**2+m2*l2**2+Jy), 0]])

        B_lon = matrix([0, l1/(m1*l1**2+m2*l2**2+Jy)]).transpose()
        Cr_lon = matrix([1, 0])

        # Lateral pole placement

        p_lat = [-zeta_phi*wn_phi + wn_phi*sqrt(1-zeta_phi**2)*1j, -zeta_phi*wn_phi - wn_phi*sqrt(1-zeta_phi**2)*1j, \
            -zeta_psi*wn_psi + wn_psi*sqrt(1-zeta_psi**2)*1j, -zeta_psi*wn_psi - wn_psi*sqrt(1-zeta_psi**2)*1j]
        L_lat = self.Obs_gains(A_lat, Cr_lat, p_lat)

        #############################

        # Longitudinal pole placement

        p_lon = [-zeta_theta*wn_theta + 1j*wn_theta*sqrt(1-zeta_theta**2),-zeta_theta*wn_theta - 1j*wn_theta*sqrt(1-zeta_theta**2)]
        L_long= self.Obs_gains(A_lon, Cr_lon, p_lon)

        ##################################


        x_long = matrix([[theta],[theta_dot]])
        x_lat = matrix([[phi], [psi], [phi_dot], [psi_dot]])


        # error_theta = self.theta_r-theta
        # self.I_theta = self.I_theta + (dt/2)*(error_theta+self.prev_theta_error)
        # error_psi = self.psi_r - psi
        # self.I_psi = self.I_psi + (dt/2)*(error_psi+self.prev_psi_error)
        Fe = (m1*l1-m2*l2)*g*np.cos(theta)/l1
        lat_y = matrix([phi,psi]).T
        for i in range(10):
            self.x_hat_lon = self.x_hat_lon + dt/10*(A_lon*self.x_hat_lon + B_lon*(self.F-Fe) + L_long*(theta-Cr_lon*self.x_hat_lon))
            self.x_hat_lat = self.x_hat_lat + dt/10*(A_lat*self.x_hat_lat + B_lat*self.tau + L_lat*(lat_y-Cr_lat*self.x_hat_lat))


        ##################################

        # Pack up and send command
        x_hat = Twist()
        x_hat.linear.x = self.x_hat_lon[0]
        x_hat.linear.y = self.x_hat_lon[1]
        x_hat.linear.z = self.x_hat_lat[0]
        x_hat.angular.x = self.x_hat_lat[1]
        x_hat.angular.y = self.x_hat_lat[2]
        x_hat.angular.z = self.x_hat_lat[3]
        self.observer_pub_.publish(x_hat)
        self.prev_theta = theta
        self.prev_phi = phi
        self.prev_psi = psi
        self.prev_F = self.F
        self.prev_tau = self.tau
    def Obs_gains(self, A,Cr,poles):

        # n = np.size(A,1)

        # A1 = np.matlib.zeros((n+1,n+1))
        # B1 = np.matlib.zeros((n+1,1))
        # C1 = np.matlib.zeros((1,n+1))
        # # Augment A matrix
        # A1[0:n,0:n] = A
        # A1[n,0:n] = -Cr
        # # Augment B matrix
        # B1[0:n,0] = B
        # # Augment C matrix
        # C1[0,0:n] = Cr

        # Create controllability matrix
        # CC1 = np.matlib.zeros((n+1,n+1))
        #
        # for i in range(n):
        #     CC1[0:n,i] = (A**n)*B
        # for i in range(n):
        #     CC1[n,i] = -Cr*(A**n*B)

        L = place(np.transpose(A),np.transpose(Cr),poles).T

        # K = K1[0,0:n]
        # kI = K1[0,n]

        return L



if __name__ == '__main__':
    rospy.init_node('observer', anonymous=True)
    try:
        observer = Observer()
    except:
        rospy.ROSInterruptException
    pass
