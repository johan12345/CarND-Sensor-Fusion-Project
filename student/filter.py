# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys

from student.measurements import Measurement
from student.trackmanagement import Track

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############

        dt = params.dt
        return np.matrix([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        q = params.q
        dt = params.dt
        a = 1 / 3 * dt ** 3 * q
        b = 1 / 2 * dt ** 2 * q
        c = dt * q
        return np.matrix([
            [a, 0, 0, b, 0, 0],
            [0, a, 0, 0, b, 0],
            [0, 0, a, 0, 0, b],
            [b, 0, 0, c, 0, 0],
            [0, b, 0, 0, c, 0],
            [0, 0, b, 0, 0, c]
        ])
        
        ############
        # END student code
        ############ 

    def predict(self, track: Track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        x = F * track.x  # state prediction
        P = F * track.P * F.T + self.Q()  # covariance prediction
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track: Track, meas: Measurement):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############

        gamma = self.gamma(track, meas)
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)

        K = track.P * H.T * np.linalg.inv(S)
        x = track.x + K * gamma
        I = np.eye(6)
        P = (I - K * H) * track.P

        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track: Track, meas: Measurement):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        return meas.z - meas.sensor.get_hx(track.x)
        
        ############
        # END student code
        ############ 

    def S(self, track: Track, meas: Measurement, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return H * track.P * H.T + meas.R
        
        ############
        # END student code
        ############ 