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
        self.dt = params.dt
        self.q  = params.q
        self.dim_state = params.dim_state
        pass

    def F(self):
        # system matrix
        dt = self.dt
        return np.matrix([[1, 0, 0, dt,  0,   0],
                          [0, 1, 0,  0, dt,   0],
                          [0, 0, 1,  0,  0,  dt], # dt not needed, we are tracing cars on the ground not planes!
                          [0, 0, 0,  1,  0,   0],
                          [0, 0, 0,  0,  1,   0],
                          [0, 0, 0,  0,  0,   1]])

    def Q(self):
        # process noise covariance Q
        q = self.q
        dt = self.dt
        q1 = ((dt**3)/3) * q
        q2 = ((dt**2)/2) * q
        q3 = dt * q
        return np.matrix([[q1, 0,  0, q2,  0,   0],
                          [0, q1,  0,  0, q2,   0],
                          [0,  0, q1,  0,  0,  q2], # vz not contributing to the noise
                          [q2, 0,  0, q3,  0,   0],
                          [0, q2,  0,  0, q3,   0],
                          [0,  0, q2,  0,  0,  q3]])

    def predict(self, track: Track):
        # predict state and estimation error covariance to next timestep
        F = self.F()
        x = F*track.x # state prediction
        P = F*track.P*F.transpose() + self.Q() # covariance prediction
        track.set_P(P)
        track.set_x(x)

    def update(self, track: Track, meas: Measurement):
        # update state and covariance with associated measurement
        H = meas.sensor.get_H(track.x) # measurement matrix
        K = track.P*H.transpose()*np.linalg.inv(self.S(track, meas, H)) # Kalman gain
        x = track.x + K*self.gamma(track, meas) # state update
        I = np.identity(self.dim_state)
        P = (I - K*H) * track.P # covariance update
        track.set_x(x)
        track.set_P(P)
        track.update_attributes(meas)


    def gamma(self, track: Track, meas: Measurement)-> np.ndarray:
        return meas.z - meas.sensor.get_hx(track.x)

    def S(self, track: Track, meas: Measurement, H: np.ndarray) -> np.ndarray:
        P = track.P
        return H*P*H.transpose() + meas.R
