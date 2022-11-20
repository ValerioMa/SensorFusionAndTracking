# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections
from typing import List

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params
from student.measurements import Measurement

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas: Measurement, id):
        print('creating track no.', id)

        # Initializing state
        self.x = np.zeros((6,1))
        self.P = np.zeros((6, 6))

        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates

        # transform measurement to vehicle coordinates
        pos_sens = np.ones((4, 1)) # homogeneous coordinates
        pos_sens[0:3] = meas.z[0:3]
        pos_veh = meas.sensor.sens_to_veh*pos_sens

        self.x[0:3] = pos_veh[0:3]
        self.P[0:3, 0:3] = M_rot * meas.R * np.transpose(M_rot)

        # Set speed covariance
        self.P[3:6, 3:6] = np.matrix([[params.sigma_p44**2, 0, 0],
                        [ 0, params.sigma_p55**2, 0],
                        [0, 0, params.sigma_p66**2]])

        self.state = 'initialized'
        self.score = 1./params.window

        ############
        # END student code
        ############

        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw =  np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

    def decrease_score(self):
        self.score -= 1./params.window

    def increase_score(self):
        self.score += 1./params.window

        if self.state == 'initialized' and self.score > params.tentative_threshold:
            self.state = 'tentative'

        if self.state =='tentative' and self.score > params.confirmed_threshold:
            self.state = 'confirmed'

    def set_x(self, x):
        self.x = x

    def set_P(self, P):
        self.P = P

    def set_t(self, t):
        self.t = t

    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c*meas.width + (1 - c)*self.width
            self.length = c*meas.length + (1 - c)*self.length
            self.height = c*meas.height + (1 - c)*self.height
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates


###################

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self):
        self.N = 0 # current number of tracks
        self.track_list: List[Track] = []
        self.last_id = -1
        self.result_list = []

    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):
        ############
        # TODO Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big (check params.py for parameters that might be helpful, but
        # feel free to define your own parameters)
        ############

        # decrease score for unassigned tracks
        for i in unassigned_tracks:
            track = self.track_list[i]
            # check visibility
            if meas_list: # if not empty
                if meas_list[0].sensor.in_fov(track.x):
                    track.decrease_score()
                    pass

        # delete old tracks
        def filter_tracks(track: Track):
            return ((track.state == 'confirmed' and track.score < params.delete_threshold) or
                    max(track.P[0,0], track.P[1,1]) > params.max_P)

        track_to_delete = list(filter(filter_tracks, self.track_list))
        for track in track_to_delete:
          self.delete_track(track)
        ############
        # END student code
        ############

        # initialize new track with unassigned measurement
        for j in unassigned_meas:
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
                self.init_track(meas_list[j])

    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        print('deleting track no.', track.id)
        self.track_list.remove(track)

    def handle_updated_track(self, track: Track) -> None:
        ############
        # TODO Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############
        track.increase_score()

        pass

        ############
        # END student code
        ############