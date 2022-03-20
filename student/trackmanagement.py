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
from typing import List

import numpy as np
import collections

# add project directory to python path to enable relative imports
import os
import sys

from student.measurements import Measurement

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas: Measurement, id):
        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        ############
        # TODO Step 2: initialization:
        # - replace fixed track initialization values by initialization of x and P based on 
        # unassigned measurement transformed from sensor to vehicle coordinates
        # - initialize track state and track score with appropriate values
        ############

        # trasform measured position to vehicle coordinates
        pos_sens = np.ones((4, 1))
        pos_sens[0:3] = meas.z[0:3]
        pos_veh = meas.sensor.sens_to_veh * pos_sens

        # transform measurement covariance to vehicle coordinates
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3]
        P_pos = M_rot * meas.R * M_rot.T
        P_vel = np.diag([params.sigma_p44 ** 2, params.sigma_p55 ** 2, params.sigma_p66 ** 2])

        self.x = np.zeros((params.dim_state, 1))
        self.x[0:3] = pos_veh[0:3]
        self.P = np.zeros((params.dim_state, params.dim_state))
        self.P[0:3, 0:3] = P_pos
        self.P[3:6, 3:6] = P_vel

        self.state = 'initialized'
        self.score = 1 / params.window
        
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

            # NOTE: this does not work if there are no measurements, because in this case we do not have
            # access to the sensor object to check whether the track is in the FOV.
            # This could be changed if I update the code so that the current sensor is passed to the manage_tracks
            # method, however it seems that there are no camera detections in this example, so the track would be
            # deleted immediately.
            if meas_list: # if not empty
                if meas_list[0].sensor.in_fov(track.x):
                    # your code goes here
                    track.score -= 1/params.window

        # delete old tracks
        for track in self.track_list:
            if track.state == 'confirmed' and track.score < params.delete_threshold or \
                track.score < params.delete_threshold_tentative or \
                    (track.P.diagonal()[:2] > params.max_P).any():
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
        
    def handle_updated_track(self, track):      
        ############
        # TODO Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############

        track.score = min(1, track.score + 1/params.window)
        if track.score >= params.confirmed_threshold:
            track.state = "confirmed"
        else:
            track.state = "tentative"
        
        ############
        # END student code
        ############ 