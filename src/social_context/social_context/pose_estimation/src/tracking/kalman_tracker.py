"""
Implements Kalman Filtering for 1 person
Remembers where a person was
Predicts where they will next frame
Needs position and velocity - [x, y, vx, vy]

"""

import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time

class KalmanPersonTracker():
    count = 0

    def __init__(self, initial_position, timestamp=None):
        """
        New person is detected
        Set up Kalman Filter for this person and 

        Args: 
            initial-position: (x, y)
            timestamp: Time of detection
        """

        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # Initialize state [x, y, vx=0, vy=0]
        x, y = initial_position
        self.kf.x = np.array([[x], [y], [0.], [0.]])
        
        # State transition matrix
        # next_x  = x + vx
        # next_y  = y + vy
        # next_vx = vx
        # next_vy = vy
        self.kf.F = np.eye(4)
        
        # Measurement matrix - we only measure position (x, y)
        # measured_x = x
        # measured_y = y
        self.kf.H = np.array([[1., 0., 0., 0.],
                                [0., 1., 0., 0.]])
        

        
        # Process noise covariance
        self.kf.Q = np.eye(4) * 0.1
        
        # Measurement noise covariance
        self.kf.R = np.eye(2) * 0.1
        
        # Initial state covariance
        self.kf.P = np.eye(4) * 1.0

        if timestamp is not None:
            self.last_predict_time = timestamp
        else:
            self.last_predict_time = time.time()  # Use current time if no timestamp is provided
        self.q_scale = 0.5
        self.max_dt = 2.0
        self.min_dt = 1e-3
    
        self.initial_position = initial_position

        KalmanPersonTracker.count += 1
        self.id = KalmanPersonTracker.count

        self.history = [initial_position]
        self.missed = 0
        self.hits = 1
        self.state = 'TENTATIVE'  # Can be 'TENTATIVE', 'CONFIRMED' (3 hits for now), 'DELETED'
        self.confidence = 0.0
        self.appearance_feature = None
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # Default orientation as a quaternion


    def update_appearance_feature(self, feature, ema_alpha=0.8):
        """
        Update the appearance feature using Exponential Moving Average (EMA)

        Args:
            feature: New appearance feature vector
            ema_alpha: EMA smoothing factor (0 < alpha < 1)
        """
        if feature is None:
            return 

        if self.appearance_feature is None:
            self.appearance_feature = feature #first time -> initialize
        else: #update using EMA
            self.appearance_feature = ema_alpha * self.appearance_feature + (1 - ema_alpha) * feature


    def predict(self, timestamp=None):
        """
        Predict the next position of the person based on the Kalman Filter

        Returns:
            predicted_position: (x, y)
        """

        if timestamp is not None:
            now = timestamp
        else:
            now = time.time()
        df = now - self.last_predict_time
        self.last_predict_time = now
        df = max(self.min_dt, min(df, self.max_dt))  # Clamp df to avoid extreme values

        self.kf.F = np.array([[1., 0., df, 0.],
                              [0., 1., 0., df],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])

        self.kf.Q = Q_discrete_white_noise(dim=2, dt=df, var=self.q_scale, block_size=2, order_by_dim=False)


        self.kf.predict()
        predicted_x = self.kf.x[0, 0]
        predicted_y = self.kf.x[1, 0]

        return (predicted_x, predicted_y)

    def update (self, new_position, confidence = 0.0, hits_threshold = 3, orientation = None):
        """
        Update the Kalman Filter with a new position measurement

        Args:
            new_position: (x, y)
            confidence: Detection confidence from MediaPipe (0.0 to 1.0)
            hits_threshold: Number of consecutive hits before a tracker is confirmed
        """
        self.kf.R = np.eye(2) * 0.1* (1.0 - confidence) + np.eye(2)*1e-6 # Adjust measurement noise based on confidence
        self.kf.update(np.array([[new_position[0]], [new_position[1]]]))
        self.history.append(new_position)
        if len(self.history) > 30:  # Keep only the last 30 positions
            self.history.pop(0)


        self.missed = 0
        self.hits += 1
        self.confidence = confidence

        # Threshold to confirm a track - if we have 3 hits, we consider it confirmed (COULD BE CHANGED)
        if self.state == 'TENTATIVE' and self.hits >= hits_threshold:
            self.state = 'CONFIRMED'
        
        if orientation is not None:
            self.orientation = orientation


    def mark_missed(self, missed_threshold = 15, tentative_missed_threshold = 7):
        """
        Mark the person as missed

        Args:
            missed_threshold: Number of consecutive misses before a tracker is deleted
            tentative_missed_threshold: Number of consecutive misses before a tentative tracker is deleted
        """
        self.missed += 1
        self.hits = 0  # reset hits if we miss a detection
        if self.state == 'TENTATIVE' and self.missed >= tentative_missed_threshold:
            self.state = 'DELETED'
        elif self.missed >= missed_threshold:
            self.state = 'DELETED'

    def get_position(self):
        """
        Get the current position of the person

        Returns:
            position: (x, y)
        """
        x = self.kf.x[0, 0]
        y = self.kf.x[1, 0]
        return (x, y)
