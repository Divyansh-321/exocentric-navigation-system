import cv2
import numpy as np

class PositionSmoother:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        # Measurement matrix: [x, y]
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        # Transition matrix: constant velocity model
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0], 
            [0, 1, 0, 1], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1]
        ], np.float32)
        # Noise covariance
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.initialized = False

    def update_and_predict(self, raw_x, raw_y):
        """Feeds a new raw measurement and returns the smoothed prediction."""
        measurement = np.array([[np.float32(raw_x)], [np.float32(raw_y)]])
        
        if not self.initialized:
            self.kf.statePre = np.array([[measurement[0,0]], [measurement[1,0]], [0], [0]], np.float32)
            self.kf.statePost = np.array([[measurement[0,0]], [measurement[1,0]], [0], [0]], np.float32)
            self.initialized = True
            
        self.kf.correct(measurement)
        predicted = self.kf.predict()
        return int(predicted[0,0]), int(predicted[1,0])

    def reset(self):
        """Call this on system hard reset."""
        self.initialized = False