import cv2
import numpy as np
from config.settings import LOWER_PINK, UPPER_PINK

def detect_heading_marker(frame, rx, ry, rw, rh, current_smooth_px, current_smooth_py, alpha=0.3):
    """Locates the pink marker within the robot bounding box and updates filtered coordinates."""
    robot_roi = frame[max(0, ry):ry+rh, max(0, rx):rx+rw]
    if robot_roi.size == 0:
        return current_smooth_px, current_smooth_py

    pink_mask = cv2.inRange(cv2.cvtColor(robot_roi, cv2.COLOR_BGR2HSV), LOWER_PINK, UPPER_PINK)
    cnts, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if cnts:
        M = cv2.moments(max(cnts, key=cv2.contourArea))
        if M["m00"] > 0:
            raw_px = rx + int(M["m10"] / M["m00"])
            raw_py = ry + int(M["m01"] / M["m00"])
            
            if current_smooth_px is None:
                return raw_px, raw_py
            else:
                smooth_px = int(alpha * raw_px + (1 - alpha) * current_smooth_px)
                smooth_py = int(alpha * raw_py + (1 - alpha) * current_smooth_py)
                return smooth_px, smooth_py
                
    return current_smooth_px, current_smooth_py