import cv2
import numpy as np
from config.settings import PIXELS_PER_CM

def compute_homography_matrix(calibration_points, frame_width, frame_height):
    """Generates the perspective transformation matrix based on 4 bounding points."""
    src_pts = np.array(calibration_points, dtype=np.float32)
    cx, cy = frame_width // 2, int(frame_height * 0.7) 
    hw = int((17.0 * PIXELS_PER_CM) / 2)
    hh = int((27.0 * PIXELS_PER_CM) / 2)
    
    dst_pts = np.array([
        [cx - hw, cy - hh], 
        [cx + hw, cy - hh], 
        [cx + hw, cy + hh], 
        [cx - hw, cy + hh]  
    ], dtype=np.float32)
    
    return cv2.getPerspectiveTransform(src_pts, dst_pts)

def apply_perspective_warp(frame, matrix, width, height):
    """Warps an incoming image frame into the orthogonal overhead view space."""
    return cv2.warpPerspective(frame, matrix, (width, height))