import cv2
import numpy as np
from config.settings import GRID_SIZE, OBSTACLE_CLASS_ID
from utils.geometry import get_grid_coordinate

class CostmapGenerator:
    def __init__(self):
        pass

    def build_costmap(self, frame, raw_frame, model, homography, target_x, target_y):
        """Analyzes deep learning bounding boxes and frame pixel array thresholds to extract obstacles."""
        h, w, _ = frame.shape
        cell_w, cell_h = w // GRID_SIZE, h // GRID_SIZE
        
        hard_obstacles = []
        soft_obstacles = []
        target_is_blocked = False

        # 1. Process Dynamic Objects via YOLO Model Tracking
        results = model(raw_frame, conf=0.75, stream=True)
        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) == OBSTACLE_CLASS_ID:
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                    
                    # Warp floor plane intercept point only
                    pts = np.array([[[x1, y2], [x2, y2]]], dtype=np.float32)
                    warped_pts = cv2.perspectiveTransform(pts, homography)
                    wx1, wy1 = warped_pts[0][0]
                    wx2, wy2 = warped_pts[0][1]
                    
                    warped_cx = int((wx1 + wx2) / 2)
                    warped_cy = int((wy1 + wy2) / 2)
                    actual_width = abs(wx2 - wx1)
                    
                    pad_w = int((actual_width / 2) + 28)
                    pad_h_front = 35
                    pad_h_back = 95
                    
                    min_gx = get_grid_coordinate(warped_cx - pad_w, w, GRID_SIZE)
                    max_gx = get_grid_coordinate(warped_cx + pad_w, w, GRID_SIZE)
                    min_gy = get_grid_coordinate(warped_cy - pad_h_back, h, GRID_SIZE)
                    max_gy = get_grid_coordinate(warped_cy + pad_h_front, h, GRID_SIZE)
                    
                    for gx in range(min_gx, max_gx + 1):
                        for gy in range(min_gy, max_gy + 1):
                            hard_obstacles.append((gx, gy))
                            for dx in [-1, 0, 1]:
                                for dy in [-1, 0, 1]:
                                    if dx != 0 or dy != 0:
                                        soft_obstacles.append((gx + dx, gy + dy))
                                        
                    cv2.rectangle(frame, (int(warped_cx - pad_w), int(warped_cy - pad_h_back)), 
                                  (int(warped_cx + pad_w), int(warped_cy + pad_h_front)), (0, 0, 255), 2)
                    
                    if min_gx <= target_x <= max_gx and min_gy <= target_y <= max_gy:
                        target_is_blocked = True

        # 2. Process Static Dark Regions
        for gy in range(GRID_SIZE):
            for gx in range(GRID_SIZE):
                check_x = int((gx + 0.5) * cell_w)
                check_y = int((gy + 0.5) * cell_h)
                if np.sum(frame[check_y, check_x]) < 20:
                    hard_obstacles.append((gx, gy))
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            soft_obstacles.append((gx + dx, gy + dy))

        return hard_obstacles, soft_obstacles, target_is_blocked