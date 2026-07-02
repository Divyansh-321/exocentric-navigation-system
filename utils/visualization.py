import cv2

def draw_hud(frame, state, cmd_index, seq_length, phase_text, current_cmd):
    """Draws the top-left status text based on the current system state."""
    if state == "WARMUP":
        cv2.putText(frame, "HOMOGRAPHY ACTIVE | READY: PRESS SPACE", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    elif state == "WIGGLING":
        cv2.putText(frame, f"SEARCHING: {cmd_index}/{seq_length} ({phase_text})", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    elif state == "NAVIGATION":
        cv2.putText(frame, "WEIGHTED A-STAR DRIVING", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"CMD: {current_cmd}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

def draw_robot(frame, rx, ry, rw, rh, smooth_cx, smooth_cy, heading_px=None, heading_py=None):
    """Draws the bounding box, center point, and heading line for the robot."""
    cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 3)
    cv2.putText(frame, "ROBOT", (rx, max(20, ry - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.circle(frame, (smooth_cx, smooth_cy), 5, (0, 255, 0), -1)
    
    if heading_px is not None and heading_py is not None:
        cv2.line(frame, (smooth_cx, smooth_cy), (heading_px, heading_py), (255, 255, 0), 3)

def draw_path(frame, path_points, target_pt=None):
    """Draws the green A* path lines and the orange lookahead waypoint."""
    if not path_points:
        cv2.putText(frame, "PATH BLOCKED!", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        return
        
    for i in range(1, len(path_points)):
        cv2.line(frame, path_points[i-1], path_points[i], (0, 255, 0), 3)
        
    if target_pt:
        cv2.circle(frame, target_pt, 8, (0, 165, 255), -1)