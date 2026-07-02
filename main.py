import cv2
import math
import time
import numpy as np
from ultralytics import YOLO

import config.settings as cfg
from comm.udp import UDPSender
from comm.protocol import RobotProtocol
from vision.homography import compute_homography_matrix, apply_perspective_warp
from vision.detector import detect_heading_marker
from tracking.kalman import PositionSmoother
from tracking.candidate_tracker import CandidateManager
from planning.costmap import CostmapGenerator
from planning.astar import a_star_search
from control.controller import KinematicController
from control.state_machine import SystemState
import utils.visualization as vis

clicked_points = []
CORNER_LABELS = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]

def mouse_click_handler(event, x, y, flags, param):
    global clicked_points
    sys_state = param[0]
    frame_w, frame_h = param[1], param[2]
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Arena calibration clicks
        if sys_state.state == "WARMUP" and len(clicked_points) < 4:
            clicked_points.append([x, y])
            print(f"[INFO] Recorded {CORNER_LABELS[len(clicked_points)-1]}: ({x}, {y})")
            
        # Target assignment during runtime
        elif sys_state.state in ["AWAITING_TARGET", "NAVIGATION"]:
            sys_state.exact_target_x, sys_state.exact_target_y = x, y
            sys_state.target_grid_x = int((x / frame_w) * cfg.GRID_SIZE)
            sys_state.target_grid_y = int((y / frame_h) * cfg.GRID_SIZE)
            sys_state.state = "NAVIGATION"
            sys_state.current_nav_cmd = "DRIVING"
            print(f"[INFO] Target locked to grid cell: ({sys_state.target_grid_x}, {sys_state.target_grid_y})")

def main():
    global clicked_points
    
    # Using DirectShow backend to prevent Windows webcam access freezes
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    model = YOLO("models/best_obstacle.pt")  
    udp_client = UDPSender(cfg.ESP8266_IP, cfg.ESP8266_PORT)
    
    sys_state = SystemState()
    smoother = PositionSmoother()
    mapper = CostmapGenerator()
    controller = KinematicController(lookahead_distance=80)
    
    motion_detector = cv2.createBackgroundSubtractorMOG2(history=30, varThreshold=50, detectShadows=False)
    tracker = cv2.TrackerCSRT_create()
    
    homography_matrix = None
    cv2.namedWindow("Nav System")
    
    # Lead Eng Fix: Capture a startup frame to establish dimension constants safely before the loop
    ret, init_frame = cap.read()
    if not ret:
        print("[ERROR] Initialization failed. Unable to read from video source.")
        return
    h, w, _ = init_frame.shape
    
    # Lead Eng Fix: Register mouse listener exactly ONCE here to protect processing resources
    cv2.setMouseCallback("Nav System", mouse_click_handler, param=[sys_state, w, h])
    
    print("[INFO] System online. Click the 4 corners of your arena setup.")

    while True:
        ret, raw_frame = cap.read()
        if not ret:
            print("[ERROR] Could not read frame from camera.")
            break

        # Calibration phase
        if sys_state.state == "WARMUP":
            # Lead Eng Fix: Removed raw_frame contamination from motion detector to protect calibration history
            
            # Still collecting the 4 corners
            if len(clicked_points) < 4:
                for idx, pt in enumerate(clicked_points):
                    cv2.circle(raw_frame, (pt[0], pt[1]), 5, (0, 0, 255), -1)
                    cv2.putText(raw_frame, CORNER_LABELS[idx], (pt[0] + 10, pt[1] - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                next_prompt = CORNER_LABELS[len(clicked_points)]
                cv2.putText(raw_frame, f"Click corner: {next_prompt}", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("Nav System", raw_frame)
            
            # 4 points clicked -> compute homography and display warped view instantly
            else:
                if homography_matrix is None:
                    homography_matrix = compute_homography_matrix(clicked_points, w, h)
                    print("[INFO] Homography matrix auto-computed successfully.")
                
                # Show the warped perspective view immediately so they know it is active
                warped_preview = apply_perspective_warp(raw_frame, homography_matrix, w, h)
                cv2.putText(warped_preview, "HOMOGRAPHY ACTIVE READY; press spacebar", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Nav System", warped_preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                if homography_matrix is not None:
                    sys_state.state = "WIGGLING"
                    sys_state.state_timer = time.time()
                    sys_state.wiggle_cmd_timer = time.time()
                    print("[INFO] Starting pulse-based wiggle calibration.")
            elif key == 27 or key == ord('q'):
                break
            continue

        # Dynamic overhead tracking loop
        frame = apply_perspective_warp(raw_frame, homography_matrix, w, h)
        
        # Pulse-based wiggle characterization phase
        if sys_state.state == "WIGGLING":
            current_time = time.time()
            elapsed = current_time - sys_state.state_timer

            move_dur = getattr(cfg, 'MOVE_DURATION', 0.3)
            stop_dur = getattr(cfg, 'STOP_DURATION', 0.5)
            active_cmd = sys_state.wiggle_sequence[sys_state.cmd_index]

            # Alternate between running and pause phases
            if sys_state.is_moving_phase and elapsed > move_dur:
                sys_state.is_moving_phase = False
                sys_state.state_timer = current_time
                udp_client.send(RobotProtocol.serialize('X'))
                
            elif not sys_state.is_moving_phase and elapsed > stop_dur:
                sys_state.is_moving_phase = True
                sys_state.state_timer = current_time
                sys_state.cmd_index += 1
                if sys_state.cmd_index >= len(sys_state.wiggle_sequence):
                    sys_state.state = "LOCKING"
                    continue

            # Send drive pulses
            if sys_state.is_moving_phase:
                if current_time - sys_state.wiggle_cmd_timer > 0.1:
                    udp_client.send(RobotProtocol.serialize(active_cmd))
                    sys_state.wiggle_cmd_timer = current_time
                    
            # Background subtraction to map motion vectors safely on warped frame only
            mask = motion_detector.apply(frame)
            _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35, 35))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            score_modifier = 2 if sys_state.is_moving_phase else (-2 if elapsed > 0.5 else 0)
            
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    bx, by, bw, bh = cv2.boundingRect(cnt)
                    cx, cy = bx + bw // 2, by + bh // 2
                    matched = False
                    
                    for cid, cdata in list(sys_state.candidates.items()):
                        if math.hypot(cx - cdata['center'][0], cy - cdata['center'][1]) < 75:
                            sys_state.candidates[cid].update({'center': (cx, cy), 'box': (bx, by, bw, bh)})
                            sys_state.candidates[cid]['score'] += score_modifier
                            matched = True
                            break
                    if not matched:
                        sys_state.candidates[sys_state.next_candidate_id] = {
                            'box': (bx, by, bw, bh), 'center': (cx, cy), 'score': 0
                        }
                        sys_state.next_candidate_id += 1
            
            # Visual text timers and live command tracking overlays
            if sys_state.is_moving_phase:
                timer_text = f"MOVING: {elapsed:.2f}s / {move_dur:.2f}s"
                cmd_text = f"ACTIVE MOTOR CMD: [{active_cmd}]"
                color = (0, 255, 0)
            else:
                timer_text = f"PAUSED: {elapsed:.2f}s / {stop_dur:.2f}s"
                cmd_text = "ACTIVE MOTOR CMD: [X] (BRAKE)"
                color = (0, 165, 255)

            cv2.putText(frame, timer_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(frame, cmd_text, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            cv2.putText(frame, f"STEP: {sys_state.cmd_index + 1} / {len(sys_state.wiggle_sequence)}", 
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            for cid, cdata in sys_state.candidates.items():
                x, y, w_box, h_box = cdata['box']
                cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (0, 165, 255), 2)

        # Confirm high-confidence object target block
        elif sys_state.state == "LOCKING":
            if sys_state.candidates:
                best_id = max(sys_state.candidates, key=lambda k: sys_state.candidates[k]['score'])
                if sys_state.candidates[best_id]['score'] > 10:
                    bx, by, bw, bh = sys_state.candidates[best_id]['box']
                    pad = 40
                    bx, by = max(0, bx - pad), max(0, by - pad)
                    bw = min(w - bx, bw + 2 * pad)
                    bh = min(h - by, bh + 2 * pad)
                    
                    tracker = cv2.TrackerCSRT_create()
                    tracker.init(frame, (bx, by, bw, bh))
                    sys_state.state = "AWAITING_TARGET"
                    print(f"[INFO] Tracking hook locked with score: {sys_state.candidates[best_id]['score']}.")
                else:
                    sys_state.reset()
                    clicked_points = []
            else:
                sys_state.reset()
                clicked_points = []

        # Closed-loop tracking and path generation
        elif sys_state.state in ["AWAITING_TARGET", "NAVIGATION"]:
            success, tracking_box = tracker.update(frame)
            
            if not success:
                print("[WARNING] Tracking signature dropped. Resetting.")
                udp_client.send(RobotProtocol.serialize('X'))
                sys_state.reset()
                clicked_points = []
                homography_matrix = None
                continue

            rx, ry, rw, rh = [int(v) for v in tracking_box]
            raw_cx, raw_cy = rx + (rw // 2), ry + (rh // 2)
            
            smooth_cx, smooth_cy = smoother.update_and_predict(raw_cx, raw_cy)
            
            sys_state.smooth_px, sys_state.smooth_py = detect_heading_marker(
                frame, rx, ry, rw, rh, sys_state.smooth_px, sys_state.smooth_py
            )

            hard_obs, soft_obs, target_blocked = mapper.build_costmap(
                frame, raw_frame, model, homography_matrix, sys_state.target_grid_x, sys_state.target_grid_y
            )

            if target_blocked:
                print("[ABORT] Target coordinates are inside an obstacle zone.")
                sys_state.state = "AWAITING_TARGET"
                sys_state.target_grid_x, sys_state.target_grid_y = -1, -1
                udp_client.send(RobotProtocol.serialize('X'))
                continue

            heading_angle = 0.0
            if sys_state.smooth_px is not None:
                heading_angle = math.degrees(math.atan2(sys_state.smooth_py - smooth_cy, sys_state.smooth_px - smooth_cx))

            robot_gx = int((smooth_cx / w) * cfg.GRID_SIZE)
            robot_gy = int((smooth_cy / h) * cfg.GRID_SIZE)

            if sys_state.state == "AWAITING_TARGET":
                udp_client.send(RobotProtocol.serialize('X'))
                if sys_state.current_nav_cmd == "ARRIVED":
                    cv2.putText(frame, "TARGET REACHED!", (w // 2 - 200, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
                else:
                    vis.draw_hud(frame, sys_state.state, 0, 0, "IDLE", "WAIT")

            elif sys_state.state == "NAVIGATION":
                dist = math.hypot(sys_state.exact_target_x - smooth_cx, sys_state.exact_target_y - smooth_cy)
                if dist < 40:
                    print("[INFO] Arrived at destination.")
                    udp_client.send(RobotProtocol.serialize('X'))
                    sys_state.state = "AWAITING_TARGET"
                    sys_state.current_nav_cmd = "ARRIVED"
                    continue

                path_grid = a_star_search(
                    (robot_gx, robot_gy), 
                    (sys_state.target_grid_x, sys_state.target_grid_y), 
                    hard_obs, 
                    soft_obs,
                    cfg.GRID_SIZE,
                    prev_path=sys_state.last_grid_path
                )

                # Emergency routing bypass if path closes up
                if not path_grid and sys_state.target_grid_x != -1:
                    cv2.putText(frame, "COLLISION ESCAPE ROUTING RUNNING", (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
                    path_grid = a_star_search(
                        (robot_gx, robot_gy), (sys_state.target_grid_x, sys_state.target_grid_y), 
                        [], (soft_obs + hard_obs), cfg.GRID_SIZE, prev_path=sys_state.last_grid_path
                    )

                if path_grid:
                    sys_state.last_grid_path = path_grid
                    
                    path_pixels = [
                        (int((gx + 0.5) * (w / cfg.GRID_SIZE)), int((gy + 0.5) * (h / cfg.GRID_SIZE))) 
                        for gx, gy in path_grid
                    ]
                    
                    target_pt = controller.extract_waypoint(path_pixels, smooth_cx, smooth_cy)
                    
                    # Command frequency execution controller (0.4s intervals)
                    if time.time() - sys_state.nav_timer > 0.4:
                        action = controller.get_steering_action(target_pt, smooth_cx, smooth_cy, heading_angle)
                        sys_state.current_nav_cmd = action
                        udp_client.send(RobotProtocol.serialize(action))
                        sys_state.nav_timer = time.time()

                    vis.draw_path(frame, path_pixels, target_pt)
                else:
                    vis.draw_path(frame, [])
                    udp_client.send(RobotProtocol.serialize('X'))

            # Lead Eng Fix: Moved visualization methods inside the explicit tracking scope to guarantee variable execution safety
            vis.draw_robot(frame, rx, ry, rw, rh, smooth_cx, smooth_cy, sys_state.smooth_px, sys_state.smooth_py)
            vis.draw_hud(frame, sys_state.state, 0, 0, "RUNNING", sys_state.current_nav_cmd)

        # UI target goal marker overlay
        if sys_state.target_grid_x >= 0 and sys_state.target_grid_y >= 0:
            cv2.circle(frame, (sys_state.exact_target_x, sys_state.exact_target_y), 4, (0, 255, 255), -1)
            cv2.putText(frame, f"GOAL ({sys_state.target_grid_x},{sys_state.target_grid_y})", 
                        (sys_state.exact_target_x - 25, sys_state.exact_target_y - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow("Nav System", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):  
            udp_client.send(RobotProtocol.serialize('X'))
            sys_state.reset()
            clicked_points = []
            homography_matrix = None
        elif key == ord('c'):  
            udp_client.send(RobotProtocol.serialize('X'))
            sys_state.reset()
            sys_state.state = "WARMUP"
            clicked_points = []
            homography_matrix = None
        elif key == 27 or key == ord('q'):  
            break

    udp_client.send(RobotProtocol.serialize('X'))
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()