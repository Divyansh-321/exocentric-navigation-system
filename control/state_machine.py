import time
import random
from config.settings import SEQUENCE_LENGTH, POSSIBLE_COMMANDS

class SystemState:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = "WARMUP"
        self.cmd_index = 0
        self.candidates = {}
        self.next_candidate_id = 0
        self.is_moving_phase = True
        self.current_robot_heading = 0.0
        self.current_nav_cmd = "WAIT"
        
        self.target_grid_x = -1
        self.target_grid_y = -1  
        self.exact_target_x = -1
        self.exact_target_y = -1
        
        self.kf_initialized = False
        self.smooth_cx = None
        self.smooth_cy = None
        self.smooth_px = None
        self.smooth_py = None
        self.last_grid_path = []
        
        self.state_timer = time.time()
        self.wiggle_cmd_timer = time.time()
        self.nav_timer = time.time()
        
        self.wiggle_sequence = [random.choice(POSSIBLE_COMMANDS) for _ in range(SEQUENCE_LENGTH)]
        print(">>> SYSTEM HARD RESET")