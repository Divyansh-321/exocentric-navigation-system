import math

class KinematicController:
    def __init__(self, lookahead_distance=80):
        self.lookahead = lookahead_distance

    def extract_waypoint(self, path_points, robot_x, robot_y):
        """Finds target lookahead points across the A* tracking grid arrays."""
        target_pt = path_points[-1]
        for pt in path_points:
            if math.hypot(pt[0] - robot_x, pt[1] - robot_y) > self.lookahead:
                target_pt = pt
                break
        return target_pt

    def get_steering_action(self, target_pt, robot_x, robot_y, heading_angle):
        """Computes shortest angular heading errors and outputs direct motor control tags."""
        waypoint_x, waypoint_y = target_pt
        desired_angle = math.degrees(math.atan2(waypoint_y - robot_y, waypoint_x - robot_x))
        diff = (desired_angle - heading_angle + 180) % 360 - 180
        
        if diff > 30: 
            return 'D'
        elif diff < -30: 
            return 'A'
        return 'W'