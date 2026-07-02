import heapq

def a_star_search(start, goal, hard_obstacles, soft_obstacles, grid_size, prev_path=None):
    if start == goal: 
        return [start]
        
    def heuristic(a, b): 
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    
    obs_set = set(hard_obstacles)
    obs_set.discard(start) 
    
    danger_zones = set(soft_obstacles)
    prev_set = set(prev_path) if prev_path else set()
    
    # Boundary danger zones
    for i in range(grid_size):
        danger_zones.add((i, 0))
        danger_zones.add((i, grid_size - 1))
        danger_zones.add((0, i))
        danger_zones.add((grid_size - 1, i))
    
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal: 
            break
            
        cx, cy = current
        for nxt in [(cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)]:
            if 0 <= nxt[0] < grid_size and 0 <= nxt[1] < grid_size:
                if nxt in obs_set: 
                    continue 
                
                base_cost = 5 if nxt in danger_zones else 1
                if prev_set and nxt not in prev_set:
                    base_cost += 2
                    
                new_cost = cost_so_far[current] + base_cost
                
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + heuristic(goal, nxt)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
                    
    if goal not in came_from: 
        return [] 
        
    current = goal
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path