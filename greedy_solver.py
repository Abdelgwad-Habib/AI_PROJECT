"""
Greedy Best-First Search Algorithm

Always explores the node that appears closest to goal by heuristic.
Fast but does NOT guarantee shortest path.
"""

import heapq


class GreedySolver:
    """
    Greedy Best-First Search using Manhattan distance heuristic.
    
    Always explores the node that appears closest to the goal (by heuristic).
    Fast and goal-directed but does NOT guarantee shortest path.
    Can get stuck in local minima or take longer routes.
    
    Good for: Quick solutions when optimality isn't critical
    Bad for: When you need guaranteed shortest path
    """
    
    def __init__(self, maze_grid):
        """
        Initialize Greedy solver with maze grid.
        
        Args:
            maze_grid: 2D array where 0 = path, 1 = wall
        """
        self.grid = maze_grid
        self.height = len(maze_grid)
        self.width = len(maze_grid[0])
        
        # Track visited positions to avoid cycles
        self.visited = set()
    
    def heuristic(self, pos, goal):
        """
        Calculate Manhattan distance to goal.
        
        Args:
            pos: Current position (x, y)
            goal: Goal position (x, y)
            
        Returns:
            int: Manhattan distance
        """
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    def solve(self, start, end):
        """
        Solve maze using Greedy Best-First Search.
        
        Yields:
            tuple: (batch_nodes, visited_set)
                - batch_nodes: Nodes with same heuristic (for cloning)
                - visited_set: Set of visited positions
        
        Algorithm flow:
        1. Initialize with start (priority = h(start))
        2. Pop node(s) with lowest heuristic value
        3. If goal, stop. Otherwise explore neighbors
        4. Add neighbors to queue with their heuristic values
        5. Repeat until goal found
        
        Note: Only uses h(n), not g(n), so not guaranteed optimal!
        """
        # Priority queue: (heuristic, counter, position, parent)
        counter = 0
        
        # Initialize with start position
        open_list = [(self.heuristic(start, end), counter, start, None)]
        
        # Mark start as visited
        self.visited.add(start)
        
        # Process until queue empty
        while open_list:
            # Pop node with lowest heuristic value (appears closest to goal)
            h, _, current, parent = heapq.heappop(open_list)
            
            # Create batch of all nodes with same heuristic (for cloning)
            batch = [(current, parent)]
            
            # Pop other nodes with identical heuristic
            while open_list and open_list[0][0] == h:
                _, _, next_node, next_parent = heapq.heappop(open_list)
                batch.append((next_node, next_parent))
            
            # Yield batch for visualization
            yield batch, self.visited
            
            # Process each node in batch
            for curr, par in batch:
                # Check if goal reached
                if curr == end:
                    return  # Path found (may not be shortest!)
                
                # Explore all 4 neighbors
                directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
                
                for dx, dy in directions:
                    # Calculate neighbor position
                    nx, ny = curr[0] + dx, curr[1] + dy
                    neighbor = (nx, ny)
                    
                    # Validate position
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # Check if it's a path and unvisited
                        if self.grid[ny][nx] == 0 and neighbor not in self.visited:
                            # Mark as visited
                            self.visited.add(neighbor)
                            
                            # Increment counter
                            counter += 1
                            
                            # Add to queue with heuristic as priority
                            # Note: No g-score, only heuristic!
                            heapq.heappush(open_list, 
                                         (self.heuristic(neighbor, end), counter, neighbor, curr))
