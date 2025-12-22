"""
A* Search Algorithm

A* combines benefits of Dijkstra and Greedy search using f(n) = g(n) + h(n).
Guarantees shortest path with admissible heuristic.
"""

import heapq


class AStarSolver:
    """
    A* Search algorithm with Manhattan distance heuristic.
    
    A* combines the benefits of Dijkstra (optimal path) and Greedy (goal-directed).
    Uses f(n) = g(n) + h(n) where:
    - g(n) = actual cost from start to n
    - h(n) = estimated cost from n to goal (heuristic)
    - f(n) = estimated total cost through n
    
    Guarantees shortest path if heuristic is admissible (never overestimates).
    """
    
    def __init__(self, maze_grid):
        """
        Initialize A* solver with maze grid.
        
        Args:
            maze_grid: 2D array where 0 = path, 1 = wall
        """
        self.grid = maze_grid
        self.height = len(maze_grid)
        self.width = len(maze_grid[0])
        
        # Track visited cells
        self.visited = set()
    
    def heuristic(self, pos, goal):
        """
        Calculate Manhattan distance heuristic.
        
        Manhattan distance = |x1 - x2| + |y1 - y2|
        This is admissible for grid movement (only up/down/left/right).
        
        Args:
            pos: Current position (x, y)
            goal: Goal position (x, y)
            
        Returns:
            int: Manhattan distance between pos and goal
        """
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    def solve(self, start, end):
        """
        Solve maze using A* algorithm.
        
        Yields:
            tuple: (batch_nodes, visited_set)
                - batch_nodes: Nodes with same f-score (for cloning effect)
                - visited_set: Set of visited positions
        
        Algorithm flow:
        1. Initialize open list with start (priority = h(start))
        2. Pop node(s) with lowest f-score
        3. If goal, stop. Otherwise explore neighbors
        4. For each neighbor, calculate g and f scores
        5. Add to open list if better path found
        6. Repeat until goal found
        """
        # Priority queue: (f_score, counter, position, parent)
        # Counter ensures stable sorting when f-scores tie
        counter = 0
        
        # Initialize with start position
        # f_score for start = 0 + h(start, end)
        open_list = [(self.heuristic(start, end), counter, start, None)]
        
        # Mark start as visited
        self.visited.add(start)
        
        # Track actual cost (g-score) from start to each position
        g_scores = {start: 0}
        
        # Process until open list is empty
        while open_list:
            # Pop node with lowest f-score
            f_score, _, current, parent = heapq.heappop(open_list)
            
            # Create batch of all nodes with same f-score (for cloning visualization)
            # This shows multiple equally-good paths being explored simultaneously
            batch = [(current, parent)]
            
            # Check if there are other nodes with identical f-score
            while open_list and open_list[0][0] == f_score:
                # Pop additional tied nodes
                _, _, next_node, next_parent = heapq.heappop(open_list)
                batch.append((next_node, next_parent))
            
            # Yield entire batch for simultaneous visualization
            yield batch, self.visited
            
            # Process each node in the batch
            for curr, par in batch:
                # Check if we reached the goal
                if curr == end:
                    return  # Found shortest path!
                
                # Explore all 4 neighbors
                directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
                
                for dx, dy in directions:
                    # Calculate neighbor position
                    nx, ny = curr[0] + dx, curr[1] + dy
                    neighbor = (nx, ny)
                    
                    # Validate neighbor position
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # Check if neighbor is a path (not wall)
                        if self.grid[ny][nx] == 0:
                            # Calculate new g-score (cost from start to neighbor)
                            # g(neighbor) = g(current) + 1 (step cost = 1)
                            curr_g = g_scores[curr]
                            new_g = curr_g + 1
                            
                            # Only update if this is a better path to neighbor
                            if neighbor not in g_scores or new_g < g_scores[neighbor]:
                                # Update g-score for this neighbor
                                g_scores[neighbor] = new_g
                                
                                # Calculate f-score: f = g + h
                                f = new_g + self.heuristic(neighbor, end)
                                
                                # Increment counter for stable sorting
                                counter += 1
                                
                                # Add to priority queue with parent tracking
                                heapq.heappush(open_list, (f, counter, neighbor, curr))
                                
                                # Mark as visited
                                self.visited.add(neighbor)
