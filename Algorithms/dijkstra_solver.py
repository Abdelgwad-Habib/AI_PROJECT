"""
Dijkstra's Algorithm

Classic shortest path algorithm that explores nodes in order of distance from start.
Guarantees shortest path in weighted graphs.
"""

import heapq


class DijkstraSolver:
    """
    Dijkstra's algorithm for shortest path finding.
    
    Uniform cost search that explores nodes in order of their distance from start.
    Guarantees shortest path in weighted graphs (all weights = 1 for grid mazes).
    Similar to A* but without heuristic (explores more evenly in all directions).
    """
    
    def __init__(self, maze_grid):
        """
        Initialize Dijkstra solver with maze grid.
        
        Args:
            maze_grid: 2D array where 0 = path, 1 = wall
        """
        self.grid = maze_grid
        self.height = len(maze_grid)
        self.width = len(maze_grid[0])
        
        # Track visited positions
        self.visited = set()
    
    def solve(self, start, end):
        """
        Solve maze using Dijkstra's algorithm.
        
        Yields:
            tuple: (batch_nodes, visited_set)
                - batch_nodes: Nodes with same distance (for cloning)
                - visited_set: Set of visited positions
        
        Algorithm flow:
        1. Initialize with start (distance = 0)
        2. Pop node(s) with smallest distance
        3. If goal, stop. Otherwise explore neighbors
        4. Update neighbor distances if shorter path found
        5. Repeat until goal found
        """
        # Priority queue: (distance, counter, position, parent)
        counter = 0
        
        # Start with distance 0
        open_list = [(0, counter, start, None)]
        
        # Mark start as visited
        self.visited.add(start)
        
        # Track shortest distance to each position
        distances = {start: 0}
        
        # Process until queue is empty
        while open_list:
            # Pop node with smallest distance
            dist, _, current, parent = heapq.heappop(open_list)
            
            # Create batch of all nodes at same distance (for cloning effect)
            batch = [(current, parent)]
            
            # Pop all other nodes with same distance
            while open_list and open_list[0][0] == dist:
                _, _, next_node, next_parent = heapq.heappop(open_list)
                batch.append((next_node, next_parent))
            
            # Yield batch for visualization
            yield batch, self.visited
            
            # Process each node in batch
            for curr, par in batch:
                # Check if goal reached
                if curr == end:
                    return  # Shortest path found!
                
                # Explore all 4 directions
                directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
                
                for dx, dy in directions:
                    # Calculate neighbor position
                    nx, ny = curr[0] + dx, curr[1] + dy
                    neighbor = (nx, ny)
                    
                    # Validate position
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # Check if it's a path
                        if self.grid[ny][nx] == 0:
                            # Calculate new distance (current distance + 1)
                            new_dist = distances[curr] + 1
                            
                            # Update if this is a shorter path
                            if neighbor not in distances or new_dist < distances[neighbor]:
                                # Update shortest distance to neighbor
                                distances[neighbor] = new_dist
                                
                                # Increment counter for stable sorting
                                counter += 1
                                
                                # Add to priority queue
                                heapq.heappush(open_list, (new_dist, counter, neighbor, curr))
                                
                                # Mark as visited
                                self.visited.add(neighbor)
