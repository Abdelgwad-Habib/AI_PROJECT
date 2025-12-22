"""
Breadth-First Search (BFS) Algorithm

BFS explores all neighbors at the current depth before moving deeper.
Guarantees shortest path in unweighted graphs.
"""

from collections import deque


class BFSLayerSolver:
    """
    Breadth-First Search with layer-based exploration.
    
    BFS explores all neighbors at the current depth before moving deeper.
    This guarantees the shortest path in unweighted graphs.
    Processes and yields entire layers at once for "cloning" visualization.
    """
    
    def __init__(self, maze_grid):
        """
        Initialize BFS solver with maze grid.
        
        Args:
            maze_grid: 2D array where 0 = path, 1 = wall
        """
        self.grid = maze_grid
        self.height = len(maze_grid)
        self.width = len(maze_grid[0])
        
        # Track visited cells to avoid revisiting
        self.visited = set()

    def solve(self, start, end):
        """
        Solve maze using Breadth-First Search (layer by layer).
        
        Yields:
            tuple: (layer_nodes, visited_set)
                - layer_nodes: List of (node, parent) for all nodes in current layer
                - visited_set: Set of all visited positions
        
        Algorithm flow:
        1. Start with layer 0 containing only start position
        2. For each node in current layer, find all unvisited neighbors
        3. All neighbors form the next layer
        4. Yield entire layer at once (creates simultaneous "clones")
        5. Repeat until goal found
        """
        # Layer 0: Start position with no parent
        current_layer = [(start, None)]
        
        # Mark start as visited
        self.visited.add(start)
        
        # Process layers until no more positions to explore
        while current_layer:
            # Yield entire current layer for simultaneous visualization
            yield current_layer, self.visited
            
            # Build next layer from current layer's neighbors
            next_layer = []
            found_target = False  # Track if we found the goal
            
            # Process each node in current layer
            for node, parent in current_layer:
                # Check if this node is the goal
                if node == end:
                    found_target = True
                    # Don't break - let all nodes in layer finish processing
                
                # Explore all 4 directions
                directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
                
                for dx, dy in directions:
                    # Calculate neighbor position
                    nx, ny = node[0] + dx, node[1] + dy
                    
                    # Validate neighbor
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # Check if it's a path and unvisited
                        if self.grid[ny][nx] == 0 and (nx, ny) not in self.visited:
                            # Mark as visited
                            self.visited.add((nx, ny))
                            
                            # Add to next layer with current node as parent
                            next_layer.append(((nx, ny), node))
            
            # If we found the goal in this layer
            if found_target:
                # Yield any remaining nodes in next layer
                if next_layer:
                    yield next_layer, self.visited
                # Stop searching
                return
            
            # Move to next layer for next iteration
            current_layer = next_layer
