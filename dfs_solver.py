"""
Depth-First Search (DFS) Algorithm

DFS explores as far as possible along each branch before backtracking.
Includes explicit backtracking visualization for dead ends.
"""


class DFSSolver:
    """
    Depth-First Search pathfinding with explicit backtracking visualization.
    
    DFS explores as far as possible along each branch before backtracking.
    This implementation yields both forward steps and backward steps (backtracking)
    so the visualization shows Tom physically walking back when hitting dead ends.
    """
    
    def __init__(self, maze_grid):
        """
        Initialize DFS solver with maze grid.
        
        Args:
            maze_grid: 2D array where 0 = path, 1 = wall
        """
        self.grid = maze_grid
        self.height = len(maze_grid)
        self.width = len(maze_grid[0])
        
        # Track visited cells to avoid cycles
        self.visited = set()

    def solve(self, start, end):
        """
        Solve maze using Depth-First Search with backtracking.
        
        Yields:
            tuple: (batch_nodes, visited_set)
                - batch_nodes: List of (node, parent) tuples for current step
                - visited_set: Set of all visited positions
        
        Algorithm flow:
        1. Push start onto stack
        2. Explore deepest unvisited neighbor
        3. If dead end, backtrack (pop stack, yield backward step)
        4. Repeat until goal found
        """
        # Stack maintains the current path from start to current position
        stack = [start]
        
        # Mark start as visited
        self.visited.add(start)
        
        # Yield initial position (parent=None means this is the starting point)
        yield [(start, None)], self.visited
        
        # Continue until stack is empty (all paths explored) or goal found
        while stack:
            # Peek at current position (don't pop yet - we might explore further)
            current = stack[-1]
            
            # Check if we've reached the goal
            if current == end:
                return  # Stop iteration - goal found!
            
            # Try all 4 directions: down, right, up, left
            directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            found_neighbor = False  # Track if we found an unvisited neighbor
            
            # Try each direction
            for dx, dy in directions:
                # Calculate neighbor position
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                
                # Check if neighbor is valid
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    # Check if it's a path (not wall) and unvisited
                    if self.grid[ny][nx] == 0 and neighbor not in self.visited:
                        # Mark as visited immediately to prevent revisiting
                        self.visited.add(neighbor)
                        
                        # Add to stack (we'll continue from here)
                        stack.append(neighbor)
                        
                        # Mark that we found a valid move
                        found_neighbor = True
                        
                        # Yield forward step: Moving from current to neighbor
                        # Format: [(destination, source)]
                        yield [(neighbor, current)], self.visited
                        
                        # Break to explore this direction fully (depth-first)
                        break
            
            # If no unvisited neighbors found, we hit a dead end
            if not found_neighbor:
                # Backtrack: Remove current position from path
                popped = stack.pop()
                
                # If stack still has positions, yield backward movement
                if stack:
                    back_to = stack[-1]  # The position we're returning to
                    
                    # Yield backward step: Moving from dead end back to last intersection
                    # This creates the backtracking animation
                    yield [(back_to, popped)], self.visited
