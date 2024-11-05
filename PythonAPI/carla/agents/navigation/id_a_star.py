import math

class Node:
    def __init__(self, position, g=0, h=0):
        self.position = position  # Position is a tuple (x, y)
        self.g = g  # Cost from the start node (g(n))
        self.h = h  # Heuristic estimate to the goal (h(n))

    @property
    def f(self):
        # Total cost function f(n) = g(n) + h(n)
        return self.g + self.h

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

def heuristic(current, goal):
    #(Manhattan Distance)
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def get_neighbors(node, grid):
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    for direction in directions:
        new_pos = (node.position[0] + direction[0], node.position[1] + direction[1])
        # Check if the new position is within grid bounds and is walkable (i.e., not an obstacle)
        if 0 <= new_pos[0] < len(grid) and 0 <= new_pos[1] < len(grid[0]) and grid[new_pos[0]][new_pos[1]] == 0:
            neighbors.append(Node(new_pos, node.g + 1))  # g = current g + 1 (cost of 1 for each move)
    return neighbors

def search(current_node, goal_node, threshold, grid):
    f = current_node.f
    if f > threshold:
        return f, None  # Return the new threshold and no solution

    if current_node == goal_node:
        return f, [current_node.position]  # Return the solution path

    min_threshold = math.inf
    for neighbor in get_neighbors(current_node, grid):
        neighbor.h = heuristic(neighbor.position, goal_node.position)  # Update heuristic
        temp_threshold, result = search(neighbor, goal_node, threshold, grid)
        if result:  # If a valid path is found
            return temp_threshold, [current_node.position] + result
        min_threshold = min(min_threshold, temp_threshold)

    return min_threshold, None

def ida_star(start, goal, grid):
    start_node = Node(start)
    goal_node = Node(goal)
    start_node.h = heuristic(start_node.position, goal_node.position)

    threshold = start_node.f  # Initial threshold is based on f(start) = h(start)
    while True:
        threshold, path = search(start_node, goal_node, threshold, grid)
        if path:
            return path  # Path found
        if threshold == math.inf:
            return None
