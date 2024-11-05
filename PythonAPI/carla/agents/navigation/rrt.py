import random
import math

class Node:
    def __init__(self, position, parent=None):
        self.position = position  # (x, y) or (x, y, z) in 2D/3D space
        self.parent = parent      # Pointer to parent node

def distance(point1, point2):
    """ Euclidean distance between two points """
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def steer(from_node, to_point, max_step):
    """ Move from `from_node` towards `to_point` by a distance of `max_step` """
    vector = (to_point[0] - from_node.position[0], to_point[1] - from_node.position[1])
    length = distance(from_node.position, to_point)
    scale = min(max_step / length, 1.0)  # Ensure the step doesn't exceed max_step
    new_position = (from_node.position[0] + vector[0] * scale,
                    from_node.position[1] + vector[1] * scale)
    return new_position

def get_random_point(space_bounds):
    """ Generate a random point within the defined space bounds (2D or 3D) """
    return (random.uniform(space_bounds[0][0], space_bounds[0][1]),
            random.uniform(space_bounds[1][0], space_bounds[1][1]))
    
def nearest_neighbor(tree, random_point):
    """ Find the nearest node in the tree to the random point """
    nearest = tree[0]
    min_dist = distance(nearest.position, random_point)
    for node in tree:
        dist = distance(node.position, random_point)
        if dist < min_dist:
            nearest = node
            min_dist = dist
    return nearest

def rrt(start, goal, space_bounds, max_iter=1000, max_step=1.0, goal_threshold=0.1, goal_bias=0.1):
    """
    Rapidly-Exploring Random Tree (RRT) Algorithm
    - start: start position (tuple)
    - goal: goal position (tuple)
    - space_bounds: [(x_min, x_max), (y_min, y_max)] for 2D space
    - max_iter: maximum number of iterations
    - max_step: maximum step size
    - goal_threshold: how close to the goal is considered success
    - goal_bias: probability of steering towards the goal instead of a random point
    """
    tree = [Node(start)]

    for i in range(max_iter):
        if random.random() < goal_bias:
            random_point = goal  # Bias towards the goal with probability `goal_bias`
        else:
            random_point = get_random_point(space_bounds)

        # Find nearest node in the tree
        nearest_node = nearest_neighbor(tree, random_point)

        # Steer towards the random point
        new_position = steer(nearest_node, random_point, max_step)

        # Check if new position is valid (no collisions)
        if not is_collision_free(nearest_node.position, new_position):  # Define collision check
            continue

        # Create new node and add it to the tree
        new_node = Node(new_position, nearest_node)
        tree.append(new_node)

        # Check if we've reached the goal
        if distance(new_node.position, goal) < goal_threshold:
            return reconstruct_path(new_node)

    return None  # No path found

def reconstruct_path(node):
    """ Reconstruct the path by following the parent nodes back to the start """
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]

def is_collision_free(p1, p2):
    """ Check if the line between p1 and p2 is free of obstacles (dummy for now) """
    return True  # This is a dummy; replace with real collision check logic