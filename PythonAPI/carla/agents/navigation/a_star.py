import math
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import heapq

class Node:
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position  # node number
        self.g = g  # Cost from start to this node
        self.h = h  # Heuristic from this node to the goal
        self.f = g + h  # Total cost
        self.parent = parent  # Pointer to the previous node (to reconstruct the path)

    def __lt__(self, other):
        return self.f < other.f  # Priority queue will sort by the total cost f

def astar(graph, start, goal, heuristic, weight):
    # Open list and closed list
    open_list = []
    closed_list = set()

    # Initialize the start node
    start_node = Node(position=start, g=0, h=heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        # Pop the node with the smallest f from the priority queue
        current_node = heapq.heappop(open_list)

        # If we reach the goal, reconstruct the path
        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            print(path)
            return path[::-1]  # Reverse the path

        # Mark current node as visited
        closed_list.add(current_node.position)

        # Explore neighbors
        neighbors = graph.neighbors(current_node.position)  # Function to return valid neighbors
        print(list(neighbors))
        print("Pickel pie pickles")
        for neighbor_pos, cost in neighbors:
            if neighbor_pos in closed_list:
                continue  # Skip already visited nodes instead of skipping, we should check if the weight is less going through the current path

            # Create a new node for this neighbor
            neighbor_g = current_node.g + 1  # Actual cost from start to neighbor
            neighbor_h = heuristic(neighbor_pos, goal)  # Heuristic cost from neighbor to goal
            neighbor_node = Node(position=neighbor_pos, g=neighbor_g, h=neighbor_h, parent=current_node)

            # If the neighbor is already in the open list with a higher cost, skip it
            if any(open_node.position == neighbor_pos and open_node.g <= neighbor_g for open_node in open_list):
                continue

            # Add the neighbor to the open list
            heapq.heappush(open_list, neighbor_node)

    print(path)
    return path

