import math
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from a_star import astar, heuristic


def build_graph():
        G = nx.DiGraph()
        # Lets create a grid to resemble city with blocks
        for x in range(6):
                G.add_node(x)
                G.add_node(x+6)
                G.add_node(x+12)
                G.add_node(x+18)

        for x in range(36):
                if (x%6 != 5):
                        # print("in the other bay", x) debug help
                        if (x%5):
                                G.add_edge(x, x+1)
                                G.add_edge(x+1, x)
                if (x/6 < 4):
                        # print("in this bay", x) debug help
                        G.add_edge(x, x+6)
                        G.add_edge(x+6, x)

        G.add_edge(29, 30)
 
        return G


def _path_search_new(G, start, end):
        """
        This function finds a path between the origin and destination
        using IDA* search.

        # # adjusted so input is self._localize(origin), and self._localize(destination) # #

        DEPRECATED # origin      :   carla.Location object of start position, 
        DEPRECATED # destination :   carla.Location object of of end position
        return      :   path as list of node ids (as int) of the graph self._graph
        connecting origin and destination
        """
        # start, end = self._localize(origin), self._localize(destination)

        

        # The localize function returns the edge corresponding to the road segment that the car is currently on.

        # So if we run localize on the carla location, it will return something like the following
        # start = [1, 2]
        # end = [18, 19]

        # route = nx.astar_path(G, source=start[0], target=end[0], weight='length')
        route = astar(G, start=start[0], goal=end[0])
        route.append(end[1])
        return route


def main():
        G = build_graph()
        start = [0, 1]
        end = [23, 29]
        route = _path_search_new(G, start, end)
        for street in route:
                print(street)
                
        nx.draw(G, with_labels=True)
        plt.show()

# if __name__ == "main":
#         main()
main()