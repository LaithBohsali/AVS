import math
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
# from global_route_planner import *
from a_star import *

# grp = GlobalRoutePlanner()
class GRP (object):
        def __init__(self, G):
                self._graph = G

        def _distance_heuristic(self, n1, n2):
                """
                Distance heuristic calculator for path searching
                in self._graph
                """
                l1 = np.array(self._graph.nodes[n1]['vertex'])
                l2 = np.array(self._graph.nodes[n2]['vertex'])
                return np.linalg.norm(l1-l2)


def build_graph():  # Bulids a graph, mess withth
        G = nx.DiGraph()
        # Lets create a grid to resemble city with blocks
        for x in range(6):
                G.add_node(x, vertex=(x, 0, 0))
                print(G.nodes[x])
                G.add_node(x + 6, vertex=(x, 0, 1))
                G.add_node(x + 12, vertex=(x, 0, 1))
                G.add_node(x + 18, vertex=(x, 0, 0))

        for x in range(36):
                if (x % 6 != 5):
                        # print("in the other bay", x) debug help
                        # if (x%5):
                        G.add_edge(x, x + 1)
                        G.add_edge(x + 1, x)
                if (x / 6 < 4):
                        # print("in this bay", x) debug help
                        G.add_node(x + 6, vertex=(x, 0, 0))
                        G.add_edge(x, x + 6)
                        G.add_edge(x + 6, x)

        G.add_edge(29, 30)

        return G

def _path_search_new(G, start, end, grp):
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

        route = nx.astar_path(G,
                              source=start[0],
                              target=end[0],
                              weight='length')

        route = astar(G, start=start[0], goal=end[0], heuristic=grp._distance_heuristic, weight='length')
        # route = astar(G, start=start[0], goal=end[0])
        route.append(end[1])
        return route


def main():
        G = build_graph()
        grp = GRP(G)

        start = [0, 1]
        end = [23, 29]
        route = _path_search_new(G, start, end, grp)
        for street in route:
                print(G.nodes[street]['vertex'])
                print(street)

        nx.draw(G, with_labels=True)
        plt.show()


# if __name__ == "main":
#         main()
main()