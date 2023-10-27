import heapq
import numpy as np

class Node:
    def __init__(self, index, parent=None, g_cost=float('inf'), h_cost=0):
        self.index = index
        self.parent = parent
        self.g_cost = g_cost
        self.h_cost = h_cost

    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        if self.f_cost() == other.f_cost():
            return self.g_cost > other.g_cost
        return self.f_cost() < other.f_cost()

def astar(adj_matrix, start, end):
    num_nodes = len(adj_matrix)
    open_set = []
    closed_set = set()
    visited_nodes = {}

    start_node = Node(start, None, 0, adj_matrix[start][end])
    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.index == end:
            path = reconstruct_path(current_node, visited_nodes)
            return current_node.g_cost, path

        if current_node.index in closed_set:
            continue

        closed_set.add(current_node.index)
        visited_nodes[current_node.index] = current_node

        for neighbor in range(num_nodes):
            if adj_matrix[current_node.index][neighbor] > 0:
                g_cost = current_node.g_cost + adj_matrix[current_node.index][neighbor]
                h_cost = adj_matrix[neighbor][end]  # Updated heuristic to actual edge cost
                neighbor_node = Node(neighbor, current_node, g_cost, h_cost)

                if neighbor_node.index not in closed_set:
                    heapq.heappush(open_set, neighbor_node)

    return float('inf'), []

def reconstruct_path(node, visited_nodes):
    path = []
    while node is not None:
        path.insert(0, node.index)
        node = node.parent
    return path

adjacency_matrix = np.array([
    # Matrix here
])

start_node = 0
end_node = 3

shortest_distance, nodes_traveled = astar(adjacency_matrix, start_node, end_node)

if shortest_distance < float('inf'):
    print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
    print(f"Nodes traveled: {nodes_traveled}")
else:
    print(f"No path from {start_node} to {end_node}")
