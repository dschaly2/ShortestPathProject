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

def astar(adj_matrix, start, end):
    num_nodes = len(adj_matrix)
    open_set = []
    closed_set = set()

    start_node = Node(start, None, 0, heuristic(start, end))
    heapq.heappush(open_set, (start_node.f_cost(), start_node))

    while open_set:
        _, current_node = heapq.heappop(open_set)

        if current_node.index == end:
            # Reconstruct the path and return the distance
            distance = current_node.g_cost
            return distance

        if current_node.index in closed_set:
            continue

        closed_set.add(current_node.index)

        for neighbor in range(num_nodes):
            if adj_matrix[current_node.index][neighbor] > 0:
                g_cost = current_node.g_cost + adj_matrix[current_node.index][neighbor]
                h_cost = heuristic(neighbor, end)
                neighbor_node = Node(neighbor, current_node, g_cost, h_cost)

                if neighbor_node.index not in closed_set:
                    heapq.heappush(open_set, (neighbor_node.f_cost(), neighbor_node))

    return float('inf')  # No path found

def heuristic(node, end):
    # A simple heuristic (Euclidean distance between nodes)
    return np.sqrt((node // 2 - end // 2) ** 2 + (node % 2 - end % 2) ** 2)

# Example usage:
adjacency_matrix = np.array([
    [0, 4, 8, 6, 12],
    [4, 0, 6, 15, 3],
    [8, 6, 0, 14, 7],
    [12, 3, 7, 8, 0]
])

start_node = 0
end_node = 3

shortest_distance = astar(adjacency_matrix, start_node, end_node)

if shortest_distance < float('inf'):
    print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
else:
    print(f"No path from {start_node} to {end_node}")

