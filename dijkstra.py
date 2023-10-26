import numpy as np

def dijkstra(adj_matrix, start, end):
    num_nodes = len(adj_matrix)
    distances = [float('inf')] * num_nodes
    predecessors = [None] * num_nodes
    distances[start] = 0

    unvisited = list(range(num_nodes))

    while unvisited:
        current_node = min(unvisited, key=lambda node: distances[node])
        unvisited.remove(current_node)

        if current_node == end:
            return distances[end], reconstruct_path(predecessors, start, end)  # Return distance and path

        for neighbor in range(num_nodes):
            if adj_matrix[current_node][neighbor] > 0:
                distance = distances[current_node] + adj_matrix[current_node][neighbor]

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    predecessors[neighbor] = current_node

    return float('inf'), []  # Return infinity and an empty path if no path to the end node is found

def reconstruct_path(predecessors, start, end):
    path = []
    while end is not None:
        path.insert(0, end)
        end = predecessors[end]
    return path

# Input Adjacency Matrix
adjacency_matrix = np.array([
    # Matrix Here
])

start_node = 0
end_node = 3

shortest_distance, shortest_path = dijkstra(adjacency_matrix, start_node, end_node)

if shortest_distance < float('inf'):
    print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
    print(f"Shortest path: {shortest_path}")
else:
    print(f"No path from {start_node} to {end_node}")
