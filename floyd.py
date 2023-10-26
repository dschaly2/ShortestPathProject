import numpy as np

def floyd_warshall(adj_matrix):
    num_nodes = len(adj_matrix)
    distances = np.copy(adj_matrix)

    for k in range(num_nodes):
        for i in range(num_nodes):
            for j in range(num_nodes):
                if distances[i][j] > distances[i][k] + distances[k][j]:
                    distances[i][j] = distances[i][k] + distances[k][j]

    return distances

# Example usage:
adjacency_matrix = np.array([
    [0, 4, 8, 6, 12],
    [4, 0, 6, 15, 3],
    [8, 6, 0, 14, 7],
    [12, 3, 7, 8, 0]
])

start_node = 0
end_node = 3

all_shortest_distances = floyd_warshall(adjacency_matrix)
shortest_distance = all_shortest_distances[start_node][end_node]

if shortest_distance < float('inf'):
    print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
else:
    print(f"No path from {start_node} to {end_node}")

