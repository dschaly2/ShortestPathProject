import numpy as np

def floyd_warshall_with_path(adj_matrix):
    num_nodes = len(adj_matrix)
    distances = np.copy(adj_matrix)
    predecessors = np.full((num_nodes, num_nodes), -1, dtype=int)

    for k in range(num_nodes):
        for i in range(num_nodes):
            for j in range(num_nodes):
                if distances[i][j] > distances[i][k] + distances[k][j]:
                    distances[i][j] = distances[i][k] + distances[k][j]
                    predecessors[i][j] = k  # Store the intermediate node k

    return distances, predecessors

def reconstruct_path(predecessors, start, end):
    if predecessors[start][end] == -1:
        return [start, end]
    intermediate = predecessors[start][end]
    path1 = reconstruct_path(predecessors, start, intermediate)
    path2 = reconstruct_path(predecessors, intermediate, end)
    return path1[:-1] + path2


adjacency_matrix = np.array([
    # Matrix here
])

start_node = 6
end_node = 17

all_shortest_distances, predecessors = floyd_warshall_with_path(adjacency_matrix)
shortest_distance = all_shortest_distances[start_node][end_node]

if shortest_distance < float('inf'):
    print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
    shortest_path = reconstruct_path(predecessors, start_node, end_node)
    print(f"Shortest path: {shortest_path}")
else:
    print(f"No path from {start_node} to {end_node}")
