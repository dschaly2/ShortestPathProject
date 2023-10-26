import numpy as np

def dijkstra(adj_matrix, start, end):
    num_nodes = len(adj_matrix)
    distances = [float('inf')] * num_nodes
    distances[start] = 0

    unvisited = list(range(num_nodes))

    while unvisited:
        current_node = min(unvisited, key=lambda node: distances[node])
        unvisited.remove(current_node)

        if current_node == end:
            return distances[end]  # Return the distance when the end node is reached

        for neighbor in range(num_nodes):
            if adj_matrix[current_node][neighbor] > 0:
                distance = distances[current_node] + adj_matrix[current_node][neighbor]

                if distance < distances[neighbor]:
                    distances[neighbor] = distance

    return float('inf')  # Return infinity if no path to the end node is found

# Example usage:
adjacency_matrix = np.array([
    [0, 4, 8, 6, 12],
    [4, 0, 6, 15, 3],
    [8, 6, 0, 14, 7],
    [12, 3, 7, 8, 0]
])

start_node = 0
end_node = 3

shortest_distance = dijkstra(adjacency_matrix, start_node, end_node)

if shortest_distance < float('inf'):
    print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
else:
    print(f"No path from {start_node} to {end_node}")

