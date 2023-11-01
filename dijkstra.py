import numpy as np
import time
import resource
from config import dataset
import pandas as pd
from tabulate import tabulate

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

def run(testcase, start, end):

    results = []

    time_start = time.process_time()
    
    shortest_distance, shortest_path = dijkstra(dataset(testcase), start, end)

    if shortest_distance < float('inf'):
        results.append(f"{start} to {end}: {shortest_distance}")
        results.append(f"{shortest_path}")

        time_elapsed = (time_start)
        memMb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.0/1024.0
        results.append("%5.1f secs %5.1f MByte" % (time_elapsed,memMb))
        df = pd.DataFrame(results, index=['Shortest Distance', 'Nodes Traveled', 'Time/Memory'], columns=['Data'])
    else:
        print(f"No path from {start} to {end}")

        time_elapsed = (time.perf_counter() - time_start)
        memMb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.0/1024.0
        print("%5.1f secs %5.1f MByte" % (time_elapsed,memMb))

    return tabulate(df, headers = 'keys', tablefmt = 'psql')


print(run("Bestcase", 0, 9))
print(run("Worstcase", 0, 5))
print(run("Smallest", 0, 3))
print(run("Longest", 7, 21))
print(run("Blockedcase", 3,8))
