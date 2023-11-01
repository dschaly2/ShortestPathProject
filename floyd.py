import numpy as np
import time
import resource
from config import dataset
import pandas as pd
from tabulate import tabulate

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


def run(testcase, start, end):

    results = []
    time_start = time.process_time()
    

    all_shortest_distances, predecessors = floyd_warshall_with_path(dataset(testcase))
    shortest_distance = all_shortest_distances[start][end]

    if shortest_distance < float('inf'):
        results.append(f"{start} to {end}: {shortest_distance}")
        shortest_path = reconstruct_path(predecessors, start, end)
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
