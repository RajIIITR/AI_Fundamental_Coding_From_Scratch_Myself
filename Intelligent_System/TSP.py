'''
Code to solve traveling salesman problem using minimum spanning tree as heuristic function which we can calculate 
by krushkal algorithm, we can prefer A* algorithm
'''


import heapq

# ---------- Step 1: Kruskal's Algorithm for MST ----------
class DisjointSet:
    def __init__(self, n):
        self.parent = list(range(n))

    def find(self, u):
        if self.parent[u] != u:
            self.parent[u] = self.find(self.parent[u])
        return self.parent[u]

    def union(self, u, v):
        pu, pv = self.find(u), self.find(v)
        if pu != pv:
            self.parent[pu] = pv


def kruskal_mst_cost(nodes, graph):
    """Compute MST cost using Kruskal for subset of nodes."""
    edges = []

    # Collect edges among remaining nodes
    for i in nodes:
        for j in nodes:
            if i < j:
                edges.append((graph[i][j], i, j))

    # Sort edges by weight
    edges.sort()
    ds = DisjointSet(len(graph))
    total_cost = 0
    count = 0

    for w, u, v in edges:
        if ds.find(u) != ds.find(v):
            ds.union(u, v)
            total_cost += w
            count += 1
            if count == len(nodes) - 1:
                break

    return total_cost


# ---------- Step 2: Heuristic Function ----------
def heuristic(current, unvisited, graph):
    """Estimate cost to finish tour from current using MST + nearest edges."""
    if not unvisited:
        return graph[current][0]  # Return to start

    # MST cost among unvisited nodes
    mst_cost = kruskal_mst_cost(unvisited, graph)

    # Minimum edge from current to any unvisited node
    min_edge_to_unvisited = min(graph[current][u] for u in unvisited)

    # Minimum edge from unvisited nodes back to start
    min_edge_to_start = min(graph[u][0] for u in unvisited)

    return mst_cost + min_edge_to_unvisited + min_edge_to_start


# ---------- Step 3: A* Algorithm ----------
def tsp_a_star(graph):
    n = len(graph)
    start_state = (0, 0, frozenset([0]))  # (cost_so_far, current_city, visited_set)
    pq = [(0, start_state, [0])]  # (f_score, state, path)

    best_cost = float('inf')
    best_path = None

    while pq:
        f, (g, current, visited), path = heapq.heappop(pq)

        if len(visited) == n:
            total_cost = g + graph[current][0]  # return to start
            if total_cost < best_cost:
                best_cost = total_cost
                best_path = path + [0]
            continue

        # Expand neighbors
        unvisited = set(range(n)) - visited
        for nxt in unvisited:
            g_new = g + graph[current][nxt]
            h_new = heuristic(nxt, unvisited - {nxt}, graph)
            f_new = g_new + h_new
            new_state = (g_new, nxt, visited | {nxt})
            heapq.heappush(pq, (f_new, new_state, path + [nxt]))

    return best_cost, best_path


# ---------- Step 4: Example ----------
if __name__ == "__main__":
    # Example graph (symmetric TSP)
    graph = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0]
    ]

    cost, path = tsp_a_star(graph)
    print("\nOptimal Path:", path)
    print("Minimum Cost:", cost)