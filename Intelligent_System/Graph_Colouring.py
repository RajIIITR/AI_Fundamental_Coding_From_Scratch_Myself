class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[] for _ in range(vertices)]
        self.color_V = self.V * [0]

    def add_edge(self, u, v):
        self.graph[u].append(v)
        self.graph[v].append(u)
        
    def coloring(self):
        for ver, neighbors in enumerate(self.graph):
            for neigh in neighbors:
                if self.color_V[ver] == self.color_V[neigh]:
                    self.color_V[neigh] += 1 
                    
        for i, color in enumerate(self.color_V):
            print(f"node {i} has color {color}")

    def is_colorable(self, color_V):
        for ver, neighbors in enumerate(self.graph):
            for neigh in neighbors:
                if color_V[ver] == color_V[neigh]:   
                    return False
        return True
        
        
if __name__ == "__main__":
    graph = Graph(6)
    graph.add_edge(0, 1)
    graph.add_edge(0, 3)
    graph.add_edge(1, 2)
    graph.add_edge(1, 4)
    graph.add_edge(1, 5)
    graph.add_edge(2, 4)
    graph.add_edge(3, 4)
    graph.add_edge(3, 5)

    # print("Coloring of vertices:")
    # graph.coloring()

    color_V = [0, 1, 0, 1, 2, 2]  

    print("Graph colorable?", graph.is_colorable(color_V))