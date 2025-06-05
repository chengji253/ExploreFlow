class Edge(object):
    def __init__(self, u, v, w):
        self.source = u
        self.sink = v
        self.capacity = w

    def __repr__(self):
        return "%s->%s:%s" % (self.source, self.sink, self.capacity)


class FlowAlgo(object):
    def __init__(self):
        self.adj = {}
        self.flow = {}

    def add_vertex(self, vertex):
        self.adj[vertex] = []

    def get_edges(self, v):
        return self.adj[v]

    def add_edge(self, u, v, w=0):
        if u == v:
            raise ValueError("u == v")
        edge = Edge(u, v, w)
        redge = Edge(v, u, 0)
        edge.redge = redge
        redge.redge = edge
        self.adj[u].append(edge)
        self.adj[v].append(redge)
        self.flow[edge] = 0
        self.flow[redge] = 0

    def find_path(self, source, sink, path):
        if source == sink:
            return path
        for edge in self.get_edges(source):
            residual = edge.capacity - self.flow[edge]
            if residual > 0 and edge not in path:
                result = self.find_path(edge.sink, sink, path + [edge])
                if result != None:
                    return result

    def find_min_cut_node_set(self, source):
        reachable = set()
        explore = [source]
        while explore:
            current = explore.pop()
            if current in reachable:
                continue
            reachable.add(current)
            for edge in self.get_edges(current):
                if edge.capacity - self.flow[edge] > 0 and edge.sink not in reachable:
                    explore.append(edge.sink)

        min_cut_edges = []
        for u in reachable:
            for edge in self.get_edges(u):
                if edge.sink not in reachable and self.flow[edge] > 0:
                    min_cut_edges.append(edge)
        print(min_cut_edges)
        return min_cut_edges

    def max_flow(self, source, sink):
        path = self.find_path(source, sink, [])
        while path != None:
            residuals = [edge.capacity - self.flow[edge] for edge in path]
            flow = min(residuals)
            for edge in path:
                self.flow[edge] += flow
                self.flow[edge.redge] -= flow
            path = self.find_path(source, sink, [])

        total_flow = 0
        edges = self.get_edges(source)
        for edge in edges:
            total_flow += self.flow[edge]
        # print()
        return total_flow
        # return sum(self.flow[edge] for edge in self.get_edges(source))


if __name__ == "__main__":
    g = FlowAlgo()
    # [g.add_vertex(v) for v in "sopqrt"]
    # g.add_edge('s', 'o', 3)
    # g.add_edge('s', 'p', 3)
    # g.add_edge('o', 'p', 2)
    # g.add_edge('o', 'q', 3)
    # g.add_edge('p', 'r', 2)
    # g.add_edge('r', 't', 3)
    # g.add_edge('q', 'r', 4)
    # g.add_edge('q', 't', 2)
    # print(g.max_flow('s', 't'))
    v_list = ['c1', 'c2', 'c3', 'c4', 'c5', 'c6']
    for v in v_list:
        g.add_vertex(v)
    g.add_edge('c1', 'c2', 4)
    g.add_edge('c1', 'c3', 4)
    g.add_edge('c2', 'c4', 1)
    g.add_edge('c3', 'c5', 1)

    g.add_edge('c4', 'c6', 4)
    g.add_edge('c5', 'c6', 4)

    print(g.max_flow('c1', 'c6'))
    print()