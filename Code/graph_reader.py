import networkx as nx
from numpy import random


class GraphReader:
    def read(self, path):
        with open(path, 'r') as f:
            graph = nx.Graph()
            start = -1
            target = -1
            for line in f:
                if not line.isspace():
                    line = line.split(';')[0][1:].strip()
                    line = line.split()
                    if line[0].startswith('E'):
                        eid = int(line[0][1:])
                        u = int(line[1])
                        v = int(line[2])
                        weight = int(line[3][1:])
                        bprob = 0
                        if len(line) == 5:
                            bprob = float(line[4][1:])
                        graph.add_edge(u, v, eid=eid, weight=weight, bprob=bprob)
                    elif line[0].startswith('V'):
                        graph.add_nodes_from(list(range(int(line[0][1:]))))
                    elif line[0].startswith('Start'):
                        start = int(line[1])
                    elif line[0].startswith('Target'):
                        target = int(line[1])
            if target not in graph[start].keys():
                max_weight = - 1
                for edge in graph.edges.data():
                    if edge[2]['weight'] > max_weight:
                        max_weight = edge[2]['weight']
                graph.add_edge(start, target, eid=len(graph.edges), weight=max_weight * (len(graph.edges) + 1), bprob=0)
            return graph, start, target

    def generate_graph_instance(self, graph):
        graph_instance = nx.Graph(graph)
        for edge in graph_instance.edges:
            r = random.uniform(0, 1)
            if r <= graph_instance.edges[edge]['bprob']:
                graph_instance.edges[edge]['bprob'] = 1
            else:
                graph_instance.edges[edge]['bprob'] = 0
        return graph_instance