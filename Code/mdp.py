import copy

import networkx as nx


class MDP:
    def __init__(self, graph: nx.Graph, start, target):
        self.graph = graph
        self.bns = {vertex: [] for vertex in graph.nodes}
        self.bn_graph = nx.DiGraph()
        self._add_bns()
        self._add_edges()
        self._add_special_start(start)
        self.target = target
        self.reachable_tree = None
        self.create_reachable_mdp(-1)

    def _add_special_start(self, node):
        possible_starts = []
        for bn in self.node_to_id(node):
            if self._is_starting_bn_legal((bn, self.bn_graph.nodes[bn])):
                possible_starts.append(bn)
        self.bn_graph.add_node(-1, reachable=False, node=-1)
        for start in possible_starts:
            self.bn_graph.add_edge(-1, start, prob=1)
        prob_edges = []
        for edge in self.graph.edges.data():
            if edge[2]['bprob'] > 0:
                prob_edges.append((edge[0], edge[1], edge[2]['bprob']))
        for i in range(len(prob_edges)):
            self.bn_graph.nodes[-1][(prob_edges[i][0], prob_edges[i][1])] = prob_edges[i][2]

    def get_special_start(self):
        return self.bn_graph.nodes[-1]

    def create_reachable_mdp(self, bn_id):
        reachable = []
        for node in nx.dfs_postorder_nodes(self.bn_graph, source=bn_id):
            reachable.append(node)
            self.bn_graph.nodes[node]['reachable'] = True
        reachable.remove(-1)
        self.bn_graph.nodes[-1]['reachable'] = False
        self.reachable_tree = self.bn_graph.copy()
        for node in self.bn_graph.nodes:
            if node not in reachable:
                self.reachable_tree.remove_node(node)
        for node in self.reachable_tree.nodes:
            self.reachable_tree.nodes[node]['utility'] = None

    def _add_bns(self):
        prob_edges = []
        for edge in self.graph.edges.data():
            if edge[2]['bprob'] > 0:
                prob_edges.append((edge[0], edge[1], edge[2]['bprob']))
        all_blockages = [[]]
        for edge in prob_edges:
            length = len(all_blockages)
            all_blockages += copy.deepcopy(all_blockages) + copy.deepcopy(all_blockages)
            for i in range(length):
                all_blockages[i].append(0)
                all_blockages[i + length].append(edge[2])
                all_blockages[i + length * 2].append(1)
        curr_id = 0
        for node in self.graph.nodes:
            for bn_assignment in all_blockages:
                self.bn_graph.add_node(curr_id, reachable=False)
                self.bn_graph.nodes[curr_id]['node'] = node
                for i in range(len(prob_edges)):
                    self.bn_graph.nodes[curr_id][(prob_edges[i][0], prob_edges[i][1])] = bn_assignment[i]
                curr_id += 1

    def _add_edges(self):
        for bn1 in self.bn_graph.nodes.data():
            if not self._is_legal_node(bn1):
                continue
            for bn2 in self.bn_graph.nodes.data():
                if not self._is_legal_node(bn2):
                    continue
                if bn1[0] == bn2[0]:
                    continue
                if not self._is_legal_edge(bn1, bn2):
                    continue
                self._add_edge(bn1, bn2)

    def _is_legal_node(self, bn):
        for edge in bn[1]:
            if edge != 'node':
                if (edge[0] == bn[1]['node'] or edge[1] == bn[1]['node']) and 0 < bn[1][edge] < 1:
                    return False
        return True

    def _is_legal_edge(self, bn1, bn2):
        if self.graph.has_edge(bn1[1]['node'], bn2[1]['node']):
            for edge in bn1[1]:
                if edge != 'node':
                    if edge == (bn1[1]['node'], bn2[1]['node']) and (bn1[1][edge] == 1):
                        return False
                    # Check if the information was retained while traversing
                    # Probs have to be the same unless bn1 had prob and bn2 is neighbour to that edge
                    if (bn1[1][edge] != bn2[1][edge]) and (bn1[1][edge] == 0 or bn1[1][edge] == 1):
                        return False
                    # If we had probability, and it changed to 0 or 1, and the vertex is not a neighbour to that
                    # edge, then it is illegal
                    if 0 < bn1[1][edge] < 1 and (bn2[1][edge] == 0 or bn2[1][edge] == 1) and \
                            not (edge[0] == bn2[1]['node'] or edge[1] == bn2[1]['node']):
                        return False
            return True
        return False

    def _add_edge(self, bn1, bn2):
        probability = 1
        for edge in bn1[1]:
            if edge != 'node':
                if 0 < bn1[1][edge] < 1:
                    if bn2[1][edge] == 0:
                        probability *= (1 - bn1[1][edge])
                    elif bn2[1][edge] == 1:
                        probability *= bn1[1][edge]
        self.bn_graph.add_edge(bn1[0], bn2[0], prob=probability)

    def node_to_id(self, node, edge_probs=None):
        ret = []
        for bn in self.bn_graph.nodes.data():
            if bn[1]['node'] == node:
                is_node = True
                if edge_probs is not None:
                    for edge in edge_probs:
                        if bn[1][edge] != edge_probs[edge]:
                            is_node = False
                            break
                    if is_node:
                        return bn[0]
                else:
                    ret.append(bn[0])
        if len(ret) > 0:
            return ret
        else:
            raise ValueError('No such belief node')

    def value_iteration(self):
        policy = {}
        target = self.node_to_id(self.target)
        for bn in target:
            if bn in self.reachable_tree.nodes:
                policy[bn] = None
                self.reachable_tree.nodes[bn]['utility'] = 0
        while True:
            should_break = True
            # Iterate over all nodes until convergence
            for node in self.reachable_tree.nodes:
                best_policy = None
                # Look at all neighbors as possible actions
                for neighbor in self.graph.neighbors(self.reachable_tree.nodes[node]['node']):
                    # Initialize the utility to the cost of moving to that neighbor
                    weight_to_neighbor = self.graph[self.reachable_tree.nodes[node]['node']][neighbor]['weight']
                    utility = -weight_to_neighbor
                    # Look at all bn nodes corresponding to moving to that neighbor
                    neighbor_exists = False
                    for bn_neighbor in self.reachable_tree.neighbors(node):
                        if self.reachable_tree.nodes[bn_neighbor]['node'] == neighbor and \
                                self.reachable_tree.nodes[bn_neighbor]['utility'] is not None:
                            neighbor_exists = True
                            utility += (self.reachable_tree[node][bn_neighbor]['prob'] *
                                        self.reachable_tree.nodes[bn_neighbor]['utility'])
                    if neighbor_exists and (self.reachable_tree.nodes[node]['utility'] is None or utility >
                                            self.reachable_tree.nodes[node]['utility']):
                        self.reachable_tree.nodes[node]['utility'] = utility
                        best_policy = neighbor
                        should_break = False
                if best_policy is not None:
                    policy[node] = best_policy
            if should_break:
                break
        return policy

    def _is_starting_bn_legal(self, bn):
        for edge in bn[1]:
            if edge != 'node' and edge != 'reachable':
                if (edge[0] == bn[1]['node'] or edge[1] == bn[1]['node']) and 0 < bn[1][edge] < 1:
                    return False
                if (edge[0] != bn[1]['node'] and edge[1] != bn[1]['node']) and (bn[1][edge] == 0 or bn[1][edge] == 1):
                    return False
        return True
