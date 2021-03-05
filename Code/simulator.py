from graph_reader import GraphReader
from mdp import MDP


class Simulator:
    def __init__(self, path):
        self.gr = GraphReader()
        self.graph, self.start, self.target = self.gr.read(path)
        self.mdp = MDP(self.graph, self.start, self.target)
        self.policy = self.mdp.value_iteration()

    def print_belief_nodes(self):
        print("PRINTING BELIEF NODES")
        for bn_id in self.mdp.bn_graph.nodes:
            if bn_id != -1:
                self.print_belief_node(bn_id)
        print('-' * 20)

    def print_belief_node(self, bn_id):
        edge_list = []
        for edge in self.mdp.bn_graph.nodes.data()[bn_id]:
            if edge != 'node' and edge != 'reachable':
                value = 'U'
                if self.mdp.bn_graph.nodes.data()[bn_id][edge] == 0:
                    value = 'F'
                elif self.mdp.bn_graph.nodes.data()[bn_id][edge] == 1:
                    value = 'B'
                edge_list.append((edge, value))
        str_edge_list = [f'{edge[0][0]}-{edge[0][1]}:{edge[1]}' for edge in edge_list]
        print(f'BN node={self.mdp.bn_graph.nodes.data()[bn_id]["node"]} with edges {", ".join(str_edge_list)}'
              f'{" with utility=" + str(self.mdp.reachable_tree.nodes[bn_id]["utility"]) + ((" goto " + str(self.policy[bn_id])) if self.policy[bn_id]!=None else " target") if self.mdp.bn_graph.nodes.data()[bn_id]["reachable"] else " is unreachable"}')

    def run(self):
        self.run_simulation(self.mdp, self.graph, self.start, self.target, self.policy)
        while input("Run another simulation [Y|N]?").lower() == 'y':
            self.run_simulation(self.mdp, self.graph, self.start, self.target, self.policy)

    def run_simulation(self, mdp, graph, start, target, policy):
        g_instance = self.gr.generate_graph_instance(graph)
        print("The edges in the graph instance are:")
        for edge in g_instance.edges:
            if g_instance.edges[edge]['bprob'] == 0:
                print(edge)
        total_cost = 0
        belief_state = self.update_belief_state(mdp.get_special_start(), g_instance, start).copy()
        del belief_state['reachable']
        print("The agent takes the path:")
        while belief_state['node'] != target:
            print(belief_state)
            edges = belief_state.copy()
            del edges['node']
            sn = mdp.node_to_id(belief_state['node'], edges)
            next_loc = policy[sn]
            total_cost -= g_instance[belief_state['node']][next_loc]['weight']
            self.update_belief_state(belief_state, g_instance, next_loc)
        print(belief_state)
        print(f"total cost of trip was {-total_cost}")

    def update_belief_state(self, state, g_instance, next_loc):
        state['node'] = next_loc
        for key in state:
            if key != 'node' and key != 'reachable':
                if key[0] == state['node'] or key[1] == state['node']:
                    state[key] = g_instance[key[0]][key[1]]['bprob']
        return state
