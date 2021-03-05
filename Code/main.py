from simulator import Simulator

if __name__ == '__main__':
    path = input("Enter path: ").replace('"', '')
    sim = Simulator(path)
    sim.print_belief_nodes()
    sim.run()
    print('done')
