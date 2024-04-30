import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

class Node:
    '''
    Class to represent a node in a network
    '''
    def __init__(self, value, number, connections=None):

        self.index = number
        self.connections = connections
        self.value = value

    def get_neighbours(self):
        '''
        Neighbours is a numpy array representing the row of the adjacency matrix that corresponds to the node
        '''
        return np.where(np.array(self.connections)==1)[0]

class Network:

    def __init__(self, nodes=None):
        
        if nodes is None:
             self.nodes = []

        else:
            self.nodes = nodes

    def get_mean_degree(self):
        '''
        Calculate the mean degree of the network
        '''
        #Loop through each nodes in the network
        total_degree = sum(sum(node.connections) for node in self.nodes)
        mean_degree = total_degree/ len(self.nodes)
        return mean_degree

    def get_mean_clustering(self):
        '''
        Calculate the mean clustering co-efficient
        (the fraction of a node's neighbours forming a triangle that includes the original node)
        '''
        #List of the clustering coefficient of each nodes
        clustering_coefficient = []
        
        #Loop through each nodes
        for node in self.nodes:
            #Find neighbours to each nodes
            neighbours = node.get_neighbours()

            #Number of neighbour
            num_neighbours = len(neighbours)

            #if the number of neighbours is less than 2,
            if num_neighbours < 2:
                clustering_coefficient.append(0)
                
            else:
                #Formula for the number of possible triangles that can be formed
                possible_triangles = (num_neighbours * (num_neighbours-1)) / 2
                connected_nodes = sum(node.connections[neighbour] for neighbour in neighbours)
                triangle = connected_nodes / possible_triangles
                clustering_coefficient.append(triangle)
                
        mean_clustering_coefficient = np.mean(clustering_coefficient)
        return mean_clustering_coefficient
    
    class Queue:
        def __init__(self):
            self.queue = []
            
        def push(self,item):
            pass
        
        def pop(self):
            pass
        def is_empty(self):
            return len(self.queue)==0
        
        def bfs(self, start_node, goal):
            start_node = network.nodes[0]
            goal = network.nodes[-1]
            search_queue = Queue()
            search_queue.push(start_node)
            visited = []
            
            while queue:
                current_node, distance = queue.pop(0)
                visited.add(current_node)
                
                if current_node == goal:
                    return distance
                for neighbour in current_node.neighbours:
                    if neighbour not in visited:
                        queue.append((neighbour, distance +1))
                
            return -1
        
    def get_mean_path_length(self):
        '''
        Calculate the mean path length
        (average of the distance between two nodes)
        '''
        #List of path length (distance between 2 nodes)
        total_path_length = 0
        for node in self.nodes:
            for neighbour in node.neighbours:
                total_path_length += self.bfs(node, neighbour)
        
        return total_path_length / (len(self.nodes) * len(self.nodes)-1)
        
    def make_random_network(self, N, connection_probability):
        '''
        This function makes a *random* network of size N.
        Each node is connected to each other node with probability p
        '''
        self.nodes = []
        
        for node_number in range(N):
            value = np.random.random()
            connections = [0 for _ in range(N)]
            self.nodes.append(Node(value, node_number, connections))
            
        for (index, node) in enumerate(self.nodes):
            for neighbour_index in range(index+1, N):
                if np.random.random() < connection_probability:
                    node.connections[neighbour_index] = 1
                    self.nodes[neighbour_index].connections[index] = 1

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_axis_off()
        
        num_nodes = len(self.nodes)
        network_radius = num_nodes * 10
        ax.set_xlim([-1.1*network_radius, 1.1*network_radius])
        ax.set_ylim([-1.1*network_radius, 1.1*network_radius])
        
        for (i, node) in enumerate(self.nodes):
            node_angle = i * 2 * np.pi / num_nodes
            node_x = network_radius * np.cos(node_angle)
            node_y = network_radius * np.sin(node_angle)
            
            circle = plt.Circle((node_x, node_y), 0.3*num_nodes, color=cm.hot(node.value))
            ax.add_patch(circle)
            
            for neighbour_index in range(i+1, num_nodes):
                if node.connections[neighbour_index]:
                    neighbour_angle = neighbour_index * 2 * np.pi / num_nodes
                    neighbour_x = network_radius * np.cos(neighbour_angle)
                    neighbour_y = network_radius * np.sin(neighbour_angle)
                    
                    ax.plot((node_x, neighbour_x), (node_y, neighbour_y), color='black')

def test_networks():

    #Ring network
    nodes = []
    num_nodes = 10
    for node_number in range(num_nodes):
        connections = [0 for val in range(num_nodes)]
        connections[(node_number-1)%num_nodes] = 1
        connections[(node_number+1)%num_nodes] = 1
        new_node = Node(0, node_number, connections=connections)
        nodes.append(new_node)
    network = Network(nodes)

    print("Testing ring network")
    assert(network.get_mean_degree()==2), network.get_mean_degree()
    assert(network.get_mean_clustering()==0), network.get_mean_clustering()
    assert(network.get_mean_path_length()==2.777777777777778), network.get_mean_path_length()

    nodes = []
    num_nodes = 10
    for node_number in range(num_nodes):
        connections = [0 for val in range(num_nodes)]
        connections[(node_number+1)%num_nodes] = 1
        new_node = Node(0, node_number, connections=connections)
        nodes.append(new_node)
    network = Network(nodes)

    print("Testing one-sided network")
    assert(network.get_mean_degree()==1), network.get_mean_degree()
    assert(network.get_mean_clustering()==0),  network.get_mean_clustering()
    assert(network.get_mean_path_length()==5), network.get_mean_path_length()

    nodes = []
    num_nodes = 10
    for node_number in range(num_nodes):
        connections = [1 for val in range(num_nodes)]
        connections[node_number] = 0
        new_node = Node(0, node_number, connections=connections)
        nodes.append(new_node)
    network = Network(nodes)

    print("Testing fully connected network")
    assert(network.get_mean_degree()==num_nodes-1), network.get_mean_degree()
    assert(network.get_mean_clustering()==1),  network.get_mean_clustering()
    assert(network.get_mean_path_length()==1), network.get_mean_path_length()

    print("All tests passed")


if __name__ == "__main__":
    test_networks()
