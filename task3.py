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
  		Neighbours is a numpy array representing the row of the adjency matrix that corresponds to the node
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
		for node in self.nodes:

			#Calculate the total degree of all nodes in the network
			total_degree = sum(len(node.connections))

		#Calculate the average of the degree of all nodes in the network
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
			neighbours = node.get_neighbours()
			num_neighbours = len(neighbours)

			possible_triangles = (num_neighbours * (num_neighbours-1)) / 2

			if num_neighbours < 2:
				continue

			triangles = 0

			for i in range(num_neighbours):
				for j in range(i+1, num_neighbours):
					if self.nodes[neighbours[i]].connections[neighbours[j]] == 1:
						triangles+=1
			clustering_coefficient.append(triangles/ possible_triangles)
			mean_clustering_coefficient = sum(clustering_coefficient)/ len(clustering_coefficient)
			return mean_clustering_coefficient

	def get_mean_path_length(self):
		#Your code for task 3 goes here
		'''
  			Calculate the mean path length
			(average of the distance between two nodes)
   		'''
		

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
	assert(network.get_clustering()==0), network.get_clustering()
	assert(network.get_path_length()==2.777777777777778), network.get_path_length()

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
	assert(network.get_clustering()==0),  network.get_clustering()
	assert(network.get_path_length()==5), network.get_path_length()

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
	assert(network.get_clustering()==1),  network.get_clustering()
	assert(network.get_path_length()==1), network.get_path_length()

	print("All tests passed")

test_networks()