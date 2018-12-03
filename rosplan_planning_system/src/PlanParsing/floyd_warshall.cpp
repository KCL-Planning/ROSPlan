/**
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * Floy-Warshall algorithm for computing minimum distance_matrix_ance between nodes in a directed graph
 *
 * code partially reused from: https://www.geeksforgeeks.org/floyd-warshall-algorithm-dp-16/
 */

#include "rosplan_planning_system/PlanParsing/floyd_warshall.h"

floydWarshall::floydWarshall(): number_of_nodes_(-1), init_(false), distance_matrix_updated_once_(false) {}

floydWarshall::floydWarshall(std::vector<std::vector<double> > &graph): number_of_nodes_(-1), init_(false),
distance_matrix_updated_once_(false) {

	setDistanceMatrix(graph);
}

void floydWarshall::setDistanceMatrix(std::vector<std::vector<double> > &graph) {

	// save initial graph in member variable for future use
	distance_matrix_ = graph;

	// get length of square matrix
	number_of_nodes_ = distance_matrix_[0].size();

	// all set, ready to compute floyd warshall
	init_ = true;
}

bool floydWarshall::updateDistanceMatrix() {

	// number of nodes not set
	if(!init_) {
		std::cerr << "error: graph not set, please call setDistanceMatrix(args) function before updateDistanceMatrix(args)" << std::endl;
		return false;
	}

	/* Add all vertices one by one to the set of intermediate vertices.
	---> Before start of an iteration, we have shortest distance_matrix_ances between all
	pairs of vertices such that the shortest distance_matrix_ances consider only the
	vertices in set {0, 1, 2, .. k-1} as intermediate vertices.
	----> After the end of an iteration, vertex no. k is added to the set of
	intermediate vertices and the set becomes {0, 1, 2, .. k} */
	for (size_t k = 0; k < number_of_nodes_; k++) {
		// Pick all vertices as source one by one
		for (size_t i = 0; i < number_of_nodes_; i++) {
			// Pick all vertices as destination for the
			// above picked source
			for (size_t j = 0; j < number_of_nodes_; j++) {
				// If vertex k is on the shortest path from
				// i to j, then update the value of distance_matrix_[i][j]
				if (distance_matrix_[i][k] + distance_matrix_[k][j] < distance_matrix_[i][j])
					distance_matrix_[i][j] = distance_matrix_[i][k] + distance_matrix_[k][j];
			}
		}
	}

	return true;
}

void floydWarshall::setEmptyDistanceMatrix(int number_of_nodes) {

	number_of_nodes_ = number_of_nodes;

	// remove old data if any
	distance_matrix_.clear();

	// lower flag, empty matrix is not updated
	distance_matrix_updated_once_ = false;

	// grow initial matrix to the value of number of nodes
	distance_matrix_.resize(number_of_nodes_);

	// resize distance_matrix_ std vector to number_of_nodes
	distance_matrix_.resize(number_of_nodes_);
	for (size_t i = 0; i < number_of_nodes_; i++)
		distance_matrix_[i].resize(number_of_nodes_);

	// init all nodes to inf, and those which are equal to itself to 0
	for (size_t i = 0; i < number_of_nodes_; i++) {
		for(size_t j = 0; j < number_of_nodes_; j++) {
			if(i == j)
				distance_matrix_[i][j] = 0;
			else
				distance_matrix_[i][j] = std::numeric_limits<double>::max();
		}
	}

	// all set, now you have an empty graph, ready to be filled incrementally
	init_ = true;
}

bool floydWarshall::connectedSingleQuery(int source_node_id, int sink_node_id) {

	// number of nodes not set
	if(!init_) {
		std::cerr << "error: graph not set, please call setDistanceMatrix(args) function before updateDistanceMatrix(args)" << std::endl;
		return false;
	}

	if(distance_matrix_[source_node_id][sink_node_id] < std::numeric_limits<double>::max())
		return true;
	else
		return false;
}

void floydWarshall::printMatrix() {

	std::cout << "The following matrix shows the shortest distances in the graph"
		 << " between every pair of vertices" << std::endl;

	for (size_t i = 0; i < number_of_nodes_; i++) {
		for (size_t j = 0; j < number_of_nodes_; j++) {
			if (distance_matrix_[i][j] == std::numeric_limits<double>::max())
				printf("%7s", "inf");
			else
				printf ("%7.2lf", distance_matrix_[i][j]);
		}
		printf("\n");
	}
}

bool floydWarshall::checkEdgeExistance(int source_node_id, int sink_node_id) {

	if(distance_matrix_updated_once_) {
		// throw exception
		throw std::invalid_argument("checkEdgeExistance function does not work if updateDistanceMatrix was called");
	}

	if(distance_matrix_[source_node_id][sink_node_id] < std::numeric_limits<double>::max())
		return true; // edge exists
	else
		return false;
}

bool floydWarshall::makeNonDuplicateEdge(int source_node_id, int sink_node_id, double weight) {

	if(checkEdgeExistance(source_node_id, sink_node_id))
		return false; // edge exists, don't add

	// edge does not exist, add
	distance_matrix_[source_node_id][sink_node_id] = weight;
	return true;
}

void RunExample1() {
	// this routine shows an example of how to use this class

	// object creation
	floydWarshall example_problem1;

	/* Let us create the following weighted graph
            10
       (0)------->(3)
        |         /|\
      5 |          |
        |          | 1
       \|/         |
       (1)------->(2)
            3           */
	double inf = std::numeric_limits<double>::max();
	std::vector<std::vector<double> > graph = { {0.0, 5.0, inf, 10.0},
												{inf, 0.0, 3.0, inf},
												{inf, inf, 0.0, 1.0},
												{inf, inf, inf, 0.0} };

	// set graph
	example_problem1.setDistanceMatrix(graph);

	// compute floyd-Warshall to get min distance matrix
	example_problem1.updateDistanceMatrix();

	// Print the solution
	example_problem1.printMatrix();

	// check connectivity between nodes
	int source_node_id = 1;
	int sink_node_id = 3;

	if(example_problem1.connectedSingleQuery(source_node_id, sink_node_id))
		std::cout << "nodes " << source_node_id << " and " << sink_node_id << " are connected" << std::endl;
	else
		std::cout << "nodes " << source_node_id << " and " << sink_node_id << " are not connected" << std::endl;
}

void RunExample2() {
	// this routine shows an example of how to use this class

	// incremental graph update example

	// object creation
	floydWarshall example_problem2;

	/* Let us create the following weighted graph incrementally
            10
       (0)------->(3)
        |         /|\
      5 |          |
        |          | 1
       \|/         |
       (1)------->(2)
            3           */
	// create empty init matrix
	example_problem2.setEmptyDistanceMatrix(4); // 4 nodes

	// create edges incrementally
	example_problem2.makeNonDuplicateEdge(0, 1, 5.0);
	example_problem2.makeNonDuplicateEdge(1, 2, 3.0);
	example_problem2.makeNonDuplicateEdge(2, 3, 1.0);
	example_problem2.makeNonDuplicateEdge(0, 3, 10.0);

	// compute floyd-Warshall to get min distance matrix
	example_problem2.updateDistanceMatrix();

	// Print the solution
	example_problem2.printMatrix();

	// check connectivity between nodes
	int source_node_id = 1;
	int sink_node_id = 3;

	if(example_problem2.connectedSingleQuery(source_node_id, sink_node_id))
		std::cout << "nodes " << source_node_id << " and " << sink_node_id << " are connected" << std::endl;
	else
		std::cout << "nodes " << source_node_id << " and " << sink_node_id << " are not connected" << std::endl;
}

int main() {

	// run simple example
	RunExample1();
	std::cout << "-----" << std::endl;

	// run incremental graph update example
	RunExample2();

	return 0;
}
