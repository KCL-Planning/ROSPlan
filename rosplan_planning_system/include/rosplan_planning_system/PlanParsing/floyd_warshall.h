/**
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * Floy-Warshall algorithm for computing minimum distance between nodes in a directed graph
 *
 * code partially reused from: https://www.geeksforgeeks.org/floyd-warshall-algorithm-dp-16/
 */

#include <iostream>
#include <stdio.h>
#include <vector>
#include <limits>
#include <stdexcept>

class floydWarshall
{
  public:
	/**
	 * @brief empty constructor, relies on setDistanceMatrix(args) function to be called first
	 */
	floydWarshall();

	/**
	 * @brief overloaded constructor, allows to set graph at object creation
	 * @param graph 2D matrix that contains the graph connectivity
	 */
	floydWarshall(std::vector<std::vector<double> > &graph);

	/**
	 * @brief input the graph to the class, will be used as initial distance matrix
	 * saves graph to member variable for future use
	 */
	void setDistanceMatrix(std::vector<std::vector<double> > &graph);

	/**
	 * @brief Solves the all-pairs shortest path problem using Floyd Warshall algorithm
	 * @return true if algorithm found solution, false if there was an error
	 */
	bool updateDistanceMatrix();

	/**
	 * @brief make empty matrix of nxn and fill it diagonally with 0's, upper and lower triangle with inf
	 * @param number_of_nodes the size of the square (minimum distance / connectivity) graph matrix
	 */
	void setEmptyDistanceMatrix(int number_of_nodes);

	/**
	 * @brief helper function to determine if two nodes are connected in a directed graph
	 * based on the minimum distance matrix (needs to be updated so call updateDistanceMatrix first)
	 * @param source_node_id the start node from which the path is analyzed
	 * @param sink_node_id the end node in which the path ends
	 * @return true if there is a combination of directed edges that connects them trough a path
	 * false otherwise
	 */
	bool connectedSingleQuery(int source_node_id, int sink_node_id);

	/**
	 * @brief based on existance matrix is easy to check if there is already an edge in the graph
	 * this function however is valid only before calling updateDistanceMatrix
	 * @param source_node_id the parent node from which the edge is coming from
	 * @param sink_node_id the child node to which the edge is going
	 * @return true if edge between nodes exist, false otherwise
	 */
	bool checkEdgeExistance(int source_node_id, int sink_node_id);

	/**
	 * @brief helper function useful to create graph incrementally (add edges one by one)
	 * @param source_node_id the id of the node from which the edge is comming from
	 * @param sink_node_id the id of the node to wich the edge is going to
	 * @param weight the weight of the directed edge
	 * @return true if the edge was created, false if the edge existed previously : edge will not be created
	 */
	bool makeNonDuplicateEdge(int source_node_id, int sink_node_id, double weight);

	/**
	 * @brief Sample function for debugging purposes that allows to print distance_matrix_[][]
	 * @return void, this funciton prints to console
	 */
	void printMatrix();

  private:

	// flag to indicate floyd warshall algorithm is ready to be computed
	bool init_;

	// flag to indicate if checkEdgeExistance function is valid or not
	bool distance_matrix_updated_once_;

	// the number of nodes in the directed graph
	int number_of_nodes_;

	// graph as matrix, which will be converted later into distance matrix
	std::vector<std::vector<double> > distance_matrix_;
};
