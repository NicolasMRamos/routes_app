#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "../data_structures/Graph.h"

// Edge Relaxation  ----------------------------------------------------------------------------------------------------

/**
 * @brief Relaxes an edge for driving, updating the destination vertex's driving distance if a shorter path is found.
 *
 * @param edge The edge to relax.
 * @return true if the relaxation was successful (i.e., a shorter path was found), false otherwise.
 */
bool driving_relax(Edge<int> *edge);


/**
 * @brief Relaxes an edge for walking, updating the destination vertex's walking distance if a shorter path is found.
 *
 * @param edge The edge to relax.
 * @return true if the relaxation was successful (i.e., a shorter path was found), false otherwise.
 */
bool walking_relax(Edge<int> *edge);

// Dijkstra for Driving ------------------------------------------------------------------------------------------------

/**
 * @brief Executes Dijkstra's algorithm for driving, computing the shortest driving paths from a given origin vertex.
 *
 * @param g The graph on which to execute the algorithm.
 * @param origin The ID of the origin vertex.
 */
void driving_dijkstra(Graph<int> *g, const int &origin);

// Dijkstra for Walking ------------------------------------------------------------------------------------------------

/**
 * @brief Executes Dijkstra's algorithm for walking, computing the shortest walking paths from a given origin vertex.
 *
 * @param g The graph on which to execute the algorithm.
 * @param origin The ID of the origin vertex.
 */
void walking_dijkstra(Graph<int> *g, const int &origin);

// Auxiliary Function to set up for Dijkstra execution -----------------------------------------------------------------

/**
 * @brief Resets the graph's vertices and edges to their initial state, allowing Dijkstra's algorithm to be re-run.
 *
 * @param g The graph to reset.
 */
void setup(Graph<int> *g);

// Auxiliary function to get Shortest Path -----------------------------------------------------------------------------

/**
 * @brief Retrieves the shortest driving path from the origin to the destination vertex.
 *
 * @param g The graph containing the vertices.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @return A vector of vertices representing the shortest driving path.
 */
std::vector<Vertex<int>*> getPathDrive(Graph<int> *g, const int &origin, const int &dest);

/**
 * @brief Retrieves the shortest walking path from the origin to the destination vertex.
 *
 * @param g The graph containing the vertices.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @return A vector of vertices representing the shortest walking path.
 */
std::vector<Vertex<int>*> getPathWalk(Graph<int> *g, const int &origin, const int &dest);

#endif //ALGORITHMS_H
