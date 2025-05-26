#ifndef DRIVING_H
#define DRIVING_H

#include <vector>
#include <utility>

#include "../data_structures/Graph.h"

// Driving Computation: no restrictions --------------------------------------------------------------------------------

/**
 * @brief Computes the best and alternative driving routes from an origin to a destination without any restrictions.
 *
 * @param g The graph containing the vertices and edges.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @param batch If true, outputs the results to a file; otherwise, outputs to the console.
 */
void driving_mode(Graph<int> *g, const int &origin, const int &dest, bool batch = false);

// Driving Computation: with restrictions ------------------------------------------------------------------------------

/**
 * @brief Computes the best driving route from an origin to a destination while avoiding specific nodes and edges, and optionally including a specific node.
 *
 * @param g The graph containing the vertices and edges.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @param avoid_nodes A vector of node IDs to avoid in the route.
 * @param avoid_edges A vector of pairs representing edges to avoid in the route.
 * @param include_node The ID of a node that must be included in the route, or -1 if no such node is required.
 * @param batch If true, outputs the results to a file; otherwise, outputs to the console.
 */
void driving_mode(Graph<int> *g, const int &origin, const int &dest, const std::vector<int> &avoid_nodes, const std::vector<std::pair<int,int>> &avoid_edges, const int &include_node = -1, bool batch = false);

#endif //DRIVING_H
