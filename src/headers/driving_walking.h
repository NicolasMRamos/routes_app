#ifndef DRIVING_WALKING_H
#define DRIVING_WALKING_H

#include <vector>
#include <utility>

#include "../data_structures/Graph.h"


/**
 * @brief Computes the most eco-friendly route from an origin to a destination, combining driving and walking.
 *
 * This function calculates the optimal route by driving to a parking node and then walking to the destination,
 * while respecting constraints such as maximum walking time and avoiding specific nodes and edges.
 *
 * @param g The graph containing the vertices and edges.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @param max_walk_time The maximum allowed walking time in minutes.
 * @param avoid_nodes A vector of node IDs to avoid in the route.
 * @param avoid_edges A vector of pairs representing edges to avoid in the route.
 */

void eco_mode(Graph<int> *g, const int &origin, const int &dest, const double &max_walk_time, const std::vector<int> &avoid_nodes, const std::vector<std::pair<int,int>> &avoid_edges, bool batch = false);

void eco_mode_approximate(Graph<int> *g, const int &origin, const int &dest, const double &max_walk_time, const std::vector<int> &avoid_nodes, const std::vector<std::pair<int,int>> &avoid_edges, bool batch = false);


#endif //DRIVING_WALKING_H
