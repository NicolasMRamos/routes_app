#ifndef PARSING_H
#define PARSING_H

#include <fstream>

#include "../data_structures/Graph.h"


/**
 * @brief Parses the locations file to build the graph.
 *
 * This function reads a CSV file containing location data (name, ID, code, and parking availability)
 * and adds the corresponding vertices to the graph.
 *
 * @param g The graph to which vertices will be added.
 * @param file The path to the locations file.
 * @return true if the file was successfully parsed, false otherwise.
 */
bool parseLocations (Graph<int> *g, const std::string &file);

/**
 * @brief Parses the distances file to add edges to the graph.
 *
 * This function reads a CSV file containing distance data (location pairs, driving time, and walking time)
 * and adds bidirectional edges to the graph.
 *
 * @param g The graph to which edges will be added.
 * @param file The path to the distances file.
 * @return true if the file was successfully parsed, false otherwise.
 */
bool parseDistances (Graph<int> *g, const std::string &file);

/**
 * @brief Parses a line of text to extract nodes to avoid.
 *
 * This function processes a string containing a list of node IDs separated by commas
 * and adds them to the `avoid_n` vector.
 *
 * @param line The input string containing the nodes to avoid.
 * @param avoid_n The vector to store the node IDs.
 * @return true if the parsing was successful, false otherwise.
 */
bool parseAvoidNodes(const std::string &line, std::vector<int> &avoid_n);

/**
 * @brief Parses a line of text to extract segments to avoid.
 *
 * This function processes a string containing pairs of node IDs representing segments
 * to avoid and adds them to the `avoid_seg` vector.
 *
 * @param line The input string containing the segments to avoid.
 * @param avoid_seg The vector to store the segment pairs.
 * @return true if the parsing was successful, false otherwise.
 */
bool parseAvoidSegments(const std::string &line, std::vector<std::pair<int, int>> &avoid_seg);

/**
 * @brief Parses the input file to extract route computation parameters.
 *
 * This function reads an input file containing mode, source, destination, constraints,
 * and other parameters, and stores them in the provided variables.
 *
 * @param f The input file stream.
 * @param driving A boolean indicating whether the mode is driving (true) or eco (false).
 * @param src The ID of the source vertex.
 * @param dest The ID of the destination vertex.
 * @param maxWalkTime The maximum allowed walking time (for eco mode).
 * @param avoid_n A vector to store nodes to avoid.
 * @param avoid_seg A vector to store segments to avoid.
 * @param inc_n The ID of a node to include in the route (if specified).
 * @param numVert The total number of vertices in the graph.
 * @return true if the input file was successfully parsed, false otherwise.
 */
bool parseInput(std::ifstream &f, bool &driving, int &src, int &dest, int &maxWalkTime, std::vector<int> &avoid_n, std::vector<std::pair<int,int>> &avoid_seg, int &inc_n, const int numVert);

#endif //PARSING_H
