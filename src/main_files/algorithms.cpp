#include <iostream>

#include "../data_structures/Graph.h"
#include "../data_structures/MutablePriorityQueue.h"
#include "../headers/algorithms.h"


// Edge Relaxation  ----------------------------------------------------------------------------------------------------
/**
 * @brief Relaxes an edge for driving, updating the destination vertex's driving distance if a shorter path is found.
 *
 * @param edge The edge to relax.
 * @return true if the relaxation was successful (i.e., a shorter path was found), false otherwise.
 */
bool driving_relax(Edge<int> *edge) { // d[u] + w(u,v) < d[v]
    auto u = edge->getOrig();
    auto v = edge->getDest();
    if (u->getDistDrive() + edge->getWeightDrive() < v->getDistDrive() && !v->isRestricted()) {
        v->setDistDrive(u->getDistDrive() + edge->getWeightDrive());
        v->setPathDrive(edge);
        return true;
    }
    return false;
}

/**
 * @brief Relaxes an edge for walking, updating the destination vertex's walking distance if a shorter path is found.
 *
 * @param edge The edge to relax.
 * @return true if the relaxation was successful (i.e., a shorter path was found), false otherwise.
 */
bool walking_relax(Edge<int> *edge) { // d[u] + w(u,v) < d[v]
    auto u = edge->getOrig();
    auto v = edge->getDest();
    if (u->getDistWalk() + edge->getWeightWalk() < v->getDistWalk() && !v->isRestricted()) {
        v->setDistWalk(u->getDistWalk() + edge->getWeightWalk());
        v->setPathWalk(edge);
        return true;
    }
    return false;
}

// Dijkstra for Driving ------------------------------------------------------------------------------------------------

/**
 * @brief Executes Dijkstra's algorithm for driving, computing the shortest driving paths from a given origin vertex.
 *
 * @param g The graph on which to execute the algorithm.
 * @param origin The ID of the origin vertex.
 */
void driving_dijkstra(Graph<int> *g, const int &origin) {

    if (g->getVertexSet().empty()) {
        return;
    }

    MutablePriorityQueue<Vertex<int>> pq;

    for (auto s:g->getVertexSet()) {
        s->setDistDrive(INF);
        s->setPathDrive(nullptr);
        pq.insert(s);
    }

    auto temp = g->findVertex(origin);
    temp->setDistDrive(0);
    pq.decreaseKey(temp);

    while (!pq.empty()) {
        auto v = pq.extractMin();

        if (v->isRestricted()) continue;

        for (auto e:v->getAdj()) {

            if (e->isRestricted()) continue;

            if (driving_relax(e)) pq.decreaseKey(e->getDest());

        }
    }
}

// Dijkstra for Walking ------------------------------------------------------------------------------------------------

/**
 * @brief Executes Dijkstra's algorithm for walking, computing the shortest walking paths from a given origin vertex.
 *
 * @param g The graph on which to execute the algorithm.
 * @param origin The ID of the origin vertex.
 */
void walking_dijkstra(Graph<int> *g, const int &origin) {

    if (g->getVertexSet().empty()) {
        return;
    }

    MutablePriorityQueue<Vertex<int>> pq;

    for (auto s:g->getVertexSet()) {
        s->setDistWalk(INF);
        s->setPathWalk(nullptr);
        pq.insert(s);
    }

    auto temp = g->findVertex(origin);
    temp->setDistWalk(0);
    pq.decreaseKey(temp);

    while (!pq.empty()) {
        auto v = pq.extractMin();

        if (v->isRestricted()) continue;

        for (auto e:v->getAdj()) {

            if (e->isRestricted()) continue;

            if (walking_relax(e)) pq.decreaseKey(e->getDest());

        }
    }
}

// Auxiliary Function to set up for Dijkstra execution -----------------------------------------------------------------

/**
 * @brief Resets the graph's vertices and edges to their initial state, allowing Dijkstra's algorithm to be re-run.
 *
 * @param g The graph to reset.
 */
void setup(Graph<int> *g) {
    for (auto s:g->getVertexSet()) {
        s->setRestricted(false);   //To allow rerunning the driving_dijkstra and find 2 different paths
        s->setPathDrive(nullptr);
        s->setPathWalk(nullptr);
        s->setDistDrive(INF);
        s->setDistWalk(INF);

        for (auto e:s->getAdj()) {
            e->setRestricted(false);
        }
    }
}

// Auxiliary function to get Shortest Path -----------------------------------------------------------------------------

/**
 * @brief Retrieves the shortest driving path from the origin to the destination vertex.
 *
 * @param g The graph containing the vertices.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @return A vector of vertices representing the shortest driving path.
 */
std::vector<Vertex<int>*> getPathDrive(Graph<int> *g, const int &origin, const int &dest) {
    std::vector<Vertex<int>*> res;
    auto cur_node = g->findVertex(dest);
    res.push_back(cur_node);

    while (cur_node->getPathDrive() != nullptr) {
        cur_node = cur_node->getPathDrive()->getOrig();
        res.insert(res.begin(), cur_node);
    }

    return res;
}

/**
 * @brief Retrieves the shortest walking path from the origin to the destination vertex.
 *
 * @param g The graph containing the vertices.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @return A vector of vertices representing the shortest walking path.
 */
std::vector<Vertex<int>*> getPathWalk(Graph<int> *g, const int &origin, const int &dest) {
    std::vector<Vertex<int>*> res;
    auto cur_node = g->findVertex(dest);
    res.push_back(cur_node);

    while (cur_node->getPathWalk() != nullptr) {
        cur_node = cur_node->getPathWalk()->getOrig();
        res.insert(res.begin(), cur_node);
    }

    return res;
}