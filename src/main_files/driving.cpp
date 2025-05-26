#include <iostream>
#include <fstream>
#include <utility>

#include "../data_structures/Graph.h"
#include "../headers/algorithms.h"
#include"../headers/driving.h"

using namespace std;

// Driving Computation: no restrictions --------------------------------------------------------------------------------

/**
 * @brief Computes the best and alternative driving routes from an origin to a destination without any restrictions.
 *
 * @param g The graph containing the vertices and edges.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @param batch If true, outputs the results to a file; otherwise, outputs to the console.
 */
void driving_mode(Graph<int> *g, const int &origin, const int &dest, const bool batch) {

    std::ofstream fout;
    std::ostream& out = batch ? (fout.open("output.txt"), fout) : std::cout;

    out << "Source:" << origin << '\n';
    out << "Destination:" << dest << '\n';

    setup(g);

    driving_dijkstra(g, origin);
    auto path = getPathDrive(g, origin, dest);

    if (path.empty() || path[0]->getID() == dest) {
        out << "BestDrivingRoute:none\n";
        return;
    }

    out << "BestDrivingRoute:" << path[0]->getID();

    for (int i = 1; i < path.size(); i++) {
        out << ',' << path[i]->getID();
        path[i]->setRestricted(true);
    }

    path.back()->setRestricted(false);
    out << '(' << path.back()->getDistDrive() << ')' << '\n';

    driving_dijkstra(g, origin);
    path = getPathDrive(g, origin, dest);

    if (path.empty() || path[0]->getID() == dest) {
        out << "AlternativeDrivingRoute:none\n";
        return;
    }

    out << "AlternativeDrivingRoute:" << path[0]->getID();

    for (int i = 1; i < path.size(); i++) {
        out << ',' << path[i]->getID();
        path[i]->setRestricted(true);
    }

    path.back()->setRestricted(false);
    out << '(' << path.back()->getDistDrive() << ')' << '\n';
}

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
void driving_mode(Graph<int> *g, const int &origin, const int &dest, const vector<int> &avoid_nodes, const vector<pair<int,int>> &avoid_edges, const int &include_node, const bool batch) {

    std::ofstream fout;
    std::ostream& out = batch ? (fout.open("output.txt"), fout) : std::cout;

    out << "Source:" << origin << '\n';
    out << "Destination:" << dest << '\n';

    setup(g);

    for (int an:avoid_nodes) {
        auto v = g->findVertex(an);
        v->setRestricted(true);
    }

    for (auto p:avoid_edges) {
        auto v = g->findVertex(p.first);
        for (auto e:v->getAdj()) {
            if (e->getDest()->getID() == p.second) {
                e->setRestricted(true);
                e->getReverse()->setRestricted(true);
            }
        }
    }

    if (include_node != -1) {
        vector<int> nodes = {origin, include_node, dest};
        string res = "RestrictedDrivingRoute:";
        double dist = 0;

        for (int i = 0; i < 2; i++) {
            driving_dijkstra(g, nodes[i]);
            auto path = getPathDrive(g, nodes[i], nodes[i+1]);

            if (path.empty() || path[0]->getID() == dest) {
                out << "RestrictedDrivingRoute:none\n";
                return;
            }

            if (path[0]->getID() != include_node)
                res += to_string(path[0]->getID());

            for (int j = 1; j < path.size(); j++)
                res += "," + to_string(path[j]->getID());


            dist += path.back()->getDistDrive();
        }

        out << res << '(' << dist << ')' << '\n';
    }

    else {

        driving_dijkstra(g, origin);
        auto path = getPathDrive(g, origin, dest);

        if (path.empty() || path[0]->getID() == dest) {
            out << "RestrictedDrivingRoute:none\n";
            return;
        }

        out << "RestrictedDrivingRoute:" << path[0]->getID();

        for (int i = 1; i < path.size(); i++) {
            out << ',' << path[i]->getID();
        }

        out << '(' << path.back()->getDistDrive() << ')' << '\n';
    }
}
