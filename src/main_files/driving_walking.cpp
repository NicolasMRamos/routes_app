#include <iostream>
#include <fstream>

#include "../headers/driving_walking.h"
#include "../headers/algorithms.h"

using namespace std;

// Eco-mode ------------------------------------------------------------------------------------------------------------

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
 * @param batch Specifies if the function has been called on batch mode or manual mode.
 */
void eco_mode(Graph<int> *g, const int &origin, const int &dest, const double &max_walk_time, const vector<int> &avoid_nodes, const vector<pair<int,int>> &avoid_edges, bool batch) {

    std::ofstream fout;
    std::ostream& out = batch ? (fout.open("output.txt"), fout) : cout;

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

    driving_dijkstra(g, origin);

    for (auto v:g->getVertexSet()) {
        v->setWalking(true);
    }

    walking_dijkstra(g, dest);

    double best_time = INF;
    Vertex<int> *park = nullptr;
    bool no_path = true;

    for (auto v:g->getVertexSet()) {
        if (v->hasParking() && v->getDistWalk() <= max_walk_time && !v->isRestricted() && v->getID() != origin) {
            if (v->getDistWalk() + v->getDistDrive() < best_time || (v->getDistWalk() + v->getDistDrive() == best_time && v->getDistWalk() > park->getDistWalk())) {
                best_time = v->getDistWalk() + v->getDistDrive();
                park = v;
            }
        }

        if (v->hasParking() && v->getPathWalk() != nullptr && v->getPathDrive() != nullptr)
            no_path = false;
    }

    out << "Source:" << origin << '\n';
    out << "Destination:" << dest << '\n';

    if (park == nullptr) {
        out << "DrivingRoute:none\n";
        out << "ParkingNode:none\n";
        out << "WalkingRoute:none\n";
        out << "TotalTime:\n";

        if (no_path)
            out << "Message:No path from origin to destination.\n";
        else
            out << "Message:Exceeded Max Walking Time of " << max_walk_time << " minutes.\n";
    }

    else {
        auto pathDrive = getPathDrive(g, origin, park->getID());
        auto pathWalk = getPathWalk(g, dest, park->getID());

        out << "DrivingRoute:" << pathDrive[0]->getID();

        for (int i = 1; i < pathDrive.size(); i++)
            out << ',' << pathDrive[i]->getID();

        out << '(' << pathDrive.back()->getDistDrive() << ")\n";

        out << "ParkingNode:" << pathWalk.back()->getID() << '\n';

        out << "WalkingRoute:" << pathWalk.back()->getID();

        for (double i = pathWalk.size() - 2; i >= 0; i--)
            out << ',' << pathWalk[i]->getID();

        out << '(' << pathWalk.back()->getDistWalk() << ")\n";

        out << "TotalTime:" << pathDrive.back()->getDistDrive() + pathWalk.back()->getDistWalk() << '\n';
    }

    for (auto v:g->getVertexSet()) {
        v->setWalking(false);
    }
}

// Eco-mode with approximation -----------------------------------------------------------------------------------------

/**
 * @brief Computes the most eco-friendly route from an origin to a destination, combining driving and walking,
 * if the constraints can't be respected.
 *
 * This function calculates the optimal route by driving to a parking node and then walking to the destination,
 * without respect to the constraints (such as maximum walking time and avoiding specific nodes and edges).
 *
 * @param g The graph containing the vertices and edges.
 * @param origin The ID of the origin vertex.
 * @param dest The ID of the destination vertex.
 * @param max_walk_time The maximum allowed walking time in minutes.
 * @param avoid_nodes A vector of node IDs to avoid in the route.
 * @param avoid_edges A vector of pairs representing edges to avoid in the route.
 * @param batch Specifies if the function has been called on batch mode or manual mode.
 */
void eco_mode_approximate(Graph<int> *g, const int &origin, const int &dest, const double &max_walk_time, const vector<int> &avoid_nodes, const vector<pair<int,int>> &avoid_edges, bool batch) {

    std::ofstream fout;
    std::ostream& out = batch ? (fout.open("output.txt"), fout) : cout;

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

    driving_dijkstra(g, origin);

    for (auto v:g->getVertexSet()) {
        v->setWalking(true);
    }

    walking_dijkstra(g, dest);

    for (auto v:g->getVertexSet()) {
        v->setWalking(false);
    }

    double best_time = INF;
    Vertex<int> *park = nullptr;
    bool no_path = true;

    for (auto v:g->getVertexSet()) {
        if (v->hasParking() && v->getDistWalk() <= max_walk_time && !v->isRestricted() && v->getID() != origin && v->getID() != dest) {
            if (v->getDistWalk() + v->getDistDrive() < best_time || (best_time != INF && v->getDistWalk() + v->getDistDrive() == best_time && v->getDistWalk() > park->getDistWalk())) {
                best_time = v->getDistWalk() + v->getDistDrive();
                park = v;
            }
        }

        if (v->hasParking() && v->getPathWalk() != nullptr && v->getPathDrive() != nullptr)
            no_path = false;
    }

    out << "Source:" << origin << '\n';
    out << "Destination:" << dest << '\n';

    if (park == nullptr) {
        if(!no_path) {
            for (auto v:g->getVertexSet()) {
                if (v->hasParking() && !v->isRestricted() && v->getID() != origin && v->getID() != dest) {
                    if (v->getDistWalk() + v->getDistDrive() < best_time || (best_time != INF && v->getDistWalk() + v->getDistDrive() == best_time && v->getDistWalk() > park->getDistWalk())) {
                        best_time = v->getDistWalk() + v->getDistDrive();
                        park = v;
                    }
                }
            }

            auto pathDrive = getPathDrive(g, origin, park->getID());
            auto pathWalk = getPathWalk(g, dest, park->getID());

            out << "DrivingRoute1:" << pathDrive[0]->getID();

            for (int i = 1; i < pathDrive.size(); i++)
                out << ',' << pathDrive[i]->getID();

            out << '(' << pathDrive.back()->getDistDrive() << ")\n";

            out << "ParkingNode1:" << pathWalk.back()->getID() << '\n';

            out << "WalkingRoute1:" << pathWalk.back()->getID();

            for (double i = pathWalk.size() - 2; i >= 0; i--)
                out << ',' << pathWalk[i]->getID();

            out << '(' << pathWalk.back()->getDistWalk() << ")\n";

            out << "TotalTime1:" << pathDrive.back()->getDistDrive() + pathWalk.back()->getDistWalk() << '\n';

            double best_time2 = INF;
            double best_drive;
            double best_walk;
            Vertex<int> *park2 = nullptr;
            bool no_path2 = true;
            vector<Vertex<int>*> pathDrive2 = {};
            vector<Vertex<int>*> pathWalk2 = {};

            for (int i = 0; i < pathDrive.size() - 1; i++) {
                for (auto e:pathDrive[i]->getAdj()) {
                    if (e->getDest() == pathDrive[i+1]) {
                        e->setRestricted(true);

                        driving_dijkstra(g, origin);

                        e->setRestricted(false);

                        for (auto v:g->getVertexSet()) {
                            v->setWalking(true);
                        }

                        walking_dijkstra(g, dest);

                        for (auto v:g->getVertexSet()) {
                            v->setWalking(false);
                        }

                        for (auto v:g->getVertexSet()) {
                            if (v->hasParking() && !v->isRestricted() && v->getID() != origin && v->getID() != dest) {
                                if (v->getDistWalk() + v->getDistDrive() < best_time2 || (best_time2 != INF && v->getDistWalk() + v->getDistDrive() == best_time2 && v->getDistWalk() > park->getDistWalk())) {
                                    best_time2 = v->getDistWalk() + v->getDistDrive();
                                    best_drive = v->getDistDrive();
                                    best_walk = v->getDistWalk();
                                    park2 = v;
                                    pathDrive2 = getPathDrive(g, origin, park2->getID());
                                    pathWalk2 = getPathWalk(g, dest, park2->getID());
                                }
                            }

                            if (v->hasParking() && v->getPathWalk() != nullptr && v->getPathDrive() != nullptr)
                                no_path2 = false;
                        }
                        break;
                    }
                }
            }

            for (int i = 0; i < pathWalk.size() - 1; i++) {
                for (auto e:pathWalk[i]->getAdj()) {
                    if (e->getDest() == pathWalk[i+1]) {
                        driving_dijkstra(g, origin);
                        for (auto v:g->getVertexSet()) {
                            v->setWalking(true);
                        }

                        e->setRestricted(true);

                        walking_dijkstra(g, dest);

                        e->setRestricted(false);

                        for (auto v:g->getVertexSet()) {
                            v->setWalking(false);
                        }

                        for (auto v:g->getVertexSet()) {
                            if (v->hasParking() && !v->isRestricted() && v->getID() != origin && v->getID() != dest) {
                                if (v->getDistWalk() + v->getDistDrive() < best_time2 || (best_time2 != INF && v->getDistWalk() + v->getDistDrive() == best_time2 && v->getDistWalk() > park->getDistWalk())) {
                                    best_time2 = v->getDistWalk() + v->getDistDrive();
                                    best_drive = v->getDistDrive();
                                    best_walk = v->getDistWalk();
                                    park2 = v;
                                    pathDrive2 = getPathDrive(g, origin, park2->getID());
                                    pathWalk2 = getPathWalk(g, dest, park2->getID());
                                }
                            }

                            if (v->hasParking() && v->getPathWalk() != nullptr && v->getPathWalk() != nullptr)
                                no_path2 = false;
                        }
                        break;
                    }
                }
            }

            if (!no_path2) {
                out << "DrivingRoute2:" << pathDrive2[0]->getID();

                for (int j = 1; j < pathDrive2.size(); j++)
                    out << ',' << pathDrive2[j]->getID();

                out << '(' << best_drive << ")\n";

                out << "ParkingNode2:" << pathWalk2.back()->getID() << '\n';

                out << "WalkingRoute2:" << pathWalk2.back()->getID();

                for (double j = pathWalk2.size() - 2; j >= 0; j--)
                    out << ',' << pathWalk2[j]->getID();

                out << '(' << best_walk << ")\n";

                out << "TotalTime2:" << best_time2 << '\n';
            }

            else {
                out << "DrivingRoute2:none\n"
                << "ParkingNode2:none\n"
                << "WalkingRoute2:none\n"
                << "TotalTime2:\n"
                << "Message2:No path from origin to destination.\n";
            }
        }

        else {
            out << "DrivingRoute1:none\n"
            << "ParkingNode1:none\n"
            << "WalkingRoute1:none\n"
            << "TotalTime1:\n"
            << "Message1:No path from origin to destination.\n";

            out << "DrivingRoute2:none\n"
            << "ParkingNode2:none\n"
            << "WalkingRoute2:none\n"
            << "TotalTime2:\n"
            << "Message2:No path from origin to destination.\n";
        }
    }

    else {
        auto pathDrive = getPathDrive(g, origin, park->getID());
        auto pathWalk = getPathWalk(g, dest, park->getID());

        out << "DrivingRoute:" << pathDrive[0]->getID();

        for (int i = 1; i < pathDrive.size(); i++)
            out << ',' << pathDrive[i]->getID();

        out << '(' << pathDrive.back()->getDistDrive() << ")\n";

        out << "ParkingNode:" << pathWalk.back()->getID() << '\n';

        out << "WalkingRoute:" << pathWalk.back()->getID();

        for (int i = pathWalk.size() - 2; i >= 0; i--)
            out << ',' << pathWalk[i]->getID();

        out << '(' << pathWalk.back()->getDistWalk() << ")\n";

        out << "TotalTime:" << pathDrive.back()->getDistDrive() + pathWalk.back()->getDistWalk() << '\n';
    }
}