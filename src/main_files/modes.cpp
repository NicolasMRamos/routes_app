#include <iostream>

#include "../data_structures/Graph.h"
#include "../headers/parsing.h"
#include "../headers/driving.h"
#include "../headers/driving_walking.h"
#include "../headers/modes.h"

using namespace std;

// Batch Mode ----------------------------------------------------------------------------------------------------------

/**
 * @brief Executes the program in batch mode, reading input from files and computing the optimal route.
 *
 * This function reads the graph data from location and distance files, processes the input file to determine
 * the source, destination, and constraints, and then computes the optimal route based on the specified mode
 * (driving or eco mode).
 *
 * Input format: ./DA_PROJ1 <locations file> <distances file> <input file> <something>
 * Note: the last argument can be anything, only the number of arguments is relevant for this function.
 *
 * @param argv Command-line arguments containing the paths to the locations file, distances file, and input file.
 * @param argc Number of command line arguments.
 */
void batchMode(char *argv[], int argc) {
    auto *g = new Graph<int>();
    const string locations_file = argv[1];
    const string distances_file = argv[2];
    const string input_file = argv[3];

    bool approximate = false;
    if (argc > 4) {
       approximate = true;
    }

    if (!parseLocations(g, locations_file)) return;

    int numVert = g->getNumVertex();

    if (!parseDistances(g, distances_file)) return;

    int src, dest, inc_n = -1, maxWalkTime;
    bool driving, restricted = false;
    vector<int> avoid_n;
    vector<pair<int,int>> avoid_seg;

    ifstream f(input_file);

    if (!parseInput(f, driving, src, dest, maxWalkTime, avoid_n, avoid_seg, inc_n, numVert)) return;

    if (!avoid_n.empty() || !avoid_seg.empty() || inc_n != -1) {
        restricted = true;
    }

    for (int i : avoid_n) {
        if (i == inc_n || i == src || i == dest) {
            cerr << "Error: Avoided node (" << i << ") cannot be the source ("
                 << src << "), destination (" << dest << "), or include node ("
                 << inc_n << ").\n";
            return;
        }
    }

    const bool batch = true;

    if (driving && !restricted)
        driving_mode(g, src, dest, batch);
    else if (driving && restricted)
        driving_mode(g, src, dest, avoid_n, avoid_seg, inc_n, batch);
    else {
      if (approximate)
        eco_mode_approximate(g, src, dest, maxWalkTime, avoid_n, avoid_seg, batch);
      else
        eco_mode(g, src, dest, maxWalkTime, avoid_n, avoid_seg, batch);
    }
}

// Manual Mode ---------------------------------------------------------------------------------------------------------

/**
 * @brief Executes the program in manual mode, allowing the user to input data interactively.
 *
 * This function prompts the user to provide the locations and distances files, then allows them to choose
 * between driving mode and eco mode. Based on the user's inputs, it computes and displays the optimal route.
 */
void manualMode() {
    auto *g = new Graph<int>();
    string locations_file, distances_file;

    cout << "Location of the locations file: ";
    getline(cin, locations_file);

    if (!parseLocations(g, locations_file)) return;

    cout << "Location of the distances file: ";
    getline(cin, distances_file);

    if (!parseDistances(g, distances_file)) return;

    const int numVert = g->getNumVertex();

    while (true) {
        int opt;
        cout << "1. Driving Mode\n2. Eco Mode\n";
        cin >> opt;

        if (opt == 1) {

            int src, dest, inc_n, n, node, node1, node2;
            bool restricted = false;
            string yn;
            vector<int> avoid_n;
            vector<pair<int,int>> avoid_seg;

            cout << "Source: ";
            cin >> src;
            if (g->findVertex(src) == nullptr) {
                cout << "Invalid source selected.\n";
                return;
            }

            cout << "Destination: ";
            cin >> dest;
            if (g->findVertex(dest) == nullptr) {
                cout << "Invalid destination selected.\n";
                return;
            }

            cout << "Any nodes to include? (Write -1 if not; Write node id if yes)\n";
            cin >> inc_n;
            if (inc_n != -1) {
                restricted = true;
            }

            cout << "Avoid nodes? (Y/n)\n";
            cin >> yn;
            if (yn == "Y" || yn == "y") {

                restricted = true;

                cout << "How many?\n";
                cin >> n;
                if (n <= 0) {
                    cout << "Number of nodes to avoid must be higher than 0.\n";
                    return;
                }

                cout << "Which ones?\n";
                for (int i = 0; i < n; i++) {
                    cin >> node;
                    if (node == src || node == dest || node == inc_n || g->findVertex(node) == nullptr) {
                        cout << "Invalid node! Try another one.\n";
                        i--;
                        continue;
                    }
                    avoid_n.push_back(node);
                }
            }

            cout << "Avoid segments? (Y/n)\n";
            cin >> yn;
            if (yn == "Y" || yn == "y") {

                restricted = true;

                cout << "How many?\n";
                cin >> n;
                if (n <= 0) {
                    cout << "Number of segments to avoid must be higher than 0.\n";
                    return;
                }

                cout << "Which ones? (Format: node1 node2)\n";
                for (int i = 0; i < n; i++) {
                    cin >> node1 >> node2;
                    avoid_seg.emplace_back(node1,node2);
                }
            }

            if (restricted)
                driving_mode(g, src, dest, avoid_n, avoid_seg, inc_n);
            else
                driving_mode(g, src, dest);
            return;
        }

        if (opt == 2) {

            bool approximate = false;
            int src, dest, maxWalkTime, n, node, node1, node2;
            string yn;
            vector<int> avoid_n;
            vector<pair<int,int>> avoid_seg;

            cout << "Source: ";
            cin >> src;
            if (g->findVertex(src) == nullptr) {
                cout << "Invalid source selected.\n";
                return;
            }

            cout << "Destination: ";
            cin >> dest;
            if (g->findVertex(dest) == nullptr) {
                cout << "Invalid destination selected.\n";
                return;
            }

            cout << "What's the maximum time you want to spend walking?\n";
            cin >> maxWalkTime;
            if (maxWalkTime <= 0) {
                cout << "Walking time must be higher than 0.\n";
                return;
            }

            cout << "Avoid nodes? (Y/n)\n";
            cin >> yn;
            if (yn == "Y" || yn == "y") {

                cout << "How many?\n";
                cin >> n;
                if (n <= 0) {
                    cout << "Number of nodes to avoid must be higher than 0.\n";
                    return;
                }

                cout << "Which one(s)?\n";
                for (int i = 0; i < n; i++) {
                    cin >> node;
                    if (node == src || node == dest || g->findVertex(node) == nullptr) {
                        cout << "Invalid node! Try another one.\n";
                        i--;
                        continue;
                    }
                    avoid_n.push_back(node);
                }
            }

            cout << "Avoid segments? (Y/n)\n";
            cin >> yn;
            if (yn == "Y" || yn == "y") {

                cout << "How many?\n";
                cin >> n;
                if (n <= 0) {
                    cout << "Number of segments to avoid must be higher than 0.\n";
                    return;
                }

                cout << "Which one(s)? (Format: node1 node2)\n";
                for (int i = 0; i < n; i++) {
                    cin >> node1 >> node2;
                    avoid_seg.emplace_back(node1,node2);
                }
            }

            cout << "If a suitable route option with the given time requirement is not found, would you like to approximate another route ignoring the time constraints?\n";
            cin >> yn;

            if(yn == "Y" || yn == "y")
              	eco_mode_approximate(g, src, dest, maxWalkTime, avoid_n, avoid_seg);
            
	    else
            	eco_mode(g, src, dest, maxWalkTime, avoid_n, avoid_seg);
            return;
        }

        cerr << "Invalid Option.\n";
    }
}

