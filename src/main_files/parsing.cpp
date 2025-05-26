#include <sstream>
#include <iostream>

#include "../headers/parsing.h"

using namespace std;

// Locations Parsing ---------------------------------------------------------------------------------------------------

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
bool parseLocations (Graph<int> *g, const string &file) {
    string line;
    string check = "Location,Id,Code,Parking";
    ifstream f;
    f.open(file);

    if (!f.is_open()) {
        cerr << "No file found.\n";
        return false;
    }

    getline(f, line);

    if (line.find(check) == string::npos) {
        cerr << "Invalid locations file.\n";
        return false;
    }

    while (getline(f, line)) {

        if (line.find(',') == string::npos) continue;

        size_t separator = line.find(',');
        string location = line.substr(0, separator);
        line.erase(0, separator + 1);

        separator = line.find(',');
        int id = stoi(line.substr(0, separator));
        line.erase(0, separator + 1);

        separator = line.find(',');
        string code = line.substr(0, separator);
        line.erase(0, separator + 1);

        try {
            bool parking = stoi(line);
            g->addVertex(location, id, code, parking);
        } catch (...) {
            cerr << "Invalid values detected.\n";
            return false;
        }
    }
    f.close();
    return true;
}

// Distances Parsing ---------------------------------------------------------------------------------------------------

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
bool parseDistances (Graph<int> *g, const string &file) {
    string line;
    string check = "Location1,Location2,Driving,Walking";
    ifstream f(file);
    getline(f,line); // header line, not needed

    if (!f.is_open()) {
        cerr << "No file found.\n";
        return false;
    }

    if (line.find(check) == string::npos) {
        cerr << "Invalid distances file.\n";
        return false;
    }

    while (getline(f, line)) {

        if (line.find(',') == string::npos) continue;

        string code1, code2, checker;
        double driving, walking;

        size_t separator = line.find(',');
        code1 = line.substr(0, separator);
        line.erase(0, separator + 1);

        separator = line.find(',');
        code2 = line.substr(0, separator);
        line.erase(0, separator + 1);

        auto v1 = g->findVertexCode(code1);
        auto v2 = g->findVertexCode(code2);

        separator = line.find(',');
        checker = line.substr(0, separator);
        line.erase(0, separator + 1);

        if (checker == "X")
            driving = INF;
        else
            try {
                driving = stoi(checker);
            } catch (...) {
                cerr << "Invalid values detected.\n";
                return false;
            }

        checker = line;

        if (checker == "X")
            walking = INF;
        else
            try {
                walking = stoi(checker);
            } catch (...) {
                cerr << "Invalid values detected.\n";
                return false;
            }

        g->addBidirectionalEdge(v1->getID(), v2->getID(), driving, walking);
    }

    return true;
}

// Auxiliary functions for parsing -------------------------------------------------------------------------------------

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
bool parseAvoidNodes(const string &line, vector<int> &avoid_n) {

    istringstream ss(line.substr(11));
    string id;

    while (getline(ss, id, ',')) {
        try {
            avoid_n.push_back(stoi(id));
        } catch (...) {

        }
    }
    return true;
}

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
bool parseAvoidSegments(const string &line, vector<pair<int, int>> &avoid_seg) {

    istringstream ss(line.substr(14));
    string segment;

    while (getline(ss, segment, ')')) { // read until ')'

        segment.erase(0, segment.find_first_not_of(" \t\r\n")); // trim spaces

        if (segment.empty()) continue;

        size_t start = segment.find('('); // find first pair
        if (start == string::npos) {
            cerr << "Incorrect format.\n";
            return false;
        }

        segment = segment.substr(start + 1); // remove '('
        size_t comma = segment.find(',');
        if (comma == string::npos) {
            cerr << "Incorrect format.\n";
            return false;
        }

        try {
            int first = stoi(segment.substr(0, comma));
            int second = stoi(segment.substr(comma + 1));
            avoid_seg.emplace_back(first, second);
        } catch (...) {
            cerr << "Invalid format.\n";
            return false;
        }
    }
    return true;
}

// Input Parsing -------------------------------------------------------------------------------------------------------

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
bool parseInput(ifstream &f, bool &driving, int &src, int &dest, int &maxWalkTime, vector<int> &avoid_n, vector<pair<int,int>> &avoid_seg, int &inc_n, const int numVert) {

    string line;
    bool hasMode = false, hasSrc = false, hasDest = false;

    while (getline(f, line)) {

        if (line.rfind("Mode:", 0) == 0) { // line starts with "Mode:"

            string mode = line.substr(5);
            mode.erase(0, mode.find_first_not_of(" \t\r\n"));
            mode.erase(mode.find_last_not_of(" \t\r\n") + 1);
            // erase extra characters at the start and at the end of the string

            if (mode == "driving") {
                driving = true;
                hasMode = true;
            } else if (mode == "driving-walking") {
                driving = false;
                hasMode = true;
            } else {
                cerr << "Invalid Mode.\n";
                return false;
            }

        } else if (line.rfind("Source:", 0) == 0) { // line starts with "Source:"

            try {
                src = stoi(line.substr(7));
                hasSrc = true;
            } catch (...) {
                cerr << "Invalid Source Input.\n";
                return false;
            }

        } else if (line.rfind("Destination:", 0) == 0) { // line starts with "Destination:"

            try {
                dest = stoi(line.substr(12));
                hasDest = true;
            } catch (...) {
                cerr << "Invalid Destination Input.\n";
                return false;
            }

        } else if (line.rfind("MaxWalkTime:", 0) == 0) { // line starts with "MaxWalkTime:"

            try {
                maxWalkTime = stoi(line.substr(12));
                if (maxWalkTime <= 0) {
                    cerr << "Max walking time must be higher than 0.\n";
                    return false;
                }
            } catch (...) {
                cerr << "Max walking time must be specified.\n";
                return false;
            }

        } else if (line.rfind("AvoidNodes:", 0) == 0) { // line starts with "AvoidNodes:"

            parseAvoidNodes(line, avoid_n);

        } else if (line.rfind("AvoidSegments:", 0) == 0) { // line starts with "AvoidSegments:"

            if (!parseAvoidSegments(line, avoid_seg)) return false;

        } else if (line.rfind("IncludeNode:", 0) == 0) { // line starts with "IncludeNode:"

            try {
                inc_n = stoi(line.substr(12));
                if (inc_n < -1 || inc_n >= numVert) {
                    cerr << "Node to include must exist in the graph.\n";
                    return false;
                }
            } catch (...) {

            }

        }
    }

    if (!hasMode || !hasSrc || !hasDest) { // check for any missing components
        cerr << "Input file incomplete. Missing:\n";
        if (!hasMode) cerr << "Mode.\n";
        if (!hasSrc) cerr << "Source.\n";
        if (!hasDest) cerr << "Destination.\n";
        return false;
    }

    return true;
}