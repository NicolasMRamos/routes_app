// Original code by Gonçalo Leão
// Updated by DA 2024/2025 Team
// Adapted by T01_G07 (2025)

/*
    Adaptations:
    - changed visited and respective methods to restricted, just to better show the nature of the problem
    - Added two types of distances (dist_walk and dist_drive)
    - Added restricted attribute to Edge
 */

#ifndef DA_TP_CLASSES_GRAPH
#define DA_TP_CLASSES_GRAPH

#include <vector>
#include <limits>
#include <string>
#include "../data_structures/MutablePriorityQueue.h" // not needed for now

template <class T>
class Edge;

#define INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

template <class T>
class Vertex {
public:
    Vertex(const std::string &location, int id, const std::string &code, bool parking);
    bool operator<(Vertex<T> & vertex) const; // // required by MutablePriorityQueue

    std::string getCode() const;
    std::string getLocation() const;
    int getID() const;
    bool hasParking() const;
    std::vector<Edge<T> *> getAdj() const;
    bool isRestricted() const;
    double getDistWalk() const;
    double getDistDrive() const;
    Edge<T> *getPathDrive() const;
    Edge<T> *getPathWalk() const;
    std::vector<Edge<T> *> getIncoming() const;

    void setRestricted(bool restricted);

    void setDistWalk(double dist_walk);
    void setDistDrive(double dist_drive);
    void setPathDrive(Edge<T> *path);
    void setPathWalk(Edge<T> *path);
    Edge<T> * addEdge(Vertex<T> *dest, double wd, double ww);
    bool removeEdge(T in);
    void removeOutgoingEdges();

    bool getWalking();
    void setWalking(bool set);

    friend class MutablePriorityQueue<Vertex>;
protected:

    const std::string location_name;
    const int id;
    const std::string code;
    const bool parking;

    bool walking = false;    //Support for eco mode

    std::vector<Edge<T> *> adj;  // outgoing edges

    // auxiliary fields
    bool restricted = false; // used to implement restrictions on graph travel (replaced visited)
    double dist_walk = 0;
    double dist_drive = 0;
    Edge<T> *path_drive = nullptr;
    Edge<T> *path_walk = nullptr;

    std::vector<Edge<T> *> incoming; // incoming edges

    int queueIndex = 0; 		// required by MutablePriorityQueue and UFDS

    void deleteEdge(Edge<T> *edge);
};

/********************** Edge  ****************************/

template <class T>
class Edge {
public:
    Edge(Vertex<T> *orig, Vertex<T> *dest, double wd, double ww);

    Vertex<T> * getDest() const;
    double getWeightDrive() const;
    double getWeightWalk() const;
    bool isSelected() const;
    bool isRestricted() const;
    Vertex<T> * getOrig() const;
    Edge<T> *getReverse() const;
    void setRestricted(bool restricted);
    void setSelected(bool selected);
    void setReverse(Edge<T> *reverse);

protected:
    Vertex<T> * dest; // destination vertex
    double weight_drive; // edge driving weight
    double weight_walk; //edge walking weight
    bool restricted = false;

    // auxiliary fields
    bool selected = false;

    // used for bidirectional edges
    Vertex<T> *orig;
    Edge<T> *reverse = nullptr;
};

/********************** Graph  ****************************/

template <class T>
class Graph {
public:
    ~Graph();
    /*
    * Auxiliary function to find a vertex with a given the content.
    */
    Vertex<T> *findVertex(int id) const;

    Vertex<T> *findVertexCode(std::string code) const;
    /*
     *  Adds a vertex with a given content or info (in) to a graph (this).
     *  Returns true if successful, and false if a vertex with that content already exists.
     */
    bool addVertex(const std::string &location, int id, const std::string &code, bool parking);
    bool removeVertex(int id);

    /*
     * Adds an edge to a graph (this), given the contents of the source and
     * destination vertices and the edge weight (w).
     * Returns true if successful, and false if the source or destination vertex does not exist.
     */
    bool addEdge(const T &sourc, const T &dest, double wd, double ww);
    bool removeEdge(const T &source, const T &dest);
    bool addBidirectionalEdge(const T &sourc, const T &dest, double wd, double ww);

    int getNumVertex() const;

    std::vector<Vertex<T> *> getVertexSet() const;

protected:
    std::vector<Vertex<T> *> vertexSet;    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    /**
    * Auxiliary function to set the "path" field to make a spanning tree.
    */

};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);


/************************* Vertex  **************************/

template <class T>
Vertex<T>::Vertex(const std::string &location, int id, const std::string &code, bool parking): location_name(location), id(id), code(code), parking(parking) {}
/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T>
Edge<T> * Vertex<T>::addEdge(Vertex<T> *d, double wd, double ww) {
    auto newEdge = new Edge<T>(this, d, wd, ww);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}

/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Vertex<T>::removeEdge(T in) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge<T> *edge = *it;
        Vertex<T> *dest = edge->getDest();
        //To be properly adapted
        if (dest->getCode() == in) {
            it = adj.erase(it);
            deleteEdge(edge);
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        }
        else {
            it++;
        }
    }
    return removedEdge;
}

/*
 * Auxiliary function to remove an outgoing edge of a vertex.
 */
template <class T>
void Vertex<T>::removeOutgoingEdges() {
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge<T> *edge = *it;
        it = adj.erase(it);
        deleteEdge(edge);
    }
}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    //WARNING: TO BE VERIFIED
    if (walking)
        return this->dist_walk < vertex.dist_walk;

    return this->dist_drive < vertex.dist_drive;
}

template <class T>
std::string Vertex<T>::getLocation() const {
    return this->location_name;
}

template <class T>
int Vertex<T>::getID() const {
    return this->id;
}

template <class T>
std::string Vertex<T>::getCode() const {
    return this->code;
}

template <class T>
bool Vertex<T>::hasParking() const {
    return this->parking;
}

template <class T>
std::vector<Edge<T>*> Vertex<T>::getAdj() const {
    return this->adj;
}

template <class T>
bool Vertex<T>::isRestricted() const {
    return this->restricted;
}

template <class T>
double Vertex<T>::getDistWalk() const {
    return this->dist_walk;
}

template <class T>
double Vertex<T>::getDistDrive() const {
    return this->dist_drive;
}

template <class T>
Edge<T> *Vertex<T>::getPathDrive() const {
    return this->path_drive;
}

template <class T>
Edge<T> *Vertex<T>::getPathWalk() const {
    return this->path_walk;
}

template <class T>
std::vector<Edge<T> *> Vertex<T>::getIncoming() const {
    return this->incoming;
}

template <class T>
void Vertex<T>::setRestricted(bool restricted) {
    this->restricted = restricted;
}

template <class T>
void Vertex<T>::setDistWalk(double dist_walk) {
    this->dist_walk = dist_walk;
}

template <class T>
void Vertex<T>::setDistDrive(double dist_drive) {
    this->dist_drive = dist_drive;
}


template <class T>
void Vertex<T>::setPathDrive(Edge<T> *path) {
    this->path_drive = path;
}

template <class T>
void Vertex<T>::setPathWalk(Edge<T> *path) {
    this->path_walk = path;
}

template <class T>
void Vertex<T>::deleteEdge(Edge<T> *edge) {
    Vertex<T> *dest = edge->getDest();
    // Remove the corresponding edge from the incoming list
    auto it = dest->incoming.begin();
    while (it != dest->incoming.end()) {
        if ((*it)->getOrig()->getCode() == code) {
            it = dest->incoming.erase(it);
        }
        else {
            it++;
        }
    }
    delete edge;
}

template <class T>
bool Vertex<T>::getWalking() {
    return walking;
}

template <class T>
void Vertex<T>::setWalking(bool set) {
    this->walking = set;
}

/********************** Edge  ****************************/

template <class T>
Edge<T>::Edge(Vertex<T> *orig, Vertex<T> *dest, double wd, double ww): orig(orig), dest(dest), weight_drive(wd), weight_walk(ww){}

template <class T>
Vertex<T> * Edge<T>::getDest() const {
    return this->dest;
}

template<class T>
void Edge<T>::setRestricted(bool restricted) {
    this->restricted = restricted;
}


template <class T>
double Edge<T>::getWeightDrive() const {
    return this->weight_drive;
}

template <class T>
double Edge<T>::getWeightWalk() const {
    return this->weight_walk;
}

template <class T>
Vertex<T> * Edge<T>::getOrig() const {
    return this->orig;
}

template <class T>
Edge<T> *Edge<T>::getReverse() const {
    return this->reverse;
}

template <class T>
bool Edge<T>::isSelected() const {
    return this->selected;
}

template <class T>
bool Edge<T>::isRestricted() const {
    return this->restricted;
}

template <class T>
void Edge<T>::setSelected(bool selected) {
    this->selected = selected;
}

template <class T>
void Edge<T>::setReverse(Edge<T> *reverse) {
    this->reverse = reverse;
}


/********************** Graph  ****************************/

template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template <class T>
std::vector<Vertex<T> *> Graph<T>::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const int id) const {
    for (auto v : vertexSet)
        if (v->getID() == id)
            return v;
    return nullptr;
}

template <class T>
Vertex<T> * Graph<T>::findVertexCode(const std::string code) const {
    for (auto v : vertexSet)
        if (v->getCode() == code)
            return v;

    return nullptr;
}

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const std::string &location, const int id, const std::string &code, const bool parking) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex<T>(location, id, code, parking));
    return true;
}

/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
template <class T>
bool Graph<T>::removeVertex(const int id) {
    for (auto it = vertexSet.begin(); it != vertexSet.end(); it++) {
        if ((*it)->getID() == id) {
            auto v = *it;
            v->removeOutgoingEdges();
            for (auto u : vertexSet) {
                u->removeEdge(v->getID());
            }
            vertexSet.erase(it);
            delete v;
            return true;
        }
    }
    return false;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double wd, double ww) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, wd, ww);
    return true;
}

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
    Vertex<T> * srcVertex = findVertex(sourc);
    if (srcVertex == nullptr) {
        return false;
    }
    return srcVertex->removeEdge(dest);
}

template <class T>
bool Graph<T>::addBidirectionalEdge(const T &sourc, const T &dest, double wd, double ww) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, wd, ww);
    auto e2 = v2->addEdge(v1, wd, ww);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

inline void deleteMatrix(int **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

inline void deleteMatrix(double **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

template <class T>
Graph<T>::~Graph() {
    deleteMatrix(distMatrix, vertexSet.size());
    deleteMatrix(pathMatrix, vertexSet.size());
}

#endif /* DA_TP_CLASSES_GRAPH */