// -*- C++ -*-

#ifndef _VIS_3D_GRAPH
#define _VIS_3D_GRAPH

#include <map>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkGraph.h>
#include <vtkGraphToPolyData.h>
#include "Node.h"

using namespace std;

class Graph3D {
private:
    map<int, Node3D> nodes; // maps ids to nodes (ids are DIMACS variables)
    // NOTE: edges are stored inside the nodes

    int number_edges; // number of edges contained in graph

    bool positioned; // whether graph has been positioned

    map<int, int> matching;  // for graph coarsening (collapsing edges): maps node id of one end
    // of the collapsed edge to the other end's node id (or to itself)

    map<long, vector<long>> allMatching;

    map<long, long> matchMap;

public:
    Graph3D() : number_edges(0), positioned(false), allMatching({}) {
    } // constructs empty graph
    ~Graph3D() = default;

    // modifiers
    void add_node(const Node3D &n);          // node 'n' is copied into graph

    void insert_edge(long x, long y, EdgeAttribute a = NT_3_PLUS_CLAUSE);
    // add edge between nodes with ids x and y (if not
    // already present)

    void build_from_cnf(istream &is); // read file in DIMACS format, build graph

    // observables
    int nr_nodes() { return nodes.size(); }

    int nr_edges() const { return number_edges; }

    bool get_positioned() const { return positioned; }

    // misc
    int independent_components(vector<int> *one_of_each_comp);
    // computes the number of independent components of the graph.
    // From each component the id of one node is returned.

    Graph3D *coarsen(int level);
    // Coarsen graph by matching adjacent nodes (reduces the number of vertices to the half).

    void init_positions_at_random();
    // Initialize node positions randomly (all coordinates in the range [-1,1]).

    void init_coarsest_graph_positions(double k);
    // Initialize positions of the 2 nodes in coarsest graph.

    void init_positions_from_graph(Graph3D *g, double k);
    // Initialize positions from coarsened graph g. Positions in g must have been
    // computed already.

    void compute_layout(double k);
    // Compute layout according to Walshaw's paper "A Multilevel Algorithm for
    // Force-Directed Graph Drawing" (JGAA 2003)

    pair<Vector3D, Vector3D> compute_extremal_points();
    // Compute bounding box (p,q) where p is the min. and q the max point.

    // Move all vertices from position p to position q = a * p + b for scalar a and vector b
    void rescale(double a, const Vector3D &b);

    // Draw graph according to current layout (VTK).
    vtkGraphToPolyData *drawPolyData(double k, bool draw_edges, bool draw_only_2clauses, bool adaptive_node_size);

    // I/O
    friend ostream &operator<<(ostream &os, const Graph3D &g);

    void set_all_matching(map<long, vector<long>> prev);

    void set_match_map(map<long, long> prev);

    void add_match(long node, long matched);
};

#endif
