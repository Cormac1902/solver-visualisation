// -*- C++ -*-

#ifndef _VIS_3D_GRAPH
#define _VIS_3D_GRAPH

#include <map>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkGraph.h>
#include <vtkGraphToPolyData.h>
#include <vtkColor.h>
#include <vtkNamedColors.h>
#include <vtkUndirectedGraph.h>
#include <vtkMutableUndirectedGraph.h>
#include "Node.h"

using namespace std;

class Graph3D {
private:
    map<unsigned long, Node3D> nodes; // maps ids to nodes (ids are DIMACS variables)
    // NOTE: edges are stored inside the nodes

    int number_edges; // number of edges contained in graph

    bool positioned; // whether graph has been positioned
    bool drawn; // whether graph has been drawn

    float highestEdgeDuplication; // tracks highest number of recurring edges

    vtkMutableUndirectedGraph* graph{};
    vtkUnsignedCharArray* edgeColours{};

    map<int, int> matching;  // for graph coarsening (collapsing edges): maps node id of one end
    // of the collapsed edge to the other end's node id (or to itself)

    map<unsigned long, vector<unsigned long>> allMatching;

    map<unsigned long, unsigned long> matchMap;

    vtkColor4ub twoClauseColour;
    vtkColor4ub threePlusClauseColour;

    // <vertex, vertex, <insertionOrder, <occurrences, colour>>>
    map<pair<unsigned long, unsigned long>, pair<unsigned long, pair<unsigned, vtkColor4ub>>> edgeColourMap;

    void add_graph_edge(unsigned long x, unsigned long y, EdgeAttribute a);

    void add_graph_edge(unsigned long x, ExtNode3D y);

    void add_edge_to_graph(pair<const pair<unsigned long, unsigned long>, pair<unsigned long, pair<unsigned, vtkColor4ub>>>& edge);

    void add_edge_to_graph(pair<unsigned long, unsigned long> vertices, pair<unsigned long, pair<unsigned, vtkColor4ub>>& colour);

    void set_colour(pair<unsigned int, vtkColor4ub>& colour);

public:
    Graph3D() : number_edges(0),
                positioned(false),
                drawn(false),
                highestEdgeDuplication(0),
                allMatching({}) {
        vtkSmartPointer<vtkNamedColors> namedColours = vtkSmartPointer<vtkNamedColors>::New();
        twoClauseColour = namedColours->GetColor4ub("Tomato");
        threePlusClauseColour = namedColours->GetColor4ub("Mint");
    } // constructs empty graph
    ~Graph3D() = default;

    // modifiers
    void add_node(const Node3D &n);          // node 'n' is copied into graph

    void insert_edge(unsigned long x, unsigned long y, EdgeAttribute a = NT_3_PLUS_CLAUSE);
    // add edge between nodes with ids x and y (if not
    // already present)

    void add_graph_edge_from_ids(unsigned long x, unsigned long y, EdgeAttribute a);

    void add_graph_edges_from_clause(vector<long> clause);

    void build_from_cnf(istream &is); // read file in DIMACS format, build graph

    // observables
    int nr_nodes() { return nodes.size(); }

    int nr_edges() const { return number_edges; }

    bool get_positioned() const { return positioned; }

    float getHighestEdgeDuplication() const { return highestEdgeDuplication; }

    // misc
    int independent_components(vector<int> *one_of_each_comp);
    // computes the number of independent components of the graph.
    // From each component the id of one node is returned.

    Graph3D *coarsen();
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

    void reColour();

    // I/O
    friend ostream &operator<<(ostream &os, const Graph3D &g);

    void set_all_matching(map<unsigned long, vector<unsigned long>> prev);

    void set_match_map(map<unsigned long, unsigned long> prev);

    void add_match(unsigned long node, unsigned long matched);
};

#endif
