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
#include <vtkFloatArray.h>
#include <vtkLookupTable.h>
#include "Node.h"

using namespace std;

class Graph3D {
private:
    map<unsigned long, Node3D> nodes; // maps ids to nodes (ids are DIMACS variables)
    // NOTE: edges are stored inside the nodes

    int number_edges; // number of edges contained in graph
    float largest_node;
    double node_occurrences;
    double previous_mean;
    double absolute_variance;

    bool positioned; // whether graph has been positioned
    bool drawn; // whether graph has been drawn

    map<unsigned, unsigned> edgeDuplications; // tracks highest number of recurring edges

    vtkSmartPointer<vtkMutableUndirectedGraph> graph;
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkGraphToPolyData> graphToPolyData;
    vtkSmartPointer<vtkPolyData> vertexPolydata;
    vtkSmartPointer<vtkUnsignedCharArray> edgeColours;
    vtkSmartPointer<vtkUnsignedCharArray> vertexColours;
    vtkSmartPointer<vtkFloatArray> scales;

    map<int, int> matching;  // for graph coarsening (collapsing edges): maps node id of one end
    // of the collapsed edge to the other end's node id (or to itself)

    map<unsigned long, vector<unsigned long>> allMatching;

    map<unsigned long, unsigned long> matchMap;

    vtkColor4ub twoClauseColour;
    vtkColor4ub threePlusClauseColour;

    vtkColor4ub positiveColour;
    vtkColor4ub negativeColour;

//    vtkSmartPointer<vtkLookupTable> vertexColours;

    // <vertex, vertex, <insertionOrder, <occurrences, colour>>>
    map<pair<unsigned long, unsigned long>, pair<vtkIdType, pair<unsigned, vtkColor4ub>>> edgeColourMap;

    void online_absolute_variance(Node3D &node);

    void online_absolute_variance(float x);

    void online_absolute_variance_remove(float x);

    void add_graph_edge(unsigned long x, unsigned long y, EdgeAttribute a);

    void add_graph_edge(unsigned long x, ExtNode3D y);

    void add_edge_to_graph(
            pair<const pair<unsigned long, unsigned long>, pair<vtkIdType, pair<unsigned, vtkColor4ub>>> &edge);

    void add_edge_to_graph(pair<unsigned long, unsigned long> vertices,
                           pair<vtkIdType, pair<unsigned, vtkColor4ub>> &colour);

    void remove_graph_edge(unsigned long x, unsigned long y);

    void change_edge_duplication(unsigned &duplication, bool increment = true);

    void set_colour(pair<unsigned int, vtkColor4ub> &colour) const;

    [[nodiscard]] float get_scale(const Node3D &node) const;

    [[nodiscard]] double node_occurrences_mean() const { return node_occurrences / (float) nr_nodes(); }

    [[nodiscard]] double average_absolute_variance() const { return absolute_variance / nr_nodes(); }

    [[nodiscard]] double node_occurrences_previous_mean(float x) const {
        return (node_occurrences - x) / ((float) nr_nodes() - 1);
    }

public:
    Graph3D() : number_edges(0),
                largest_node(0),
                node_occurrences(0),
                previous_mean(0),
                absolute_variance(0),
                positioned(false),
                drawn(false),
                edgeDuplications({}),
                graph(vtkSmartPointer<vtkMutableUndirectedGraph>::New()),
                points(vtkSmartPointer<vtkPoints>::New()),
                graphToPolyData(vtkSmartPointer<vtkGraphToPolyData>::New()),
                vertexPolydata(vtkSmartPointer<vtkPolyData>::New()),
                edgeColours(vtkSmartPointer<vtkUnsignedCharArray>::New()),
                vertexColours(vtkSmartPointer<vtkUnsignedCharArray>::New()),
                scales(vtkSmartPointer<vtkFloatArray>::New()),
                allMatching({}),
                twoClauseColour(),
                threePlusClauseColour(),
                positiveColour(),
                negativeColour() {
        auto namedColours = vtkSmartPointer<vtkNamedColors>::New();
        twoClauseColour = namedColours->GetColor4ub("sky_blue_deep"); // Tomato
        threePlusClauseColour = namedColours->GetColor4ub("Cyan"); // Mint
        positiveColour = namedColours->GetColor4ub("Green");
        negativeColour = namedColours->GetColor4ub("Red");

        /*vertexColours = vtkSmartPointer<vtkLookupTable>::New();
        vertexColours->SetNumberOfTableValues(2);
        vertexColours->SetTableValue(0, namedColours->GetColor4d("Green").GetData());
        vertexColours->SetTableValue(1, namedColours->GetColor4d("Red").GetData());
        vertexColours->Build();*/
    } // constructs empty graph
    ~Graph3D() = default;

    void calculate_absolute_variance();

    // modifiers
    void add_node(const Node3D &n);          // node 'n' is copied into graph

    void insert_edge(unsigned long x, unsigned long y, EdgeAttribute a = NT_3_PLUS_CLAUSE);
    // add edge between nodes with ids x and y (if not
    // already present)

    void add_graph_edge_from_ids(unsigned long x, unsigned long y, EdgeAttribute a);

    void add_graph_edges_from_clause(vector<long> clause);

    void remove_graph_edge_from_ids(unsigned long x, unsigned long y);

    void remove_graph_edges_from_clause(vector<long> clause);

    void increase_variable_activity(unsigned long i);

    void assign_variable_truth_value(unsigned long i, bool value, bool undef = false);

    pair<vector<vector<long>>, unsigned int> build_from_cnf(istream &is); // read file in DIMACS format, build graph

    // observables
    [[nodiscard]] int nr_nodes() const { return nodes.size(); }

    [[nodiscard]] int nr_edges() const { return number_edges; }

    [[nodiscard]] bool get_positioned() const { return positioned; }

    [[nodiscard]] bool get_drawn() const { return drawn; }

    [[nodiscard]] float highestEdgeDuplication() const { return (float) edgeDuplications.end()->first; }

    [[nodiscard]] const vtkSmartPointer<vtkGraphToPolyData> &getGraphToPolyData() const { return graphToPolyData; }

    [[nodiscard]] const vtkSmartPointer<vtkPolyData> &getVertexPolydata() const { return vertexPolydata; }

    [[nodiscard]] unsigned int no_of_occurrences() const { return (unsigned int) node_occurrences; }

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
    void drawPolyData(double k, bool draw_edges, bool draw_only_2clauses, bool adaptive_node_size);

    void reColour();

    void reScale();

    // I/O
    friend ostream &operator<<(ostream &os, const Graph3D &g);

    void set_all_matching(map<unsigned long, vector<unsigned long>> prev);

    void set_match_map(map<unsigned long, unsigned long> prev);

    void add_match(unsigned long node, unsigned long matched);
};

#endif
