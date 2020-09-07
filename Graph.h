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
#include <mutex>
#include "EdgeColour.h"

class Graph3D {
private:
    std::mutex nodes_mutex;
    std::map<unsigned long, Node3D> nodes; // maps ids to nodes (ids are DIMACS variables)
    // NOTE: edges are stored inside the nodes

    int number_edges; // number of edges contained in graph
    float largest_node;
    double node_occurrences;
    double previous_mean;
    double absolute_variance;

    bool positioned; // whether graph has been positioned
    bool drawn; // whether graph has been drawn

    std::map<unsigned, unsigned> edgeDuplications; // tracks highest number of recurring edges

    std::mutex vtk_graph_mutex;
    vtkSmartPointer<vtkMutableUndirectedGraph> graph;
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkGraphToPolyData> graphToPolyData;
    vtkSmartPointer<vtkPolyData> vertexPolydata;
    vtkSmartPointer<vtkUnsignedCharArray> edgeColours;
    vtkSmartPointer<vtkUnsignedCharArray> vertexColours;
    vtkSmartPointer<vtkFloatArray> scales;

    std::map<unsigned long, unsigned long> matching;  // for graph coarsening (collapsing edges): maps node id of one end
    // of the collapsed edge to the other end's node id (or to itself)

    std::map<unsigned long, std::vector<unsigned long>> allMatching;
    std::map<unsigned long, unsigned long> matchMap;

    vtkColor4ub twoClauseColour;
    vtkColor4ub threePlusClauseColour;

    vtkColor4ub positiveColour;
    vtkColor4ub negativeColour;
    vtkColor4ub undefColour;

//    vtkSmartPointer<vtkLookupTable> vertexColours;

    std::mutex edge_colours_mutex;
    std::map<std::pair<unsigned long, unsigned long>, EdgeColour> edgeColourMap;

    void online_absolute_variance(Node3D &node);

    void online_absolute_variance(float x);

    void online_absolute_variance_remove(float x);

    void add_graph_edge(unsigned long x, unsigned long y, EdgeAttribute a);

    void add_graph_edge(unsigned long x, ExtNode3D y);

    inline void add_edge_to_graph(unsigned long x, unsigned long y) {
        std::scoped_lock vtkLock(vtk_graph_mutex);
        graph->AddEdge(x, y);
    }

    void remove_graph_edge(unsigned long x, unsigned long y);

    unsigned change_edge_duplication(unsigned duplication, bool increment = true);

    inline void set_colour(unsigned long x, unsigned long y) {
        set_colour({x, y});
    }

    inline void set_colour(const std::pair<unsigned long, unsigned long> vertices) {
        std::scoped_lock lock(edge_colours_mutex);
        set_colour(edgeColourMap[vertices]);
    }

    inline void set_colour(EdgeColour &e) const {
        e.setColour(highestEdgeDuplication());
        if (e.new_edge()) {
            e.setId(edgeColours->InsertNextTypedTuple(e.getColourData()));
        } else {
            set_colour_tuple(e);
        }
    }

    inline void set_two_or_three_clause_colour(unsigned long x, unsigned long y, EdgeAttribute a) {
        set_two_or_three_clause_colour({x, y}, a);
    }

    inline void set_two_or_three_clause_colour(const std::pair<unsigned long, unsigned long> vertices, EdgeAttribute a) {
        std::scoped_lock lock(edge_colours_mutex);
        set_two_or_three_clause_colour(edgeColourMap[vertices], a);
    }

    inline void set_two_or_three_clause_colour(EdgeColour &e, EdgeAttribute a) {
        if (e.getColour()[0] != threePlusClauseColour[0]
            || e.getColour()[1] != threePlusClauseColour[1]
            || e.getColour()[2] != threePlusClauseColour[2]) {
            e.setColour(a == NT_3_PLUS_CLAUSE ? threePlusClauseColour : twoClauseColour);
        }
    }

    inline void set_colour_tuple(unsigned long x, unsigned long y) {
        set_colour_tuple({x, y});
    }

    inline void set_colour_tuple(const std::pair<unsigned long, unsigned long> vertices) {
        std::scoped_lock lock(edge_colours_mutex);
        set_colour_tuple(edgeColourMap[vertices]);
    }

    inline void set_colour_tuple(EdgeColour &e) const {
        edgeColours->SetTypedTuple(e.getId(), e.getColourData());
    }

    inline bool new_edge(unsigned long x, unsigned long y) {
        return new_edge({x, y});
    }

    inline bool new_edge(const std::pair<unsigned long, unsigned long> vertices) {
        std::scoped_lock lock(edge_colours_mutex);
        return edgeColourMap[vertices].new_edge();
    }

    inline void set_duplication(unsigned long x, unsigned long y, bool increment = true) {
        set_duplication({x, y}, increment);
    }

    inline void set_duplication(const std::pair<unsigned long, unsigned long> vertices, bool increment = true) {
        std::scoped_lock lock(edge_colours_mutex);
        set_duplication(edgeColourMap[vertices], increment);
    }

    inline void set_duplication(EdgeColour &e, bool increment = true) {
        e.setDuplication(change_edge_duplication(e.getDuplication(), increment));
    }

    [[nodiscard]] float get_scale(const Node3D &node) const;

    [[nodiscard]] inline double node_occurrences_mean() const { return node_occurrences / (float) nr_nodes(); }

    [[nodiscard]] inline double average_absolute_variance() const { return absolute_variance / nr_nodes(); }

    [[nodiscard]] inline double node_occurrences_previous_mean(float x) const {
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
                twoClauseColour(126, 197, 247),
                threePlusClauseColour(189, 183, 128),
                positiveColour(31, 120, 180),
                negativeColour(213, 94, 0),
                undefColour(255, 255, 255) {

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

    void add_graph_edges_from_clause(const std::vector<long>& clause);

    void remove_graph_edge_from_ids(unsigned long x, unsigned long y);

    void remove_graph_edges_from_clause(const std::vector<long>& clause);

    void increase_variable_activity(unsigned long i);

    void assign_variable_truth_value(unsigned long i, bool value, bool undef = false);

    std::pair<std::vector<std::vector<long>>, unsigned> build_from_cnf(istream &is); // read file in DIMACS format, build graph

    // observables
    [[nodiscard]] unsigned nr_nodes() const { return nodes.size(); }

    [[nodiscard]] unsigned nr_edges() const { return number_edges; }

    [[nodiscard]] bool get_positioned() const { return positioned; }

    [[nodiscard]] bool get_drawn() const { return drawn; }

    [[nodiscard]] float highestEdgeDuplication() const {
        return (float) (!edgeDuplications.empty() ? edgeDuplications.rbegin()->first : 1);
    }

    [[nodiscard]] const vtkSmartPointer<vtkGraphToPolyData> &getGraphToPolyData() const { return graphToPolyData; }

    [[nodiscard]] const vtkSmartPointer<vtkPolyData> &getVertexPolydata() const { return vertexPolydata; }

    [[nodiscard]] unsigned no_of_occurrences() const { return (unsigned) node_occurrences; }

    // misc
    int independent_components(std::vector<int> *one_of_each_comp);
    // computes the number of independent components of the graph.
    // From each component the id of one node is returned.

    Graph3D *coarsen();
    // Coarsen graph by matching adjacent nodes (reduces the number of vertices to the half).

    void init_positions_at_random();
    // Initialize node positions randomly (all coordinates in the range [-1,1]).

    void init_coarsest_graph_positions(double k);
    // Initialize positions of the 2 nodes in coarsest graph.

    void init_positions_from_graph(Graph3D &g, double k);
    // Initialize positions from coarsened graph g. Positions in g must have been
    // computed already.

    void compute_layout(double k);
    // Compute layout according to Walshaw's paper "A Multilevel Algorithm for
    // Force-Directed Graph Drawing" (JGAA 2003)

    std::pair<Vector3D, Vector3D> compute_extremal_points();
    // Compute bounding box (p,q) where p is the min. and q the max point.

    // Move all vertices from position p to position q = a * p + b for scalar a and vector b
    void rescale(double a, const Vector3D &b);

    // Draw graph according to current layout (VTK).
    void drawPolyData(double k, bool draw_edges, bool draw_only_2clauses, bool adaptive_node_size);

    void reColour();

    void reScale();

    // I/O
    friend ostream &operator<<(ostream &os, const Graph3D &g);

    void set_all_matching(std::map<unsigned long, std::vector<unsigned long>> prev);

    void set_match_map(std::map<unsigned long, unsigned long> prev);

    void add_match(unsigned long node, unsigned long matched);
};

#endif
