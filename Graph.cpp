#include <iostream>
#include <stack>
#include <climits>
#include <cctype>
#include <cassert>
#include "Graph.h"
#include "SpaceGrid.h"
#include <vtkPoints.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkLine.h>
#include <vtkUnsignedCharArray.h>
#include <vtkNamedColors.h>
#include <vtkCellData.h>
#include <vtkPointSource.h>
#include <vtkPointData.h>
#include <vtkUndirectedGraph.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphLayoutView.h>
#include <vtkGraphToPolyData.h>
#include <vtkLookupTable.h>

#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

void Graph3D::add_node(const Node3D &n) {
    assert(nodes.find(n.id()) == nodes.end()); // node not already present

    nodes[n.id()] = n;
}

void Graph3D::set_all_matching(map<long, vector<long>> prev) {
    allMatching = std::move(prev);
}

void Graph3D::set_match_map(map<long, long> prev) {
    matchMap = std::move(prev);
}

void Graph3D::add_match(long node, long matched) {
    if (node != matched) {
        allMatching[matched].push_back(node);
        allMatching[matched].insert(allMatching[matched].end(), allMatching[node].begin(), allMatching[node].end());
        matchMap[node] = matched;
        for (auto match : allMatching[node]) {
            matchMap[match] = matched;
        }
        allMatching.erase(node);
    }
}

void Graph3D::insert_edge(long x, long y, EdgeAttribute a) {
    // assertion: nodes x and y exist
    assert(nodes.find(x) != nodes.end() && nodes.find(y) != nodes.end());

    bool ins_x = nodes[x].add_neighbor(&nodes[y], a);

#ifdef NDEBUG
    nodes[y].add_neighbor(&nodes[x], a);
#else
    bool ins_y = nodes[y].add_neighbor(&nodes[x], a);
    assert(ins_x == ins_y); // either none or both edges must have existed before insertion
#endif

    if (ins_x)
        number_edges++;
}

void Graph3D::build_from_cnf(istream &is) {
    long p;
    char c;
    vector<long> clause;
    Node3D n;

    while (!is.eof()) {

        // skip comment & problem description lines
        while ((c = is.get()) == 'c' || c == 'p')
            while ((c = is.get()) != '\n');
        is.unget();

        clause.clear();
        do {
            while (isspace(c = is.get()) || c == '-') // ignore spaces and '-'
                ;
            if (is.eof())
                break;
            else {
                is.unget();
                p = 0;
                while (isdigit(c = is.get()))
                    p = 10 * p + c - '0';
                if (p != 0)
                    clause.push_back(p);
            }
        } while (p != 0);

        if (!clause.empty()) {
            // insert nodes
            for (long &i : clause)
                if (nodes.find(i) == nodes.end()) { // node not yet present
                    n.set_id(i);
                    add_node(n);
                    allMatching[n.id()] = {};
                    matchMap[n.id()] = n.id();
                }

            // insert edges
            EdgeAttribute a = (clause.size() == 2 ? NT_2_CLAUSE : NT_3_PLUS_CLAUSE);
            for (auto i = clause.begin(); i != clause.end(); i++)
                for (auto j = clause.begin(); j != i; j++)
                    if (*i != *j)
                        insert_edge(*i, *j, a);
        }
    }
}

// one_of_each_component is reset
int Graph3D::independent_components(vector<int> *one_of_each_component) {
    map<int, Node3D>::iterator first_unmarked = nodes.begin(), nmi;
    int nr_components = 0;
    stack<Node3D *> node_stack;

    one_of_each_component->clear();
    for (;;) {
        while (first_unmarked->second.mark != 0 && first_unmarked != nodes.end())
            first_unmarked++;

        if (first_unmarked == nodes.end())
            goto UNMARK_NODES;

        // now first_unmarked points to a yet unvisited node
        Node3D &head_node = first_unmarked->second;
        assert(head_node.mark == 0);

        one_of_each_component->push_back(first_unmarked->first);
        nr_components++;
        node_stack.push(&head_node);

        // depth-first search; marks:
        // 0: not yet visited
        // 1: visited and processed

        while (!node_stack.empty()) {
            Node3D *node = node_stack.top();
            node_stack.pop();
            node->mark = 1;
            for (const auto &i : node->neighbors()) {
                if (i.first->mark == 0)
                    node_stack.push(i.first);
            }
        }
    }

    UNMARK_NODES:
    for (nmi = nodes.begin(); nmi != nodes.end(); nmi++)
        nmi->second.mark = 0;

    return nr_components;
}

Graph3D *Graph3D::coarsen(int level) {
    map<int, Node3D>::iterator i;
    set<ExtNode3D>::iterator j;
    int curr_min_weight;
    Node3D *curr_min_weight_node;

    // build matching
    for (i = nodes.begin(); i != nodes.end(); i++) {
        Node3D &node_1 = i->second;
        if (node_1.mark != 0)
            continue;   // already paired
        curr_min_weight = INT_MAX;
        curr_min_weight_node = nullptr;
        for (j = node_1.neighbors().begin(); j != node_1.neighbors().end(); j++) {
            // search 'best' matching neighbor, i.e. one with lowest weight
            Node3D *node_2_ptr = j->first;
            if (node_2_ptr->mark != 0)
                continue; // already paired
            if (node_2_ptr->weight() < curr_min_weight) {
                curr_min_weight = node_2_ptr->weight();
                curr_min_weight_node = node_2_ptr;
            }
        }
        if (curr_min_weight_node == nullptr)
            curr_min_weight_node = &node_1; // no partner found: match node with itself

        // enter pairing
        node_1.mark = 1;
        curr_min_weight_node->mark = 1;
        matching[node_1.id()] = curr_min_weight_node->id();
    }

    // remove markings
    for (i = nodes.begin(); i != nodes.end(); i++)
        i->second.mark = 0;

    // build coarsened graph
    map<int, int>::iterator k;

    auto *result = new Graph3D();
    result->set_all_matching(allMatching);
    result->set_match_map(matchMap);

    // a) set up nodes
    for (k = matching.begin(); k != matching.end(); k++) {
        Node3D n(k->second); // use second partner of each pair as merged node's id
        if (k->first == k->second)
            n.set_weight(nodes[k->first].weight());
        else
            n.set_weight(nodes[k->first].weight() + nodes[k->second].weight());
        // weight of merged node is sum of constituents
        result->add_node(n);
        result->add_match(k->first, k->second);

    }
    // b) set up edges
    long new_node_1_id, new_node_2_id;
    for (i = nodes.begin(); i != nodes.end(); i++) {
        Node3D &node = i->second;
        k = matching.find(node.id());
        new_node_1_id = (k == matching.end() ? node.id() : matching[node.id()]);
        for (j = node.neighbors().begin(); j != node.neighbors().end(); j++) {
            k = matching.find((j->first)->id());
            new_node_2_id = (k == matching.end() ? j->first->id() : matching[j->first->id()]);
            if (new_node_1_id < new_node_2_id) // no self-edges, only one 'direction'
                result->insert_edge(new_node_1_id, new_node_2_id);
        }
    }

    return result;
}

void Graph3D::init_positions_at_random() {
    for (auto &node : nodes)
        node.second.set_pos(Vector3D::init_random());
}

void Graph3D::init_coarsest_graph_positions(double k) {
    assert(nodes.size() == 2);

    auto i = nodes.begin();
    i->second.set_pos(-k / 2.0, 0.0, 0.0);
    i++;
    i->second.set_pos(k / 2.0, 0.0, 0.0);

    positioned = true;
}

// g is coarser than 'this'
void Graph3D::init_positions_from_graph(Graph3D *g, double k) {
    Vector3D offset;
    auto f_k = 0.001 * k;

    for (auto &i : matching) {
        // copy already computed positions
        if (i.first == i.second) {
            nodes[i.first].set_pos(g->nodes[i.second].position());
        } else {
            offset = f_k * Vector3D::init_random().normalize();
            nodes[i.first].set_pos(g->nodes[i.second].position() + offset);
            nodes[i.second].set_pos(g->nodes[i.second].position() - offset);
        }
    }

    positioned = true;
}

void Graph3D::compute_layout(double k) {
    map<int, Node3D>::iterator i;
    set<ExtNode3D>::iterator n;
    bool converged = false;
    double f_r, f_r_aux, f_a, t;
    Vector3D delta, theta;

    const auto tol = 0.01;
    const auto C = 0.2;
    const auto lambda = 0.9;

    t = k;
    f_r_aux = -C * k * k;

#ifdef USE_SPACE_GRID
    // put nodes into space grid (with cube length 2k)
    vector<Node3D *> grid_neighbors;
    SpaceGrid3D sg(2.0 * k);  // R = 2.0 * k
    for (i = nodes.begin(); i != nodes.end(); i++)
        sg.insert_node(&i->second);
#endif

    // iteratively compute layout

    while (!converged) {

        converged = true;

        for (i = nodes.begin(); i != nodes.end(); i++) {
            Node3D &v = i->second;

            theta = Vector3D(0.0, 0.0, 0.0);

            // calculate (global) repulsive forces
#ifdef USE_SPACE_GRID
            grid_neighbors = sg.find_neighbors(&v);
            for (auto &grid_neighbor : grid_neighbors)
                if (grid_neighbor != &i->second) {  // |delta| <= R is not enforced! (better layout quality)
                    delta = grid_neighbor->position() - v.position();
                    f_r = f_r_aux * grid_neighbor->weight() / delta.norm();
                    theta += f_r * delta.normalize();
                }
#else
            for(map<int,Node3D>::iterator j = nodes.begin(); j != nodes.end(); j++)
              if(j != i) {
                Node3D& u = j->second;
                delta = u.position() - v.position();
                f_r = f_r_aux * u.weight() / delta.norm();
                theta += f_r * delta.normalize();
              }
#endif

            // calculate (local) attractive/spring forces
            const set<ExtNode3D> &neighbors = v.neighbors();
            for (n = neighbors.begin(); n != neighbors.end(); n++) {
                delta = n->first->position() - v.position();
                double dn = delta.norm();
                f_a = dn * dn / k;
                theta += f_a * delta.normalize();
            }

            // reposition node v
            delta = min(t, theta.norm()) * theta.normalize();
            v.set_pos(v.position() + delta);

            if (delta.norm() > k * tol)
                converged = false;
        }

        t = lambda * t;
    }
}

pair<Vector3D, Vector3D> Graph3D::compute_extremal_points() {
    Vector3D curr_min(DBL_MAX, DBL_MAX, DBL_MAX), curr_max(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    map<int, Node3D>::iterator i;

    for (i = nodes.begin(); i != nodes.end(); i++) {
        Node3D &n = i->second;
        if (n.position().x < curr_min.x)
            curr_min.x = n.position().x;
        if (n.position().x > curr_max.x)
            curr_max.x = n.position().x;
        if (n.position().y < curr_min.y)
            curr_min.y = n.position().y;
        if (n.position().y > curr_max.y)
            curr_max.y = n.position().y;
        if (n.position().z < curr_min.z)
            curr_min.z = n.position().z;
        if (n.position().z > curr_max.z)
            curr_max.z = n.position().z;
    }

    return {curr_min, curr_max};
}

// move all points p to q = a * p + b for a vector b and a float a

void Graph3D::rescale(double a, const Vector3D &b) {
    map<int, Node3D>::iterator i;
    for (i = nodes.begin(); i != nodes.end(); i++) {
        Node3D &n = i->second;
        n.set_pos(a * n.position() + b);
    }
}

vtkGraphToPolyData *Graph3D::drawPolyData(double k, bool draw_edges, bool draw_only_2clauses, bool adaptive_node_size) {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkMutableUndirectedGraph> graph = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
    vtkSmartPointer<vtkUnsignedCharArray> vertexColours = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vertexColours->SetName("Vertex Colours");
    vtkGraphToPolyData *graphToPolyData = vtkGraphToPolyData::New();
//    vtkSmartPointer<vtkCellArray> lines;
    vtkSmartPointer<vtkNamedColors> namedColours;
    // vtkSmartPointer<vtkLookupTable> coloursLookupTable;
    vtkSmartPointer<vtkUnsignedCharArray> edgeColours;
//    vtkSmartPointer<vtkIntArray> edgeColours;
    vtkColor4ub twoClauseColour;
    vtkColor4ub threePlusClauseColour;
    map<pair<unsigned, unsigned>, pair<unsigned, pair<unsigned, vtkColor4ub>>> edgeColourMap;  // <vertex, vertex, <insertionOrder, <occurrences, colour>>>
    bool colourPoints = true;
    float highestEdgeDuplication = 0;

    // draw_edges = false;

    vertexColours->SetNumberOfComponents(3);

//    draw_only_2clauses = true;

    namedColours = vtkSmartPointer<vtkNamedColors>::New();

    twoClauseColour = namedColours->GetColor4ub("Tomato");
    threePlusClauseColour = namedColours->GetColor4ub("Mint");

    // Create a vtkCellArray container and store the lines in it
    if (draw_edges) {


        // Create a vtkUnsignedCharArray container and store the colors in it
        edgeColours
        = vtkSmartPointer<vtkUnsignedCharArray>::New();
//        = vtkSmartPointer<vtkIntArray>::New();
        edgeColours->SetNumberOfComponents(4);
        edgeColours->SetName("Edge Colours");

/*        coloursLookupTable = vtkSmartPointer<vtkLookupTable>::New();
        coloursLookupTable->SetNumberOfTableValues(2);
        coloursLookupTable->SetTableValue(0, twoClauseColour.GetRed(), twoClauseColour.GetGreen(), twoClauseColour.GetBlue()); // red
        coloursLookupTable->SetTableValue(1, threePlusClauseColour.GetRed(), threePlusClauseColour.GetGreen(), threePlusClauseColour.GetBlue()); // green
        coloursLookupTable->Build();*/
    }

    graph->SetNumberOfVertices(nodes.size());

    for (auto i = nodes.begin(); i != nodes.end(); i++) {
        const Node3D &node = i->second;
        points->InsertNextPoint(node.position().x, node.position().y, node.position().z);
        vtkSmartPointer<vtkPointSource> pointSource = vtkSmartPointer<vtkPointSource>::New();

        if (colourPoints) {
            vertexColours->InsertNextTupleValue(
                    (i->first % 2 == 0 ? twoClauseColour : threePlusClauseColour).GetData());
        }
        if (draw_edges) {
            const Node3D &u = i->second;
            const set<ExtNode3D> &neighbors = u.neighbors();
            auto nodeIndex = std::distance(std::begin(nodes), nodes.find(u.id()));
            for (const auto & neighbor : neighbors) {
                const Node3D *vp = neighbor.first;
                if (u.id() < vp->id()) { // draw edges only in direction of incr. ids; no self-edg.
                    EdgeAttribute a = neighbor.second;
                    if (!draw_only_2clauses || a != NT_3_PLUS_CLAUSE) {
                        auto neighbourIndex = std::distance(std::begin(nodes), nodes.find(vp->id()));
                        if (edgeColourMap[{nodeIndex, neighbourIndex}].second.first == 0) {
                            edgeColourMap[{nodeIndex, neighbourIndex}].first = edgeColourMap.size();
                        }
                        edgeColourMap[{nodeIndex, neighbourIndex}].second.first++;
                        if (edgeColourMap[{nodeIndex, neighbourIndex}].second.first > highestEdgeDuplication) {
                            highestEdgeDuplication = edgeColourMap[{nodeIndex, neighbourIndex}].second.first;
                        }
                        if (edgeColourMap.find({nodeIndex, neighbourIndex})->second.second.second != threePlusClauseColour) {
                            edgeColourMap[{nodeIndex, neighbourIndex}].second.second =
                                    a != NT_3_PLUS_CLAUSE ? twoClauseColour : threePlusClauseColour;
                        }
                    }
                }
            }
        }
    }

    for (auto & edge : edgeColourMap) {
        graph->AddEdge(edge.first.first, edge.first.second);
        edge.second.second.second[3] = 255 * ((float) edge.second.second.first / highestEdgeDuplication);
        edgeColours->InsertNextTupleValue(edge.second.second.second.GetData());
//        std::cout << edge.first.first << " " << edge.first.second << ": " << edge.second.first << " " << edge.second.second.first << " " << edge.second.second.second << std::endl;
    }

    /*auto colour = threePlusClauseColour;
    colour[3] = 0;

    edgeColours->SetTypedTuple(0, colour.GetData());*/

    graph->SetPoints(points);

    if (colourPoints) {
        graph->GetVertexData()->AddArray(vertexColours);
    }

    if (draw_edges) {
        graph->GetEdgeData()->SetScalars(edgeColours);
    }

    // Visualize
    /*vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
            vtkSmartPointer<vtkGraphLayoutView>::New();
    graphLayoutView->AddRepresentationFromInput(graph);
    graphLayoutView->SetLayoutStrategyToPassThrough();
    graphLayoutView->SetVertexColorArrayName("Vertex Colours");
    graphLayoutView->ColorVerticesOn();*/

    graphToPolyData->SetInputData(graph);
    graphToPolyData->Update();

    if (colourPoints) {
//        graphToPolyData->GetOutput()->GetPointData()->AddArray(vertexColours);
    }

    /*vtkSmartPointer<vtkViewTheme> theme =
            vtkSmartPointer<vtkViewTheme>::New();

    vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
            vtkSmartPointer<vtkGraphLayoutView>::New();
    graphLayoutView->AddRepresentationFromInput(graph);
    graphLayoutView->SetLayoutStrategyToPassThrough();

    graphLayoutView->SetVertexColorArrayName("Colour");
    graphLayoutView->ColorVerticesOn();
    theme->SetCellLookupTable(coloursLookupTable);

    graphLayoutView->SetEdgeColorArrayName("Colour");
    graphLayoutView->ColorEdgesOn();
    theme->SetPointLookupTable(coloursLookupTable);

    graphLayoutView->ApplyViewTheme(theme);

    Interaction *interactor = Interaction::New();
    interactor->SetCurrentRenderer(graphLayoutView->GetRenderer());
    graphLayoutView->SetDisplaySize(1920, 1080);
    graphLayoutView->Render();
    graphLayoutView->GetInteractor()->Start();*/


    //cout << "render" << endl;
    //cout << "second" << endl;

    return graphToPolyData;
    //return graph;
}

ostream &operator<<(ostream &os, const Graph3D &g) {
    map<int, Node3D>::const_iterator i;
    set<ExtNode3D>::iterator j;

    os << "Nodes:" << endl;
    os << "\t";
    for (i = g.nodes.begin(); i != g.nodes.end(); i++) {
        if (i != g.nodes.begin())
            os << ", ";
        os << i->first;
    }
    os << endl;
    os << "Edges:" << endl;
    for (i = g.nodes.begin(); i != g.nodes.end(); i++) {
        const Node3D *n = &i->second;
        const set<ExtNode3D> &nbs = n->neighbors();
        os << "\t";
        for (j = nbs.begin(); j != nbs.end(); j++) {
            if (j != nbs.begin())
                os << ", ";
            os << n->id() << "-" << j->first->id();
            // if(j->second == NT_2_CLAUSE)
            //   os << " [2]";
        }
        os << endl;
    }

    return os;
}
