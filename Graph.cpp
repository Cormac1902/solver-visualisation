#include <iostream>
#include <stack>
#include <climits>
#include <cctype>
#include <cassert>
#include "Graph.h"
#include "SpaceGrid.h"
#include "vtkUnsignedCharArray.h"
#include "vtkDataSetAttributes.h"
#include "vtkPointData.h"

#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

double min_scale = .5, max_scale = 2;
int large_graph = 100;

void Graph3D::add_node(const Node3D &n) {
    assert(nodes.find(n.id()) == nodes.end()); // node not already present

    nodes[n.id()] = n;
}

void Graph3D::set_all_matching(map<unsigned long, vector<unsigned long>> prev) {
    allMatching = std::move(prev);
}

void Graph3D::set_match_map(map<unsigned long, unsigned long> prev) {
    matchMap = std::move(prev);
}

void Graph3D::add_match(unsigned long node, unsigned long matched) {
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

void Graph3D::insert_edge(unsigned long x, unsigned long y, EdgeAttribute a) {
    // assertion: nodes x and y exist
    assert(nodes.find(x) != nodes.end() && nodes.find(y) != nodes.end());

    bool ins_x = nodes[x].add_neighbor(&nodes[y], a);

#ifdef NDEBUG
    nodes[y].add_neighbor(&nodes[x], a);
#else
    bool ins_y = nodes[y].add_neighbor(&nodes[x], a);
    assert(ins_x == ins_y); // either none or both edges must have existed before insertion
#endif

    if (ins_x) {
        number_edges++;
    }

    /*auto neighbours_size = max(nodes[x].no_of_occurrences(), nodes[y].no_of_occurrences());

    if (neighbours_size > largest_node) {
        largest_node = neighbours_size;
    }*/
}

void Graph3D::add_graph_edge(unsigned long x, unsigned long y, EdgeAttribute a) {
    pair<vtkIdType, pair<unsigned, vtkColor4ub>> &edgeColour = edgeColourMap[{x, y}];
    auto newEdge = edgeColour.second.first == 0;
    change_edge_duplication(edgeColour.second.first);
    if (edgeColour.second.second[0] != threePlusClauseColour[0]
        || edgeColour.second.second[1] != threePlusClauseColour[1]
        || edgeColour.second.second[2] != threePlusClauseColour[2]) {
        edgeColour.second.second = a == NT_3_PLUS_CLAUSE ? threePlusClauseColour : twoClauseColour;
    }
    if (newEdge) {
        add_edge_to_graph({x, y}, edgeColourMap[{x, y}]);
    } else {
        set_colour(edgeColour.second);
        edgeColours->SetTypedTuple(edgeColour.first, edgeColour.second.second.GetData());
    }
}

void Graph3D::add_graph_edge(unsigned long x, ExtNode3D y) {
    add_graph_edge(x,
                   std::distance(std::begin(nodes), nodes.find(matchMap[y.first->id()])),
                   y.second);
}

void Graph3D::add_graph_edge_from_ids(unsigned long x, unsigned long y, EdgeAttribute a) {
    add_graph_edge(std::distance(std::begin(nodes), nodes.find(matchMap[x])),
                   std::distance(std::begin(nodes), nodes.find(matchMap[y])),
                   a);
}

void Graph3D::add_edge_to_graph(
        pair<const pair<unsigned long, unsigned long>, pair<vtkIdType, pair<unsigned int, vtkColor4ub>>> &edge) {
    add_edge_to_graph(edge.first, edge.second);
}

void Graph3D::add_edge_to_graph(pair<unsigned long, unsigned long> vertices,
                                pair<vtkIdType, pair<unsigned int, vtkColor4ub>> &colour) {
    graph->AddEdge(vertices.first, vertices.second);
    set_colour(colour.second);
    colour.first = edgeColours->InsertNextTupleValue(colour.second.second.GetData());
}

void Graph3D::add_graph_edges_from_clause(vector<long> clause) {
    auto a = (clause.size() == 2 ? NT_2_CLAUSE : NT_3_PLUS_CLAUSE);
    for (auto &var : clause) {
        var = abs(var);
    }
    std::sort(clause.begin(), clause.end());
    for (auto i = clause.begin(); i < clause.end(); i++) {
        for (auto j = i + 1; j != clause.end(); j++) {
            add_graph_edge_from_ids(*i, *j, a);
        }
    }

    graphToPolyData->Modified();
}

void Graph3D::remove_graph_edge(unsigned long x, unsigned long y) {
    pair<vtkIdType, pair<unsigned, vtkColor4ub>> &edgeColour = edgeColourMap[{x, y}];
    if (edgeColour.second.first > 0) {
        change_edge_duplication(edgeColour.second.first, false);
        set_colour(edgeColour.second);
        edgeColours->SetTypedTuple(edgeColour.first, edgeColour.second.second.GetData());
    }
}

void Graph3D::remove_graph_edge_from_ids(unsigned long x, unsigned long y) {
    remove_graph_edge(std::distance(std::begin(nodes), nodes.find(matchMap[x])),
                      std::distance(std::begin(nodes), nodes.find(matchMap[y])));
}

void Graph3D::remove_graph_edges_from_clause(vector<long> clause) {
    for (auto &var : clause) {
        var = abs(var);
    }
    std::sort(clause.begin(), clause.end());
    for (auto i = clause.begin(); i < clause.end(); i++) {
        for (auto j = i + 1; j != clause.end(); j++) {
            remove_graph_edge_from_ids(*i, *j);
        }
    }

    graphToPolyData->Modified();
}

void Graph3D::change_edge_duplication(unsigned &duplication, bool increment) {
    if (increment) {
        if (duplication > 0) {
            edgeDuplications[duplication]--;
        }
        duplication++;
    } else {
        if (edgeDuplications[duplication] > 0) {
            edgeDuplications[duplication]--;
        } else {
            edgeDuplications.erase(duplication);
        }
        duplication--;
    }
    if (duplication > 0) {
        edgeDuplications[duplication]++;
    }
}

pair<vector<vector<long>>, unsigned int> Graph3D::build_from_cnf(istream &is) {
    long p;
    char c;
    bool positive = true;
    vector<long> clause;
    vector<vector<long>> clauses;
    unsigned int longestClause = 0;
    Node3D n;

    while (!is.eof()) {

        // skip comment & problem description lines
        while ((c = is.get()) == 'c' || c == 'p')
            while ((c = is.get()) != '\n');
        is.unget();

        clause.clear();
        do {
            while (isspace(c = is.get())) // ignore spaces and '-'
                ;
            if (is.eof())
                break;
            else {
                if (c == '-') {
                    positive = false;
                } else {
                    is.unget();
                    p = 0;

                    while (isdigit(c = is.get()))
                        p = 10 * p + c - '0';

                    if (!positive)
                        p = -p;

                    if (p != 0)
                        clause.push_back(p);

                    positive = true;
                }
            }
        } while (p != 0);

        if (!clause.empty()) {

            clauses.push_back(clause);
            longestClause = max(longestClause, (unsigned int) clause.size());
            // insert nodes
            for (long &i : clause) {
                i = abs(i);
                if (nodes.find(i) == nodes.end()) { // node not yet present
                    n.set_id(i);
                    add_node(n);
                    allMatching[n.id()] = {};
                    matchMap[n.id()] = n.id();
                } else {
                    nodes[i].increment_occurrences();
                    if (nodes[i].no_of_occurrences() > largest_node) {
                        largest_node = nodes[i].no_of_occurrences();
                    }
                }
                node_occurrences++;
            }

            // insert edges
            EdgeAttribute a = (clause.size() == 2 ? NT_2_CLAUSE : NT_3_PLUS_CLAUSE);
            for (auto i = clause.begin(); i != clause.end(); i++)
                for (auto j = clause.begin(); j != i; j++)
                    if (*i != *j)
                        insert_edge(*i, *j, a);
        }
    }

    return {clauses, longestClause};
}

void Graph3D::calculate_absolute_variance() {
    absolute_variance = 0.0;
    for (auto &node : nodes) {
        absolute_variance += abs(node.second.no_of_occurrences() - node_occurrences_mean());
    }
}

void Graph3D::online_absolute_variance(Node3D &node) {
    if (nr_nodes() < 100) {
        node_occurrences++;
        calculate_absolute_variance();
    } else {
        online_absolute_variance(node.no_of_occurrences());
    }
}

// TODO: Check accuracy
void Graph3D::online_absolute_variance(float x) {
    if (x > 1) {
        online_absolute_variance_remove(x - 1);
    }
    node_occurrences++;
    auto delta = nr_nodes() > 1 ? x - node_occurrences_previous_mean(x) : 0;
//    auto squaredDelta = delta * (x - node_occurrences_mean());
//    cout << node_occurrences_mean() << endl;
//    cout << x << endl;
//    cout << "Add delta: " << squaredDelta << endl;
    absolute_variance += sqrt(abs(delta * (x - node_occurrences_mean())));
//    cout << "Add variance: " << absolute_variance << endl;
}

void Graph3D::online_absolute_variance_remove(float x) {
//    auto squaredDelta = (x - node_occurrences_mean()) * (x - node_occurrences_previous_mean(x));
//    cout << "Curr: " << x << " - " << node_occurrences_mean() << " = " << (x - node_occurrences_mean()) << endl;
//    cout << "Prev: " << x << " - " << node_occurrences_previous_mean(x) << " = "
//         << (x - node_occurrences_previous_mean(x)) << endl;
//    cout << "Remove delta: " << squaredDelta << endl;
    absolute_variance -= sqrt(abs((x - node_occurrences_mean()) * (x - node_occurrences_previous_mean(x))));
//    cout << "Remove variance: " << absolute_variance << endl;
}

// one_of_each_component is reset
int Graph3D::independent_components(vector<int> *one_of_each_component) {
    map<unsigned long, Node3D>::iterator first_unmarked = nodes.begin(), nmi;
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

Graph3D *Graph3D::coarsen() {
    set<ExtNode3D>::iterator j;
    int curr_min_weight;
    Node3D *curr_min_weight_node;

    // build matching
    for (auto &node : nodes) {
        Node3D &node_1 = node.second;
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
    for (auto &node : nodes)
        node.second.mark = 0;

    // build coarsened graph
    map<int, int>::iterator k;

    auto *result = new Graph3D();
    result->set_all_matching(allMatching);
    result->set_match_map(matchMap);
    result->node_occurrences = node_occurrences;

    // a) set up nodes
    for (k = matching.begin(); k != matching.end(); k++) {
        Node3D n(k->second); // use second partner of each pair as merged node's id
        auto prev = nodes[k->first];
        if (k->first == k->second) {
            n.set_weight(prev.weight());
            n.set_occurrences(prev.no_of_occurrences());
        } else {
            auto secondPrev = nodes[k->second];
            n.set_weight(prev.weight() + secondPrev.weight()); // weight of merged node is sum of constituents
            n.set_occurrences(prev.no_of_occurrences() + secondPrev.no_of_occurrences());
        }
        if (n.no_of_occurrences() > result->largest_node) {
            result->largest_node = n.no_of_occurrences();
        }
        result->add_node(n);
        result->add_match(k->first, k->second);
    }
    // b) set up edges
    unsigned long new_node_1_id, new_node_2_id;
    for (auto &i : nodes) {
        absolute_variance += abs(i.second.no_of_occurrences() - node_occurrences_mean());
//        cout << absolute_variance << endl;
        Node3D &node = i.second;
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
    for (auto &node : nodes)
        sg.insert_node(&node.second);
#endif

    // iteratively compute layout

    while (!converged) {

        converged = true;

        for (auto &node : nodes) {
            Node3D &v = node.second;

            theta = Vector3D(0.0, 0.0, 0.0);

            // calculate (global) repulsive forces
#ifdef USE_SPACE_GRID
            grid_neighbors = sg.find_neighbors(&v);
            for (auto &grid_neighbor : grid_neighbors)
                if (grid_neighbor != &node.second) {  // |delta| <= R is not enforced! (better layout quality)
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

    for (auto &node : nodes) {
        Node3D &n = node.second;
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
    for (auto &node : nodes) {
        Node3D &n = node.second;
        n.set_pos(a * n.position() + b);
    }
}

void Graph3D::drawPolyData(double k, bool draw_edges, bool draw_only_2clauses, bool adaptive_node_size) {

    if (!drawn) {
        drawn = true;

        graph = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
        points = vtkSmartPointer<vtkPoints>::New();
        scales = vtkSmartPointer<vtkFloatArray>::New();

        vertexColours = vtkSmartPointer<vtkUnsignedCharArray>::New();
        vertexColours->SetName("Vertex Colours");
        // vtkSmartPointer<vtkLookupTable> coloursLookupTable;

//    vtkSmartPointer<vtkIntArray> edgeColours;
        bool colourPoints = true;

        vertexColours->SetNumberOfComponents(3);

//    draw_only_2clauses = true;

        // Create a vtkCellArray container and store the lines in it
        if (draw_edges) {


            // Create a vtkUnsignedCharArray container and store the colors in it
            edgeColours = vtkSmartPointer<vtkUnsignedCharArray>::New();
//        = vtkSmartPointer<vtkIntArray>::New();
            edgeColours->SetNumberOfComponents(4);
            edgeColours->SetName("Edge Colours");
        }

        graph->SetNumberOfVertices(nodes.size());

        for (auto i = nodes.begin(); i != nodes.end(); i++) {
            const Node3D &node = i->second;
            points->InsertNextPoint(node.position().x, node.position().y, node.position().z);
//            cout << scale << endl;
            scales->InsertNextValue(get_scale(node));

            if (colourPoints) {
                vertexColours->InsertNextTupleValue(
                        (node.no_of_occurrences() == largest_node ? positiveColour : negativeColour).GetData());
            }
            if (draw_edges) {
                const Node3D &u = i->second;
                auto nodeIndex = std::distance(std::begin(nodes), nodes.find(u.id()));
                for (const auto &neighbor : u.neighbors()) {
                    if (u.id() < neighbor.first->id()) { // draw edges only in direction of incr. ids; no self-edg.
                        if (!draw_only_2clauses || neighbor.second != NT_3_PLUS_CLAUSE) {
                            add_graph_edge(nodeIndex, neighbor);
                        }
                    }
                }
            }
        }

        graph->SetPoints(points);

        if (draw_edges) {
            graph->GetEdgeData()->SetScalars(edgeColours);
        }

        graphToPolyData->SetInputData(graph);

        graphToPolyData->Update();

        vertexPolydata = vtkSmartPointer<vtkPolyData>::New();
        vertexPolydata->SetPoints(graphToPolyData->GetOutput()->GetPoints());
        vertexPolydata->GetPointData()->SetScalars(scales);
        vertexPolydata->GetPointData()->AddArray(vertexColours);

        if (colourPoints) {
//            graph->GetVertexData()->SetScalars(vertexColours);
//            graphToPolyData->GetOutput()->GetPointData()->SetScalars(vertexColours);
        }
    }

}

void Graph3D::reColour() {
    for (auto &edge : edgeColourMap) {
        set_colour(edge.second.second);
        edgeColours->SetTypedTuple(edge.second.first, edge.second.second.second.GetData());
    }

//    graphToPolyData->Modified();
}

void Graph3D::increase_variable_activity(unsigned long i) {
    i = matchMap[i];
    nodes[i].increment_occurrences();
    online_absolute_variance(nodes[i]);

    reScale();

    vertexPolydata->Modified();
}

void Graph3D::assign_variable_truth_value(unsigned long i, bool value) {
    i = matchMap[i];
    vertexColours->SetTypedTuple(i, (value ? positiveColour : negativeColour).GetData());

    vertexPolydata->Modified();
}

void Graph3D::reScale() {
    for (auto i = 0; i < scales->GetNumberOfValues(); i++) {
        scales->SetValue(i, get_scale(nodes[i]));
    }
}

void Graph3D::set_colour(pair<unsigned int, vtkColor4ub> &colour) const {
    colour.second[3] = 255 * ((float) colour.first / highestEdgeDuplication());
}

float Graph3D::get_scale(const Node3D &node) const {
    auto scale = (
                         ((node.no_of_occurrences() - node_occurrences_mean())
                          / (average_absolute_variance() < 1 ? 1 : average_absolute_variance()))
                         / 4)
                 + 1;
    return scale < min_scale ? min_scale : min(scale, max_scale);
}

ostream &operator<<(ostream &os, const Graph3D &g) {
    set<ExtNode3D>::iterator j;

    os << "Nodes:" << endl;
    os << "\t";
    for (auto i = g.nodes.begin(); i != g.nodes.end(); i++) {
        if (i != g.nodes.begin())
            os << ", ";
        os << i->first;
    }
    os << endl;
    os << "Edges:" << endl;
    for (const auto &node : g.nodes) {
        const Node3D *n = &node.second;
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

