//
// Created by Cormac on 30-Apr-20.
//

#include <vtkLine.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include "vtkExecutive.cxx"

#include <utility>

#include "Display.h"
#include "WalkSAT.h"

#include "API.h"

#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

static double k = 1.0; // old: 5.0
static double f_k = sqrt(4.0 / 7.0);

unsigned Display::RENDER_SOCKET = 29790;
unsigned Display::CHANGE_GRAPH_SOCKET = 29791;
unsigned Display::API_RUNNING_SOCKET = 29792;

static bool draw_edges = true, adaptive_size = true, draw_only_2clauses = false; // run_anim = falseClauses

static float min_line_width = 3, max_line_width = 12;
static int max_line_width_threshold = 250, min_line_width_threshold = 2500;

// ----------------------------------------------------------------------

void Display::switchDisplay(Graph3D &g, double l) {

    auto highestEdgeDuplication = g.highestEdgeDuplication();

    if (!g.get_drawn()) {
        g.drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size);
    }

    edgeMapper->SetInputConnection(g.getGraphToPolyData()->GetOutputPort());

    if (g.highestEdgeDuplication() != highestEdgeDuplication) {
        g.reColour();
    }

    auto numberOfLines = edgeMapper->GetInput()->GetNumberOfLines();

    auto lineWidth = min(max_line_width,
                         ((float) (min_line_width_threshold - max_line_width_threshold) / numberOfLines) +
                         min_line_width);

    edgeActor->GetProperty()->SetLineWidth(lineWidth);

    sphereSource->SetRadius(lineWidth * 3 / 1000);

    glyph3D->SetInputData(g.getVertexPolydata());

    current_graph = &g;
}

void Display::changeGraphFromClause(Graph3D *g, vector<long> clause, bool add) {
    std::scoped_lock lock{graph_mutex};

    auto highestEdgeDuplication = g->highestEdgeDuplication();

    if (add) {
        g->add_graph_edges_from_clause(std::move(clause));
    } else {
        g->remove_graph_edges_from_clause(std::move(clause));
    }

    if (g->highestEdgeDuplication() != highestEdgeDuplication) {
        g->reColour();
    }

    g->getGraphToPolyData()->Modified();
}

void Display::addEdgesFromClause(Graph3D *g, vector<long> clause) {
    changeGraphFromClause(g, std::move(clause));
}

void Display::addEdgesFromClause(vector<long> clause) {
    addEdgesFromClause(current_graph, std::move(clause));
}

void Display::removeEdgesFromClause(Graph3D *g, vector<long> clause) {
    changeGraphFromClause(g, std::move(clause), false);
}

void Display::removeEdgesFromClause(vector<long> clause) {
    removeEdgesFromClause(current_graph, std::move(clause));
}

void Display::increaseVariableActivity(Graph3D *g, unsigned long i) {
    std::scoped_lock lock{graph_mutex};

    g->increase_variable_activity(i);

    g->getVertexPolydata()->Modified();
}

void Display::increaseVariableActivity(unsigned long i) {
    increaseVariableActivity(current_graph, i);
}

void Display::assignVariable(Graph3D *g, unsigned long i, bool value, bool undef) {
    std::scoped_lock lock{graph_mutex};

    g->assign_variable_truth_value(i, value, undef);

    g->getVertexPolydata()->Modified();
}

void Display::assignVariable(unsigned long i, bool value, bool undef) {
    assignVariable(current_graph, i, value, undef);
}

void Display::setupNodes(Graph3D& g) {
    Node3D n(1), m(2), o(3);

    g.add_node(n);
    g.add_node(m);
    g.add_node(o);
    g.insert_edge(1, 2);
    g.insert_edge(1, 3);
}

void Display::changeGraph(unsigned graphLevel) {

    std::string graphMessage;

    if (graphLevel == 0) {
        graphMessage = "non-coarsened graph";
    } else {
        graphMessage = "coarsened graph ";
        graphMessage.append(std::to_string(graphLevel));
        if (graph_stack.size() <= graphLevel) {
            std::cout << "No " << graphMessage << std::endl;
            return;
        }
    }

    if (!graph_stack[graphLevel]->get_positioned()) {
        auto capitalisedGraphMessage = graphMessage;
        capitalisedGraphMessage[0] = toupper(capitalisedGraphMessage[0]);
        std::cout << capitalisedGraphMessage << " is not positioned" << std::endl;
        if (!graph_stack[graphLevel + 1]->get_positioned()) {
            for (auto j = graph_stack.size() - 2; j > graphLevel; j--) {
                if (!graph_stack[j]->get_positioned()) {
                    std::cout << "Positioning coarsened graph " << j << std::endl;
                    positionGraph(j);
                }
            }
        }

        std::cout << "Positioning " << graphMessage << std::endl;
        positionGraph(graphLevel);
    }

    std::cout << "Displaying " << graphMessage << std::endl;

    switchDisplay(*graph_stack[graphLevel], kFromGraphLevel(graphLevel));

}

void Display::positionGraph(unsigned graphLevel) {
    double l = kFromGraphLevel(graphLevel);
    // std::cout << "k: " << l << std::endl;  OKAY
    graph_stack[graphLevel]->init_positions_from_graph(*graph_stack[graphLevel + 1], l);
    graph_stack[graphLevel]->compute_layout(l);

    pair<Vector3D, Vector3D> ep = graph_stack[graphLevel]->compute_extremal_points();
    min_p = ep.first;
    max_p = ep.second;
    double max_extent = max(max_p.x - min_p.x, max(max_p.y - min_p.y, max_p.z - min_p.z));
    //  cout << max_extent << " " << flush;  CHECK AGAIN
    // rescale to -10.0 .. +10.0 on all axes
    Vector3D shift = Vector3D(-1.0, -1.0, -1.0) - 2.0 / max_extent * min_p;
    graph_stack[graphLevel]->rescale(2.0 / max_extent, shift);
}

double Display::kFromGraphLevel(unsigned graphLevel) {
    return k * pow(f_k, graph_stack.size() - graphLevel - 2);
}

template<typename T>
T Display::solve(std::future<T> solverAsync, unsigned freq) {
    std::cout << "Solving..." << std::endl;

    runRerender(freq);

    solverAsync.wait();

    std::cout << "Finished solving" << std::endl;

    return solverAsync.get();
}

future<lbool> Display::solveCMSat() {
    SATSolver s;
    vector<Lit> cmsatClause;

    s.new_vars(graph_stack.front()->nr_nodes());

    for (auto &clause_it : clauses) {
        for (auto &var : clause_it) {
            cmsatClause.emplace_back(abs(var) - 1, var < 0);
        }
        s.add_clause(cmsatClause);
        cmsatClause.clear();
    }

    return std::async(std::launch::async, solveCMSatStatic, s);
}

future<int> Display::solveMaple() {
    return std::async(std::launch::async, solveMapleStatic, filename);
}

int Display::solveMapleStatic(const char *filenamePtr) {
    std::string cmd = "glucose_release -zmq '";
    cmd += filenamePtr;
    cmd += "'";
    return system(nullptr) ? system(cmd.c_str()) : 0;
}

future<int> Display::solveWalksat() {
    return std::async(std::launch::async,
                      solve_walksat,
                      longest_clause,
                      intArrayFromClauseVector(),
                      graph_stack.front()->nr_nodes(),
                      clauses.size());
}

void Display::runRerender(unsigned freq) {
    renderWindowInteractor->ExitCallback();

    auto renderSocketAsync = std::async(std::launch::async, &Display::runRenderSocket, this);

    while (rerender) {
        std::this_thread::sleep_for(std::chrono::milliseconds(freq));
        callRender();
    }

    if (renderSocketAsync.get() == START_INTERACTOR) {
        renderWindowInteractor->Start();
    }
}

void Display::callRender() {
    std::scoped_lock lock{display_mutex};
    renderWindow->Render();
}

int **Display::intArrayFromClauseVector(vector<vector<long>> clauses, unsigned longest_clause) {
    vector<long> clause_it;
    unsigned j;

    auto noOfClauses = clauses.size();

    auto **array = new int *[noOfClauses];

    for (unsigned long i = 0; i < noOfClauses; i++) {
        clause_it = clauses.at(i);
        array[i] = new int[longest_clause];
        for (j = 0; j < clause_it.size(); j++) {
            array[i][j] = clause_it.at(j);
        }
        while (j < longest_clause) {
            array[i][j] = 0;
            j++;
        }
    }

    return array;
}

vector<long> Display::clauseFromCMSATClause(const vector<Lit> &cmsatClause) {
    vector<long> return_clause;
    return_clause.reserve(cmsatClause.size());
    for (auto &lit : cmsatClause) {
        return_clause.push_back(lit.var());
    }
    return return_clause;
}

void Display::renderSocketCheck() {
    while (true) {
        zmq::message_t request;
        if (api_running_socket.recv(request)) {
            api_running_socket.send(request, zmq::send_flags::none);
            return;
        }
    }
}

Display::RENDER_ENUM Display::runRenderSocket() {
    renderSocketCheck();

    while (true) {
        zmq::message_t request;
        zmq::message_t reply;

        // Receive a request from client
        if (render_socket.recv(request)) {
            switch (unpack_render_enum(request)) {
                case EDGES_UPDATE:
//                    current_graph->getGraphToPolyData()->Modified();
//                    break;
                case VERTICES_UPDATE:
//                    current_graph->getVertexPolydata()->Modified();
                    break;
                case START_INTERACTOR:
                    rerender = false;
//                    renderWindowInteractor->Start();
                    return START_INTERACTOR;
                case STOP_INTERACTOR:
                    rerender = true;
                    renderWindowInteractor->ExitCallback();
                    break;
                case CHANGE_GRAPH:
                    change_graph_socket.send(request, zmq::send_flags::none);
                    if (change_graph_socket.recv(reply)) {
                        changeGraph(APIHelper::unpack<unsigned>(reply));
                    }
                    break;

            }
        }
    }
}

void Display::solve(SOLVER_ENUM solver) {
    switch (solver) {
        case CMSAT:
            solve(solveCMSat());
            break;
        case MAPLE:
            solve(solveMaple(), 1000);
            break;
        case MINISAT:
            cout << "MiniSAT" << endl;
            break;
        case WALKSAT:
            solve(solveWalksat(), 25);
            break;
    }
}

// builds (global) stack of coarsened graphs
void Display::init() {
    auto *g = new Graph3D;
    graph_stack.push_back(g);

    // build initial graph
    if (filename == nullptr)
        setupNodes(*g);
    else {
        pair<vector<vector<long>>, unsigned> built;
        if (strncmp("-", filename, 1) == 0) {
            built = g->build_from_cnf(cin);
        } else {
            ifstream is(filename);
            built = g->build_from_cnf(is);
            is.close();
        }
        clauses = built.first;
        longest_clause = built.second;
    }
    cout << "Built initial graph G=(V,E) with |V| = " << g->nr_nodes() << " and |E| = " << g->nr_edges() << "." << endl;

    // check for multiple components
    vector<int> head_nodes;
    int nr_comp = g->independent_components(&head_nodes);
    cout << "Graph consists of " << nr_comp << " independent component(s)." << endl;
    if (nr_comp > 1) {
        cout << "Multiple components not yet supported!" << endl;
        exit(10);
    }

    // build graph stack
    int level = 1;
    while (g->nr_nodes() > 2) {
        g = g->coarsen();
        graph_stack.push_back(g);
        cout << "Coarsened graph " << level << " consists of " << g->nr_nodes()
             << " vertices and " << g->nr_edges() << " edge(s)." << endl;
        level++;
    }

    g->calculate_absolute_variance();

    // compute (random) layout of coarsest graph (with 2 nodes)
    g->init_coarsest_graph_positions(k);
    auto ep = g->compute_extremal_points();
    min_p = ep.first;
    max_p = ep.second;

    display();
}

void Display::display() {
    changeGraph(min((int)graph_stack.size() - 1, 12));

    renderWindow->Render();

    renderWindowInteractor->Start();
}