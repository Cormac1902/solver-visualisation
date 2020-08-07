//
// Created by Cormac on 30-Apr-20.
//

#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkLine.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

#include <utility>
#include <mutex>

#include "Display.h"
#include "Interaction.h"
#include "CryptoWalkSAT.h"
#include "WalkSAT.h"

#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

static double k = 1.0; // old: 5.0
static double f_k = sqrt(4.0 / 7.0);

static bool draw_edges = true, adaptive_size = true, draw_only_2clauses = false; // run_anim = falseClauses

static float min_line_width = 3, max_line_width = 12;
static int max_line_width_threshold = 250, min_line_width_threshold = 2500;

std::mutex graph_mutex;

// ----------------------------------------------------------------------

void Display::display(Interaction &interaction) {

//    changeGraph(0);

    edgeActor->SetMapper(edgeMapper);

    edgeActor->GetProperty()->EdgeVisibilityOn();
    edgeActor->GetProperty()->RenderLinesAsTubesOn();
//    edgeActor->GetProperty()->RenderPointsAsSpheresOn();
//    edgeActor->GetProperty()->VertexVisibilityOn();

    edgeMapper->InterpolateScalarsBeforeMappingOn();
    edgeMapper->ScalarVisibilityOn();

    vertexActor->SetMapper(vertexMapper);

    sphereSource->SetThetaResolution(100);
    sphereSource->SetPhiResolution(100);

    glyph3D->SetScaleModeToScaleByScalar();
    glyph3D->SetSourceConnection(sphereSource->GetOutputPort());

    vertexMapper->InterpolateScalarsBeforeMappingOn();
    vertexMapper->ScalarVisibilityOn();

    vertexMapper->SetInputConnection(glyph3D->GetOutputPort());
    vertexMapper->SetScalarModeToUsePointFieldData();
    vertexMapper->SelectColorArray(1);

    // edgeActor->GetProperty()->BackfaceCullingOn();

    auto camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(0, 0, 20);
    camera->SetFocalPoint(0, 0, 0);

    auto renderer = vtkSmartPointer<vtkRenderer>::New();

    renderer->SetActiveCamera(camera);

    renderWindow->AddRenderer(renderer);
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderWindowInteractor->SetInteractorStyle(&interaction);
    interaction.SetCurrentRenderer(renderer);

    renderWindow->SetSize(1920, 1080);
    renderWindow->SetWindowName("Solver Visualisation");
    renderWindow->ShowCursor();

    renderer->AddActor(edgeActor);
    renderer->AddActor(vertexActor);

    switchDisplay(graph_stack.back(), k);

    renderWindow->Render();

    renderWindowInteractor->Start();

}

void Display::switchDisplay(Graph3D *g, double l) {

    auto highestEdgeDuplication = g->highestEdgeDuplication();

    // g->drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size);
    if (!g->get_drawn()) {
        g->drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size);
    }

    edgeMapper->SetInputConnection(g->getGraphToPolyData()->GetOutputPort());
//    edgeMapper->SetInputConnection(g->drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size)->GetOutputPort());

    if (g->highestEdgeDuplication() != highestEdgeDuplication) {
        g->reColour();
    }

    auto numberOfLines = edgeMapper->GetInput()->GetNumberOfLines();

    auto lineWidth = min(max_line_width,
                         ((float) (min_line_width_threshold - max_line_width_threshold) / numberOfLines) +
                         min_line_width);

    edgeActor->GetProperty()->SetLineWidth(lineWidth);
//    edgeActor->GetProperty()->SetPointSize(lineWidth * 3);

    sphereSource->SetRadius(lineWidth * 3 / 1000);

    current_graph = g;

    glyph3D->SetInputData(g->getVertexPolydata());
    glyph3D->Update();
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

    edgeMapper->Update();
    renderWindow->Render();
//    renderWindowInteractor->Render();
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

    vertexMapper->Update();

    renderWindow->Render();
//    renderWindowInteractor->Render();
//    glyph3D->Update();
}

void Display::increaseVariableActivity(unsigned long i) {
    increaseVariableActivity(current_graph, i);
}

void Display::assignVariable(Graph3D *g, unsigned long i, bool value, bool undef) {
    std::scoped_lock lock{graph_mutex};

    g->assign_variable_truth_value(i, value, undef);

    vertexMapper->Update();

    renderWindow->Render();
//    renderWindowInteractor->Render();
}

void Display::assignVariable(unsigned long i, bool value, bool undef) {
    assignVariable(current_graph, i, value, undef);
}

void Display::setupNodes(Graph3D *g) {
    Node3D n(1), m(2), o(3);

    g->add_node(n);
    g->add_node(m);
    g->add_node(o);
    g->insert_edge(1, 2);
    g->insert_edge(1, 3);
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

    switchDisplay(graph_stack[graphLevel], kFromGraphLevel(graphLevel));

    renderWindow->Render();

}


void Display::positionGraph(unsigned int graphLevel) {
    double l = kFromGraphLevel(graphLevel);
    // std::cout << "k: " << l << std::endl;  OKAY
    graph_stack[graphLevel]->init_positions_from_graph(graph_stack[graphLevel + 1], l);
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

double Display::kFromGraphLevel(unsigned int graphLevel) {
    return k * pow(f_k, graph_stack.size() - graphLevel - 2);
}

void Display::solve() {

    SATSolver s;
    vector<Lit> cmsatClause;

//    s.set_num_threads(4);

    //fill the solver, run solve_walksat, etc.

    s.new_vars(graph_stack[0]->nr_nodes());

    for (auto &clause_it : clauses) {
        for (auto &var : clause_it) {
            cmsatClause.emplace_back(abs(var) - 1, var < 0);
        }
        s.add_clause(cmsatClause);
        cmsatClause.clear();
    }

    std::cout << "Solving..." << std::endl;

    s.solve();

//    vector<Lit> lits;
//    bool ret = true;
//    while (ret) {
//        ret = s.get_next_small_clause(lits);
//        if (ret) {
//            //deal with clause_array in "lits"
//            // add_to_my_db(lits);
//            addEdgesFromClause(current_graph, clauseFromCMSATClause(lits));
//        }
//    }

    std::cout << "Finished solving" << std::endl;

//    s.end_getting_small_clauses();

}

int Display::walksat(Display *display) {
    std::cout << "Solving..." << std::endl;

    return solve_walksat(display->longest_clause,
                  intArrayFromClauseVector(display->clauses, display->longest_clause),
                  display->graph_stack.front()->nr_nodes(),
                  display->clauses.size());
}

int **Display::intArrayFromClauseVector(vector<vector<long>> clauses, unsigned int longest_clause) {
    vector<long> clause_it;
    unsigned int j;

    unsigned long noOfClauses = clauses.size();

    int **array = new int *[noOfClauses];

    for (unsigned int i = 0; i < noOfClauses; i++) {
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

// builds (global) stack of coarsened graphs
void Display::init(char *filename, Interaction *interaction) {
    auto *g = new Graph3D();
    graph_stack.push_back(g);

    // build initial graph
    if (filename == nullptr)
        setupNodes(g);
    else {
        pair<vector<vector<long>>, unsigned int> built;
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
    cout << "Built initial graph G=(V,E) with |V| = " << g->nr_nodes() << " and |E| = "
         << g->nr_edges() << "." << endl;

    // check for multiple components
    vector<int> head_nodes;
    int nr_comp = g->independent_components(&head_nodes);
    cout << "Graph consists of " << nr_comp << " independent component(s)." << endl;
    if (nr_comp > 1) {
        cout << "Multiple components not yet supported!" << endl;
        exit(10);
    }

    // build graph stack
    Graph3D *a = g, *b;
    int level = 1;
    while (a->nr_nodes() > 2) {
        b = a->coarsen();
        graph_stack.push_back(b);
        cout << "Coarsened graph " << level << " consists of " << b->nr_nodes()
             << " vertices and " << b->nr_edges() << " edge(s)." << endl;
        a = b;
        level++;
    }

    graph_stack.back()->calculate_absolute_variance();

    // compute (random) layout of coarsest graph (with 2 nodes)
    graph_stack.back()->init_coarsest_graph_positions(k);
    pair<Vector3D, Vector3D> ep = graph_stack.back()->compute_extremal_points();
    min_p = ep.first;
    max_p = ep.second;

    display(*interaction);
}