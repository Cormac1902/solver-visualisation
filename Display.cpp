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
#include <vtkPointSource.h>

#include <utility>

#include "Display.h"
#include "Interaction.h"

#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

vector<Graph3D *> graph_stack;

double k = 1.0; // old: 5.0
double f_k = sqrt(4.0 / 7.0);

bool draw_edges = true, adaptive_size = true; // run_anim = false
bool draw_only_2clauses = false;

float min_line_width = 3, max_line_width = 12;
int max_line_width_threshold = 250, min_line_width_threshold = 2500;

Vector3D min_p, max_p;

Graph3D* current_graph;

vtkPolyDataMapper *Display::mapper = vtkPolyDataMapper::New();
vtkActor *Display::actor = vtkActor::New();
vtkRenderWindow *Display::renderWindow = vtkRenderWindow::New();
vector<vector<long>> Display::clauses = {};

// ----------------------------------------------------------------------

void Display::display() {

//    changeGraph(0);

    switchDisplay(graph_stack.back(), k);

    actor->SetMapper(mapper);

    actor->GetProperty()->EdgeVisibilityOn();
    actor->GetProperty()->RenderLinesAsTubesOn();
    actor->GetProperty()->RenderPointsAsSpheresOn();
    actor->GetProperty()->VertexVisibilityOn();

    mapper->InterpolateScalarsBeforeMappingOn();
    mapper->ScalarVisibilityOn();

    // actor->GetProperty()->BackfaceCullingOn();

    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(0, 0, 20);
    camera->SetFocalPoint(0, 0, 0);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

    renderer->SetActiveCamera(camera);

    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    Interaction *interactor = Interaction::New();

    vtkSmartPointer<Interaction> interactorPointer = vtkSmartPointer<Interaction>::Take(interactor);
    renderWindowInteractor->SetInteractorStyle(interactor);
    interactor->SetCurrentRenderer(renderer);

    renderWindow->SetSize(1920, 1080);
    renderWindow->SetWindowName("Solver Visualisation");
    renderWindow->ShowCursor();

    renderer->AddActor(actor);

    renderWindow->Render();

    renderWindowInteractor->Start();

}

void Display::switchDisplay(Graph3D *g, double l) {

    auto highestEdgeDuplication = g->getHighestEdgeDuplication();

    // g->drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size);
    g->drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size);
    mapper->SetInputConnection(g->getGraphToPolyData()->GetOutputPort());
//    mapper->SetInputConnection(g->drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size)->GetOutputPort());

    if (g->getHighestEdgeDuplication() != highestEdgeDuplication) {
        g->reColour();
    }

    float numberOfLines = mapper->GetInput()->GetNumberOfLines();

    auto lineWidth = min(max_line_width,
                         ((float) (min_line_width_threshold - max_line_width_threshold) / numberOfLines) +
                         min_line_width);

    actor->GetProperty()->SetLineWidth(lineWidth);
    actor->GetProperty()->SetPointSize(lineWidth * 3);

    current_graph = g;

/*    vtkSmartPointer<vtkPointSource> pointSource =
            vtkSmartPointer<vtkPointSource>::New();
    pointSource->SetNumberOfPoints(mapper->GetInput()->GetNumberOfPoints());
    pointSource->SetInputConnection(g->drawPolyData(l, draw_edges, draw_only_2clauses, adaptive_size)->GetOutputPort());
    pointSource->Update();

    mapper->SetInputConnection(pointSource->GetOutputPort());*/

    /*vtkSmartPointer<vtkNamedColors> namedColors =
            vtkSmartPointer<vtkNamedColors>::New();

    vtkSmartPointer<vtkUnsignedCharArray> colors =
            vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName ("Colors");
    colors->InsertNextTupleValue(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTupleValue(namedColors->GetColor3ub("Mint").GetData());
    colors->InsertNextTupleValue(namedColors->GetColor3ub("Peacock").GetData());

    mapper->GetInput()->GetPointData()->AddArray(colors);*/

}

void Display::addEdgesFromClause(Graph3D *g, vector<long> clause) {
    auto highestEdgeDuplication = g->getHighestEdgeDuplication();

    g->add_graph_edges_from_clause(std::move(clause));

    if (g->getHighestEdgeDuplication() != highestEdgeDuplication) {
        g->reColour();
    }

//    g->getGraphToPolyData()->SetInputData(g->getGraph());
//    g->getGraphToPolyData()->Update();
//    renderWindow->Render();
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

    //fill the solver, run solve, etc.

    s.new_vars(graph_stack[0]->nr_nodes());

    for (auto &clause : clauses) {
        for (auto &var : clause) {
            cmsatClause.emplace_back(abs(var) - 1, var < 0);
        }
        s.add_clause(cmsatClause);
        cmsatClause.clear();
    }

    //Get all clauses of size 4 or less

    s.solve();

    s.start_getting_small_clauses(UINT32_MAX, UINT32_MAX);

    vector<Lit> lits;
    bool ret = true;
    while (ret) {
        ret = s.get_next_small_clause(lits);
        if (ret) {
            //deal with clause in "lits"
            // add_to_my_db(lits);
            addEdgesFromClause(current_graph, clauseFromCMSATClause(lits));
        }
    }

    renderWindow->Render();

    cout << "Finished solving" << endl;

    s.end_getting_small_clauses();

}

vector<long> Display::clauseFromCMSATClause(const vector<Lit> &cmsatClause) {
    vector<long> clause;
    clause.reserve(cmsatClause.size());
    for (auto &lit : cmsatClause) {
        clause.push_back(lit.var());
    }
    return clause;
}

// builds (global) stack of coarsened graphs
void Display::init(char *filename) {
    auto *g = new Graph3D();
    graph_stack.push_back(g);

    // build initial graph
    if (filename == nullptr)
        setupNodes(g);
    else {
        if (strncmp("-", filename, 1) == 0)
            clauses = g->build_from_cnf(cin);
        else {
            ifstream is(filename);
            clauses = g->build_from_cnf(is);
            is.close();
        }
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

    // compute (random) layout of coarsest graph (with 2 nodes)
    graph_stack.back()->init_coarsest_graph_positions(k);
    pair<Vector3D, Vector3D> ep = graph_stack.back()->compute_extremal_points();
    min_p = ep.first;
    max_p = ep.second;

    display();
}