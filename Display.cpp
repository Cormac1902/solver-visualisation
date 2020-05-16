//
// Created by Cormac on 30-Apr-20.
//

#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkLine.h>

#include "Display.h"
#include "Graph.h"
#include "Interaction.h"

float sphi = 0.0, stheta = 0.0;
float zoom = 1.0;
float zNear = 0.1, zFar = 20.0;
int downX, downY;
bool leftButton = false, middleButton = false, rightButton = false;

float xOffset = 0.0, yOffset = 0.0;

Graph3D *current_graph; // currently displayed graph
vector<Graph3D *> graph_stack;

float k = 1.0; // old: 5.0
float f_k = sqrt(4.0 / 7.0);
int curr_L;

bool next_graph = false, draw_edges = true, run_anim = false, adaptive_size = true;
bool draw_only_2clauses = false;

Vector3D min_p, max_p;

//vtkSmartPointer<vtkVertexGlyphFilter> Display::vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
//vtkSmartPointer<vtkPolyData> Display::polydata = vtkSmartPointer<vtkPolyData>::New();
vtkSmartPointer<vtkPolyDataMapper> Display::mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkRenderWindow> Display::renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

// ----------------------------------------------------------------------

void Display::display() {
    if (current_graph != nullptr) {

        switchDisplay();

        vtkSmartPointer<vtkActor> actor =
                vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        vtkSmartPointer<vtkNamedColors> colors =
                vtkSmartPointer<vtkNamedColors>::New();

        // actor->GetProperty()->SetColor(colors->GetColor3d("cyan_white").GetData());

        actor->GetProperty()->EdgeVisibilityOn();
        actor->GetProperty()->SetLineWidth(12);
        actor->GetProperty()->SetPointSize(36);
        actor->GetProperty()->RenderLinesAsTubesOn();
        actor->GetProperty()->RenderPointsAsSpheresOn();
        actor->GetProperty()->VertexVisibilityOn();
        // actor->GetProperty()->SetVertexColor(0.5,1.0,0.8);

        // actor->GetProperty()->BackfaceCullingOn();

        vtkSmartPointer<vtkCamera> camera =
                vtkSmartPointer<vtkCamera>::New();
        camera->SetPosition(0, 0, 20);
        camera->SetFocalPoint(0, 0, 0);

        vtkSmartPointer<vtkRenderer> renderer =
                vtkSmartPointer<vtkRenderer>::New();

        renderer->SetActiveCamera(camera);

        renderWindow->AddRenderer(renderer);
        vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
                vtkSmartPointer<vtkRenderWindowInteractor>::New();
        renderWindowInteractor->SetRenderWindow(renderWindow);

        Interaction* interactor = Interaction::New();

        vtkSmartPointer<Interaction> interactorPointer = vtkSmartPointer<Interaction>::Take(interactor);
        renderWindowInteractor->SetInteractorStyle(interactor);
        interactor->SetCurrentRenderer(renderer);

        renderWindow->SetSize(1920, 1080);
        renderWindow->SetWindowName("ReadPolyData");
        renderWindow->ShowCursor();

        renderer->AddActor(actor);
//        renderWindow->Render();

        renderWindow->Render();

        renderWindowInteractor->Start();

    }
}

void Display::switchDisplay() {
    if (current_graph != nullptr) {
        mapper->SetInputData(vtkSmartPointer<vtkPolyData>
                             ::Take(current_graph->drawVTP(k, draw_edges, draw_only_2clauses, adaptive_size)));
    }
}

void Display::setupNodes(Graph3D *g) {
    Node3D n(1), m(2), o(3);

    g->add_node(n);
    g->add_node(m);
    g->add_node(o);
    g->insert_edge(1, 2);
    g->insert_edge(1, 3);
}

// builds (global) stack of coarsened graphs
void Display::init(char *filename) {
    auto *g = new Graph3D();
    g->set_all_matching({});
    graph_stack.push_back(g);

    // build initial graph
    if (filename == nullptr)
        setupNodes(g);
    else {
        if (strncmp("-", filename, 1) == 0)
            g->build_from_cnf(cin);
        else {
            ifstream is(filename);
            g->build_from_cnf(is);
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
    curr_L = (int) graph_stack.size() - 1;
    graph_stack[curr_L]->init_coarsest_graph_positions(k);
    pair<Vector3D, Vector3D> ep = graph_stack[curr_L]->compute_extremal_points();
    min_p = ep.first;
    max_p = ep.second;
    current_graph = graph_stack[curr_L];
    curr_L--;

    for (int i = curr_L; i >= 0; i--) {
        graph_stack[i]->init_positions_from_graph(graph_stack[i + 1], k);
        graph_stack[i]->compute_layout(k);
        k *= f_k;
    }

    k = 1.0;

    display();
}

void Display::changeGraph(unsigned i) {

    std::string graphMessage;

    if (i == 0) {
        graphMessage = "non-coarsened graph";
    } else {
        graphMessage = "coarsened graph ";
        graphMessage.append(std::to_string(i));
        if (graph_stack.size() <= i) {
            std::cout << "No " << graphMessage << std::endl;
            return;
        }
    }
    std::cout << "Displaying " << graphMessage << std::endl;

            // layout & display next finer graph
    // graph_stack[i]->init_positions_from_graph(graph_stack[i + 1], k);
    //      graph_stack[curr_L]->init_positions_at_random();
//    graph_stack[i]->compute_layout(k);
    pair<Vector3D, Vector3D> ep = graph_stack[i]->compute_extremal_points();
    min_p = ep.first;
    max_p = ep.second;
    float max_extent = max(max_p.x - min_p.x, max(max_p.y - min_p.y, max_p.z - min_p.z));
    //  cout << max_extent << " " << flush;
    // rescale to -10.0 .. +10.0 on all axes
    Vector3D shift = Vector3D(-1.0, -1.0, -1.0) - 2.0 / max_extent * min_p;
    graph_stack[i]->rescale((float) 2.0 / max_extent, shift);

    k = pow(f_k, graph_stack.size() - i - 1);

    //graph_stack[i]->drawVTP(k, draw_edges, draw_only_2clauses, adaptive_size);
    current_graph = graph_stack[i];
    switchDisplay();

    renderWindow->Render();

}