//
// Created by Cormac on 30-Apr-20.

#ifndef INC_3DVIS_DISPLAY_H
#define INC_3DVIS_DISPLAY_H

#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGlyph3D.h>
#include <vtkSphereSource.h>
#include <vtkActor.h>
#include "Graph.h"
#include <cryptominisat5/cryptominisat.h>
#include "cryptominisat5/dimacsparser.h"
#include "Interaction.h"
#include <mutex>
#include <zmq.hpp>
#include "APIHelper.hpp"
#include "vtkProperty.h"
#include <vtkCamera.h>
#include <thread>
#include <future>

using namespace CMSat;

class API;

class Display {
    char *filename;

    std::mutex graph_mutex;
    std::mutex display_mutex;

    vector<Graph3D*> graph_stack;
    Graph3D *current_graph;

    vtkSmartPointer<vtkPolyDataMapper> edgeMapper;
    vtkSmartPointer<vtkActor> edgeActor;

    vtkSmartPointer<vtkSphereSource> sphereSource;
    vtkSmartPointer<vtkGlyph3D> glyph3D;
    vtkSmartPointer<vtkPolyDataMapper> vertexMapper;
    vtkSmartPointer<vtkActor> vertexActor;

    vtkSmartPointer<vtkCamera> camera;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
    Interaction interaction;

    vector<vector<long>> clauses;
    unsigned longest_clause{};
    bool rerender;

    Vector3D min_p, max_p;

    zmq::context_t context;
    zmq::socket_t render_socket;
    zmq::socket_t change_graph_socket;
    zmq::socket_t api_running_socket;

    void runRerender();

    void callRender();

    static void setupNodes(Graph3D& g);

    void switchDisplay(Graph3D& g, double l);

    void changeGraphFromClause(Graph3D *g, vector<long> clause, bool add = true);

    void addEdgesFromClause(Graph3D *g, vector<long> clause);

    void removeEdgesFromClause(Graph3D *g, vector<long> clause);

    void increaseVariableActivity(Graph3D *g, unsigned long i);

    void assignVariable(Graph3D *g, unsigned long i, bool value, bool undef = false);

    void changeGraph(unsigned graphLevel);

    void positionGraph(unsigned graphLevel);

    double kFromGraphLevel(unsigned graphLevel);

    inline int **intArrayFromClauseVector() { return intArrayFromClauseVector(clauses, longest_clause); }

    static vector<long> clauseFromCMSATClause(const vector<Lit> &cmsatClause);

    static int **intArrayFromClauseVector(vector<vector<long>> clauses, unsigned longest_clause);

    void renderSocketCheck();

    void addEdgesFromClause(vector<long> clause);

    void removeEdgesFromClause(vector<long> clause);

    void increaseVariableActivity(unsigned long i);

    void assignVariable(unsigned long i, bool value, bool undef = false);

    template<typename T>
    T solve(std::future<T> solverAsync);

    future<int> solveWalksat();

    future<lbool> solveCMSat();

    future<int> solveMaple();

    static inline lbool solveCMSatStatic(SATSolver s) { return s.solve(); }

    static int solveMapleStatic(const char *filenamePtr);

    friend class Interaction;

    friend class API;

public:
    enum RENDER_ENUM {
        EDGES_UPDATE, VERTICES_UPDATE, START_INTERACTOR, STOP_INTERACTOR, CHANGE_GRAPH
    };

    enum SOLVER_ENUM {
        CMSAT = 'C',
        MAPLE = 'G',
        MINISAT = 'M',
        WALKSAT = 'W'
    };

private:
    RENDER_ENUM runRenderSocket();

    static inline RENDER_ENUM unpack_render_enum(zmq::message_t &message) {
        auto enumInt = APIHelper::unpack<unsigned>(message);
        if (enumInt >= '0') {
            enumInt -= '0';
        }
        return static_cast<RENDER_ENUM>(enumInt);
    }

    static inline SOLVER_ENUM solver_enum_from_char(char ch) {
        return static_cast<SOLVER_ENUM>(ch);
    }

public:
    explicit Display(char *filenamePtr = nullptr) :
            filename(filenamePtr),
            current_graph(nullptr),
            edgeMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
            edgeActor(vtkSmartPointer<vtkActor>::New()),
            sphereSource(vtkSmartPointer<vtkSphereSource>::New()),
            glyph3D(vtkSmartPointer<vtkGlyph3D>::New()),
            vertexMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
            vertexActor(vtkSmartPointer<vtkActor>::New()),
            camera(vtkSmartPointer<vtkCamera>::New()),
            renderer(vtkSmartPointer<vtkRenderer>::New()),
            renderWindow(vtkSmartPointer<vtkRenderWindow>::New()),
            renderWindowInteractor(vtkSmartPointer<vtkRenderWindowInteractor>::New()),
            interaction(),
            longest_clause(0),
            rerender(true),
            context(1),
            render_socket(context, ZMQ_PULL),
            change_graph_socket(context, ZMQ_REQ),
            api_running_socket(context, ZMQ_REP) {

        APIHelper::bind(render_socket, RENDER_SOCKET);
        APIHelper::connect(change_graph_socket, CHANGE_GRAPH_SOCKET);
        APIHelper::bind(api_running_socket, API_RUNNING_SOCKET);

        edgeMapper->InterpolateScalarsBeforeMappingOn();
        edgeMapper->ScalarVisibilityOn();

        edgeActor->SetMapper(edgeMapper);
        edgeActor->GetProperty()->EdgeVisibilityOn();
        edgeActor->GetProperty()->RenderLinesAsTubesOn();

        sphereSource->SetThetaResolution(100);
        sphereSource->SetPhiResolution(100);

        glyph3D->SetScaleModeToScaleByScalar();
        glyph3D->SetSourceConnection(sphereSource->GetOutputPort());

        vertexMapper->InterpolateScalarsBeforeMappingOn();
        vertexMapper->ScalarVisibilityOn();

        vertexMapper->SetInputConnection(glyph3D->GetOutputPort());
        vertexMapper->SetScalarModeToUsePointFieldData();
        vertexMapper->SelectColorArray(1);

        vertexActor->SetMapper(vertexMapper);

        camera->SetPosition(0, 0, 10);
        camera->SetFocalPoint(0, 0, 0);

        renderer->SetActiveCamera(camera);

        renderWindow->AddRenderer(renderer);

        renderWindow->SetSize(1920 * 2, 1080 * 2);
        renderWindow->SetWindowName("Solver Visualisation");
        renderWindow->ShowCursor();

        renderer->AddActor(edgeActor);
        renderer->AddActor(vertexActor);

        interaction.setDisplay(this);
        interaction.SetCurrentRenderer(renderer);
        renderWindowInteractor->SetInteractorStyle(&interaction);
        renderWindowInteractor->SetRenderWindow(renderWindow);
    }

    static unsigned RENDER_SOCKET;
    static unsigned CHANGE_GRAPH_SOCKET;
    static unsigned API_RUNNING_SOCKET;

    void init();

    void display();

    unsigned graphStackSize() { return graph_stack.size(); }

    Interaction &getInteraction() { return interaction; }

    std::mutex &getDisplayMutex() { return display_mutex; }

    void solve(SOLVER_ENUM solver);
};


#endif //INC_3DVIS_DISPLAY_H
