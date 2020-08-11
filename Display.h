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

using namespace CMSat;

class API;

class Display {
    std::mutex graph_mutex;

    vector<Graph3D *> graph_stack;
    Graph3D *current_graph{};

    vtkSmartPointer<vtkPolyDataMapper> edgeMapper;
    vtkSmartPointer<vtkActor> edgeActor;

    vtkSmartPointer<vtkPolyDataMapper> vertexMapper;
    vtkSmartPointer<vtkActor> vertexActor;
    vtkSmartPointer<vtkGlyph3D> glyph3D;

    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

    vtkSmartPointer<vtkSphereSource> sphereSource;

    vector<vector<long>> clauses;
    unsigned int longest_clause{};

    Vector3D min_p, max_p;

    Interaction &interaction;

    zmq::context_t context;
    zmq::socket_t render_socket;
    zmq::socket_t change_graph_socket;
    zmq::socket_t api_running_socket;

    void display();

    static void setupNodes(Graph3D *g);

    void switchDisplay(Graph3D *g, double l);

    void changeGraphFromClause(Graph3D *g, vector<long> clause, bool add = true);

    void addEdgesFromClause(Graph3D *g, vector<long> clause);

    void removeEdgesFromClause(Graph3D *g, vector<long> clause);

    void increaseVariableActivity(Graph3D *g, unsigned long i);

    void assignVariable(Graph3D *g, unsigned long i, bool value, bool undef = false);

    void changeGraph(unsigned graphLevel);

    void positionGraph(unsigned graphLevel);

    double kFromGraphLevel(unsigned graphLevel);

    void solve();

    static vector<long> clauseFromCMSATClause(const vector<Lit> &cmsatClause);

    static int **intArrayFromClauseVector(vector<vector<long>> clauses, unsigned int longest_clause);

    friend class Interaction;

public:
    Display() :
            graph_stack({}),
            edgeMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
            edgeActor(vtkSmartPointer<vtkActor>::New()),
            vertexMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
            vertexActor(vtkSmartPointer<vtkActor>::New()),
            glyph3D(vtkSmartPointer<vtkGlyph3D>::New()),
            renderer(vtkSmartPointer<vtkRenderer>::New()),
            renderWindow(vtkSmartPointer<vtkRenderWindow>::New()),
            renderWindowInteractor(vtkSmartPointer<vtkRenderWindowInteractor>::New()),
            sphereSource(vtkSmartPointer<vtkSphereSource>::New()),
            clauses({}),
            longest_clause(0),
            interaction(*new Interaction),
            context(1),
            render_socket(context, ZMQ_PULL),
            change_graph_socket(context, ZMQ_REQ),
            api_running_socket(context, ZMQ_REP) {
        interaction.setDisplay(this);
        APIHelper::bind(render_socket, RENDER_SOCKET);
        APIHelper::connect(change_graph_socket, CHANGE_GRAPH_SOCKET);
        APIHelper::bind(api_running_socket, API_RUNNING_SOCKET);
        /*render_socket.bind(APIHelper::bind_string(RENDER_SOCKET));
        change_graph_socket.connect(APIHelper::connect_string(CHANGE_GRAPH_SOCKET));*/
    }

    static unsigned RENDER_SOCKET;
    static unsigned CHANGE_GRAPH_SOCKET;
    static unsigned API_RUNNING_SOCKET;
    enum RENDER_ENUM {
        EDGES_UPDATE, VERTICES_UPDATE, START_INTERACTOR, STOP_INTERACTOR, CHANGE_GRAPH
    };

    void init(char *filename);

    void addEdgesFromClause(vector<long> clause);

    void removeEdgesFromClause(vector<long> clause);

    void increaseVariableActivity(unsigned long i);

    void assignVariable(unsigned long i, bool value, bool undef = false);

    unsigned int graphStackSize() { return graph_stack.size(); }

    Interaction &getInteraction() { return interaction; }

    void run_render_socket();

    static int walksat(Display *display, API* api);

    static RENDER_ENUM unpack_render_enum(zmq::message_t &message) {
            auto enumInt = APIHelper::unpack<unsigned int>(message);
            if (enumInt >= '0') {
                enumInt -= '0';
            }
        return static_cast<RENDER_ENUM>(enumInt);
    }
};


#endif //INC_3DVIS_DISPLAY_H
