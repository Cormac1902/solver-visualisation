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

    Interaction& interaction;

    zmq::context_t context;
    zmq::socket_t render_socket;

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
            render_socket(context, ZMQ_PULL) {
        interaction.setDisplay(this);
        render_socket.bind(APIHelper::port_string(RENDER_SOCKET));
    }

    static unsigned RENDER_SOCKET;
    static unsigned EDGES_UPDATE;
    static unsigned VERTICES_UPDATE;

    void init(char *filename);

    void startDisplayInteractor();

    void addEdgesFromClause(vector<long> clause);

    void removeEdgesFromClause(vector<long> clause);

    void increaseVariableActivity(unsigned long i);

    void assignVariable(unsigned long i, bool value, bool undef = false);

    [[noreturn]] void run_render_socket();

    static int walksat(Display *display);
};


#endif //INC_3DVIS_DISPLAY_H
