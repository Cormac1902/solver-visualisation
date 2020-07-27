//
// Created by corma on 30-Apr-20.
//walking sa appy

#ifndef INC_3DVIS_DISPLAY_H
#define INC_3DVIS_DISPLAY_H

#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkGlyph3D.h>
#include <vtkSphereSource.h>
#include "Graph.h"
#include <cryptominisat5/cryptominisat.h>
#include "cryptominisat5/dimacsparser.h"

using namespace CMSat;

class Display {
    static vtkSmartPointer<vtkPolyDataMapper> edgeMapper;
    static vtkSmartPointer<vtkActor> edgeActor;

    static vtkSmartPointer<vtkPolyDataMapper> vertexMapper;
    static vtkSmartPointer<vtkActor> vertexActor;
    static vtkSmartPointer<vtkGlyph3D> glyph3D;

    static vtkSmartPointer<vtkRenderWindow> renderWindow;

    static vtkSmartPointer<vtkSphereSource> sphereSource;

    static vector<vector<long>> clauses;
    static unsigned int longest_clause;

    static void display();

    static void setupNodes(Graph3D *g);

    static void switchDisplay(Graph3D *g, double l);

    static void changeGraphFromClause(Graph3D *g, vector<long> clause, bool add = true);

    static void addEdgesFromClause(Graph3D *g, vector<long> clause);

    static void removeEdgesFromClause(Graph3D *g, vector<long> clause);

    static void increaseVariableActivity(Graph3D *g, unsigned long i);

    static void changeGraph(unsigned graphLevel);

    static void positionGraph(unsigned graphLevel);

    static double kFromGraphLevel(unsigned graphLevel);

    static void solve();

    static void walksat();

    static vector<long> clauseFromCMSATClause(const vector<Lit>& cmsatClause);

    static int** intArrayFromClauseVector();

    friend class Interaction;

public:
    static void init(char *filename);

    static void addEdgesFromClause(vector<long> clause);

    static void removeEdgesFromClause(vector<long> clause);

    static void increaseVariableActivity(unsigned long i);
};


#endif //INC_3DVIS_DISPLAY_H
