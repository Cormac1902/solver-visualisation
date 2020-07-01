//
// Created by corma on 30-Apr-20.
//

#ifndef INC_3DVIS_DISPLAY_H
#define INC_3DVIS_DISPLAY_H

#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include "Graph.h"
#include <cryptominisat5/cryptominisat.h>

using namespace CMSat;

class Display {
    static vtkPolyDataMapper* mapper;
    static vtkActor* actor;
    static vtkRenderWindow* renderWindow;
    static vector<vector<long>> clauses;

    static void display();

    static void setupNodes(Graph3D *g);

    static void switchDisplay(Graph3D *g, double l);

    static void changeGraphFromClause(Graph3D *g, vector<long> clause, bool add = true);

    static void addEdgesFromClause(Graph3D *g, vector<long> clause);

    static void removeEdgesFromClause(Graph3D *g, vector<long> clause);

    static void changeGraph(unsigned graphLevel);

    static void positionGraph(unsigned graphLevel);

    static double kFromGraphLevel(unsigned graphLevel);

    static void solve();

    static vector<long> clauseFromCMSATClause(const vector<Lit>& cmsatClause);

    friend class Interaction;

public:
    static void init(char *filename);

    static void addEdgesFromClause(vector<long> clause);

    static void removeEdgesFromClause(vector<long> clause);
};


#endif //INC_3DVIS_DISPLAY_H
