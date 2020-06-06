//
// Created by corma on 30-Apr-20.
//

#ifndef INC_3DVIS_DISPLAY_H
#define INC_3DVIS_DISPLAY_H

#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include "Graph.h"

class Display {
    static vtkPolyDataMapper* mapper;
    static vtkActor* actor;
    static vtkRenderWindow* renderWindow;

    static void display();

    static void setupNodes(Graph3D *g);

    static void switchDisplay(Graph3D *g, double l);

    static void changeGraph(unsigned graphLevel);

    static void positionGraph(unsigned graphLevel);

    static double kFromGraphLevel(unsigned graphLevel);

    friend class Interaction;

public:
    static void init(char *filename);
};


#endif //INC_3DVIS_DISPLAY_H
