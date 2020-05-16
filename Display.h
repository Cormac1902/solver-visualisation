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
    static vtkSmartPointer<vtkPolyDataMapper> mapper;
    static vtkSmartPointer<vtkRenderWindow> renderWindow;

    static void display();
    static void switchDisplay();
    static void setupNodes(Graph3D *g);
    static void changeGraph(unsigned i);

    friend class Interaction;

public:
    static void init(char *filename);
};


#endif //INC_3DVIS_DISPLAY_H
