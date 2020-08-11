//
// Created by Cormac on 24-Apr-20.
//

#ifndef INC_3DVIS_INTERACTION_H
#define INC_3DVIS_INTERACTION_H

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include "vtkRenderWindowInteractor.h"

class Display;

class API;

class Interaction : public vtkInteractorStyleTrackballCamera {
public:
    Display *display;
    API *api;

    static Interaction *New();

    void setDisplay(Display *displayRef) {
        display = displayRef;
    }

    void setAPI(API *apiRef) {
        api = apiRef;
    }

vtkTypeMacro(Interaction, vtkInteractorStyleTrackballCamera);

    void OnKeyPress() override;
};

#endif //INC_3DVIS_INTERACTION_H
