//
// Created by Cormac on 24-Apr-20.
//

#ifndef INC_3DVIS_INTERACTION_H
#define INC_3DVIS_INTERACTION_H

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include "vtkRenderWindowInteractor.h"


class Interaction : public vtkInteractorStyleTrackballCamera {
public:
    static Interaction *New();

vtkTypeMacro(Interaction, vtkInteractorStyleTrackballCamera);

    void OnKeyPress() override;
};

#endif //INC_3DVIS_INTERACTION_H
