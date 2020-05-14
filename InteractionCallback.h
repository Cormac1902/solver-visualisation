//
// Created by Cormac on 25-Apr-20.
//

#ifndef INC_3DVIS_INTERACTIONCALLBACK_H
#define INC_3DVIS_INTERACTIONCALLBACK_H


#include <vtkCallbackCommand.h>
#include <vtkRenderWindowInteractor.h>
#include "Interaction.h"

class InteractionCallback : public vtkCallbackCommand
{
public:
    static InteractionCallback* New() { return new InteractionCallback; }
    // Here we Create a vtkCallbackCommand and reimplement it.
    void Execute(vtkObject* caller, unsigned long evId, void*) override;

    InteractionCallback()
            : interactor(nullptr)
    {
    }
    // Set pointers to any clientData or callData here.
    Interaction* interactor;

    InteractionCallback(const InteractionCallback&) = delete;
    void operator=(const InteractionCallback&) = delete;
};


#endif //INC_3DVIS_INTERACTIONCALLBACK_H
