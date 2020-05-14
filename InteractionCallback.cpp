//
// Created by corma on 25-Apr-20.
//

#include "InteractionCallback.h"

void InteractionCallback::Execute(vtkObject *caller, unsigned long evId, void *callData) {

    // Note the use of reinterpret_cast to cast the caller to the expected type.
    auto *interactorCaller = reinterpret_cast<vtkRenderWindowInteractor *>(caller);
    // Just do this to demonstrate who called callback and the event that
    // triggered it.
    std::cout << interactorCaller->GetClassName() << "  Event Id: " << evId << std::endl;

    // Now print the camera orientation.
};