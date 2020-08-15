//
// Created by Cormac on 24-Apr-20.
//

#include "Interaction.h"
#include "API.h"
#include "thread"

void Interaction::OnKeyPress() {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    if (key.substr(0, 3) == "KP_") {
        key = key.substr(3, key.length() - 3);
    }

    if (key.length() == 1) {
        auto solverEnum = (char)toupper(key.at(0));
        if (isalpha(solverEnum)) {
            display->solve(Display::solver_enum_from_char(solverEnum));
        } else {
            display->changeGraph(solverEnum - '0');
//            display->renderWindow->Render();
        }
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

vtkStandardNewMacro(Interaction)
