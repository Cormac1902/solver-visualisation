//
// Created by Cormac on 24-Apr-20.
//

#include "Interaction.h"
#include "Display.h"
#include <zmq.hpp>

void Interaction::OnKeyPress() {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    if (key.substr(0, 3) == "KP_") {
        key = key.substr(3, key.length() - 3);
    }

    if (key == "s" || key == "S") {
        Display::solve();
    } else if (key == "w" || key == "W") {
        Display::walksat();
    } else {
        try {
            Display::changeGraph(std::stoi(key));
        } catch (std::invalid_argument &e) {
        }
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

vtkStandardNewMacro(Interaction)
