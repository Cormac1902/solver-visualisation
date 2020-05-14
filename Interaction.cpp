//
// Created by Cormac on 24-Apr-20.
//

#include "Interaction.h"
#include "Display.h"

void Interaction::OnKeyPress() {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    int keyInt;
    if (key.substr(0, 3) == "KP_") {
        key = key.substr(3, key.length() - 3);
    }

    try {
        keyInt = std::stoi(key);
        keyInt--;
        Display::changeGraph(keyInt);
    } catch(std::invalid_argument& e) {
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

vtkStandardNewMacro(Interaction);
