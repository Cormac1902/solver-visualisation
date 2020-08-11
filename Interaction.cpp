//
// Created by Cormac on 24-Apr-20.
//

#include "Interaction.h"
#include "Display.h"
#include "API.h"
#include "thread"

void Interaction::OnKeyPress() {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    if (key.substr(0, 3) == "KP_") {
        key = key.substr(3, key.length() - 3);
    }

    if (key == "s" || key == "S") {
        display->solve();
    } else if (key == "w" || key == "W") {
//        std::thread walksatThread(Display::walksat, display, api);
//        walksatThread.join();
        Display::walksat(display, api);
        std::cout << "Finished solving" << std::endl;
    }
    else {
        try {
//            api->setGraphLevel(std::stoi(key));
            display->changeGraph(std::stoi(key));
        } catch (std::invalid_argument &e) {
        }
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

vtkStandardNewMacro(Interaction)
