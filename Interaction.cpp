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
    } else if (key == "z" || key == "Z") {
/*        zmq::context_t context (1);
        zmq::socket_t socket (context, ZMQ_REQ);

        std::cout << "Connecting to hello world server…" << std::endl;
        socket.connect ("tcp://localhost:29786");

        //  Do 10 requests, waiting each time for a response
        for (int request_nbr = 0; request_nbr != 10; request_nbr++) {
            zmq::message_t request (5);
            memcpy (request.data (), "Hello", 5);
            std::cout << "Sending Hello " << request_nbr << "…" << std::endl;
            socket.send (request);

            //  Get the reply.
            zmq::message_t reply;
            socket.recv (&reply);
            std::cout << "Received World " << request_nbr << std::endl;
        }*/
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
