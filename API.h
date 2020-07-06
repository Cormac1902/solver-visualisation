//
// Created by cormac on 30/06/2020.
//
#include <zmq.hpp>
#include "msgpack.hpp"

#ifndef INC_3DVIS_API_H
#define INC_3DVIS_API_H

#endif //INC_3DVIS_API_H

class API {
private:
    zmq::context_t context;
    zmq::socket_t add_socket, remove_socket;

public:
    API() : context(1),
            add_socket(context, ZMQ_PULL),
            remove_socket(context, ZMQ_PULL) {
        add_socket.bind("tcp://*:29786");
        remove_socket.bind("tcp://*:29787");
    }

    [[noreturn]] void run_add_socket();
    [[noreturn]] void run_remove_socket();
};
