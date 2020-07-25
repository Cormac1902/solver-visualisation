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
    zmq::socket_t add_clause_socket, remove_clause_socket, variable_activity_socket;

    static msgpack::object unpack(zmq::message_t &message);
    static std::vector<long> unpack_vector(zmq::message_t &message);
    static unsigned long unpack_long(zmq::message_t &message);

public:
    API() : context(3),
            add_clause_socket(context, ZMQ_PULL),
            remove_clause_socket(context, ZMQ_PULL),
            variable_activity_socket(context, ZMQ_PULL) {
        add_clause_socket.bind("tcp://*:29786");
        remove_clause_socket.bind("tcp://*:29787");
        variable_activity_socket.bind("tcp://*:29788");
    }

    [[noreturn]] void run_add_clause_socket();

    [[noreturn]] void run_remove_clause_socket();

    [[noreturn]] void run_increase_variable_activity_socket();
};
