//
// Created by cormac on 30/06/2020.
//

#include <iostream>
#include "API.h"
#include "Display.h"

msgpack::object API::unpack(zmq::message_t &message) {
    msgpack::sbuffer buffer;
    buffer.write(static_cast<const char *>(message.data()), message.size());

    // deserialize it.
    msgpack::object_handle result;
    msgpack::unpack(result, buffer.data(), buffer.size());

    msgpack::object obj(result.get());

    return obj;
}

std::vector<long> API::unpack_vector(zmq::message_t& message) {
    return unpack(message).as<std::vector<long>>();
}

unsigned long API::unpack_long(zmq::message_t &message) {
    return unpack(message).as<unsigned long>();
}

[[noreturn]] void API::run_add_clause_socket() {
    while (true) {
        zmq::message_t request;

        // Receive a request from client
        add_clause_socket.recv(&request);

        Display::addEdgesFromClause(unpack_vector(request));
    }
}

[[noreturn]] void API::run_remove_clause_socket() {

    while (true) {
        zmq::message_t request;

        // Receive a request from client
        remove_clause_socket.recv(&request);

        Display::removeEdgesFromClause(unpack_vector(request));
    }
}

[[noreturn]] void API::run_increase_variable_activity_socket() {
    while(true) {
        zmq::message_t request;

        variable_activity_socket.recv(&request);

        Display::increaseVariableActivity(unpack_long(request));
    }
}