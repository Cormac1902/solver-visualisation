//
// Created by cormac on 30/06/2020.
//

#include <iostream>
#include "API.h"
#include "Display.h"

std::vector<long> API::unpack_vector(zmq::message_t& message) {
    msgpack::sbuffer buffer;
    buffer.write(static_cast<const char *>(message.data()), message.size());

    // deserialize it.
    msgpack::object_handle result;
    unpack(result, buffer.data(), buffer.size());

    // print the deserialized object.
    msgpack::object obj(result.get());
//    std::cout << obj << std::endl;

    return obj.as<std::vector<long>>();
}

[[noreturn]] void API::run_add_socket() {
//    std::cout << "Running add socket" << std::endl;

    while (true) {
        zmq::message_t request;

        // Receive a request from client
        add_socket.recv(&request);

//        std::cout << "Add request received" << std::endl;

        Display::addEdgesFromClause(unpack_vector(request));
    }
}

[[noreturn]] void API::run_remove_socket() {
//    std::cout << "Running remove socket" << std::endl;

    while (true) {
        zmq::message_t request;

        // Receive a request from client
        remove_socket.recv(&request);

//        std::cout << "Remove request received" << std::endl;

        Display::removeEdgesFromClause(unpack_vector(request));
    }
}