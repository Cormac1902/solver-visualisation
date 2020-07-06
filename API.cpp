//
// Created by cormac on 30/06/2020.
//

#include <iostream>
#include "API.h"
#include "Display.h"

[[noreturn]] void API::run_add_socket() {
    std::cout << "Running add socket" << std::endl;

    while (true) {
        zmq::message_t request;

        // receive a request from client
        add_socket.recv(&request);

        std::cout << "Req received" << std::endl;

        msgpack::sbuffer buffer;
        buffer.write(static_cast<const char *>(request.data()), request.size());

        // deserialize it.
        msgpack::object_handle result;
        unpack(result, buffer.data(), buffer.size());

//        std::cout << "Add request received" << std::endl;

        // print the deserialized object.
        msgpack::object obj(result.get());
        std::cout << obj << std::endl;

        // convert it into statically typed object.
        auto clause = obj.as<std::vector<long>>();

        // send the reply to the client
//        zmq::message_t reply(1);
//        memcpy(reply.data(), "1", 5);

        Display::addEdgesFromClause(clause);

//        add_socket.send(reply);
    }
}

[[noreturn]] void API::run_remove_socket() {
    std::cout << "Running remove socket" << std::endl;
    while (true) {
        zmq::message_t request;

        // receive a request from client
        remove_socket.recv(&request);
        msgpack::sbuffer buffer;
        buffer.write(static_cast<const char *>(request.data()), request.size());

        // deserialize it.
        msgpack::object_handle result;
        unpack(result, buffer.data(), buffer.size());

        std::cout << "Remove request received" << std::endl;

        // print the deserialized object.
        msgpack::object obj(result.get());
        std::cout << obj << std::endl;

        // convert it into statically typed object.
        auto clause = obj.as<std::vector<long>>();

        // send the reply to the client
        zmq::message_t reply(5);
        memcpy(reply.data(), "World", 5);

        Display::removeEdgesFromClause(clause);

        remove_socket.send(reply);
    }
}