//
// Created by cormac on 30/06/2020.
//

#include "API.h"
#include "Display.h"

msgpack::object_handle *API::unpack(zmq::message_t &message) {
    msgpack::sbuffer buffer;
    buffer.write(static_cast<const char *>(message.data()), message.size());

    // deserialize it.
    auto *result = new msgpack::object_handle;
    msgpack::unpack(*result, buffer.data(), buffer.size());

    return result;
}

std::vector<long> API::unpack_vector(zmq::message_t &message) {
    auto resultPtr = unpack(message);

    auto clause = resultPtr->get().as<std::vector<long>>();

    delete resultPtr;

    return clause;
}

long API::unpack_long(zmq::message_t &message) {
    msgpack::sbuffer buffer;
    buffer.write(static_cast<const char *>(message.data()), message.size());

    // deserialize it.
    msgpack::object_handle result;
    msgpack::unpack(result, buffer.data(), buffer.size());

    return result.get().as<long>();
}

[[noreturn]] void API::run_add_clause_socket() {
    while (true) {
        zmq::message_t request;

        // Receive a request from client
        add_clause_socket.recv(&request);

        display.addEdgesFromClause(unpack_vector(request));
    }
}

[[noreturn]] void API::run_remove_clause_socket() {

    while (true) {
        zmq::message_t request;

        // Receive a request from client
        remove_clause_socket.recv(&request);

        display.removeEdgesFromClause(unpack_vector(request));
    }
}

[[noreturn]] void API::run_assign_variable_truth_value() {
    while (true) {
        zmq::message_t request;

        variable_assignment_socket.recv(&request);

        auto var = unpack_long(request);

        display.assignVariable(abs(var), var < 0);
    }
}

[[noreturn]] void API::run_increase_variable_activity_socket() {
    while (true) {
        zmq::message_t request;

        variable_activity_socket.recv(&request);

        display.increaseVariableActivity(abs(unpack_long(request)));
    }
}