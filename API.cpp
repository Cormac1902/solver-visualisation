//
// Created by cormac on 30/06/2020.
//

#include "API.h"
#include "Display.h"

template <typename T>
T API::unpack(zmq::message_t &message) {
    msgpack::sbuffer buffer;
    buffer.write(static_cast<const char *>(message.data()), message.size());

    msgpack::object_handle result;
    msgpack::unpack(result, buffer.data(), buffer.size());

//    msgpack::object obj(result.get());

    return result.get().as<T>();
}

std::vector<long> API::unpack_vector(zmq::message_t &message) {
    return unpack<std::vector<long>>(message);
}

/*T API::unpack_t(zmq::message_t &message) {
    auto resultPtr = unpack(message);

    auto clause = resultPtr->get().as<T>();

    delete resultPtr;

    return clause;
}

std::vector<long> API::unpack_vector(zmq::message_t &message) {
    return unpack_t<std::vector<long>>(message);
}

long API::unpack_long(zmq::message_t &message) {
    auto resultPtr = unpack(message);

    auto var = resultPtr->get().as<long>();

    delete resultPtr;

    return var;
}

std::pair<long, bool> API::unpack_long_bool(zmq::message_t &message) {
    auto resultPtr = unpack(message);

    auto var = resultPtr->get().as<std::pair<long, bool>>();

    delete resultPtr;

    return var;
}*/

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

        pair<long, bool> var = {0, false};

        try {
            var = unpack<pair<long, bool>>(request);
        } catch (msgpack::v1::type_error &e) {
            var.first = unpack<long>(request);
        }

        display.assignVariable(abs(var.first), var.first < 0, var.second);
    }
}

[[noreturn]] void API::run_increase_variable_activity_socket() {
    while (true) {
        zmq::message_t request;

        variable_activity_socket.recv(&request);

        display.increaseVariableActivity(abs(unpack<long>(request)));
    }
}