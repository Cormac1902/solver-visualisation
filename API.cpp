//
// Created by cormac on 30/06/2020.
//

#include <mutex>
#include <thread>
#include "API.h"

unsigned API::ADD_CLAUSE_SOCKET = 29786;
unsigned API::REMOVE_CLAUSE_SOCKET = 29787;
unsigned API::VARIABLE_ASSIGNMENT_SOCKET = 29788;
unsigned API::VARIABLE_ACTIVITY_SOCKET = 29789;

std::mutex display_mutex;

std::vector<long> API::unpack_vector(zmq::message_t &message) {
    return APIHelper::unpack<std::vector<long>>(message);
}

void API::send_render(unsigned int opt) {
    msgpack::sbuffer buffer;
    msgpack::pack(buffer, opt);

    zmq::message_t request(buffer.size());

    memcpy(request.data(), buffer.data(), buffer.size());

    try {
        render_socket_push.send(request, zmq::send_flags::none);
    } catch (zmq::error_t &e) {
        std::cout << "Error" << std::endl;
    }
}

[[noreturn]] void API::run_add_clause_socket() {
    while (true) {
        zmq::message_t request;

        // Receive a request from client
        if (add_clause_socket.recv(request)) {
            std::scoped_lock lock{display_mutex};
            display.addEdgesFromClause(unpack_vector(request));
            send_edges_render();
        }
    }
}

[[noreturn]] void API::run_remove_clause_socket() {

    while (true) {
        zmq::message_t request;

        // Receive a request from client
        if (remove_clause_socket.recv(request)) {
            std::scoped_lock lock{display_mutex};
            display.removeEdgesFromClause(unpack_vector(request));
            send_edges_render();
        }
    }
}

[[noreturn]] void API::run_assign_variable_truth_value() {
    while (true) {
        zmq::message_t request;

        if (variable_assignment_socket.recv(request)) {
            pair<long, bool> var = {0, false};

            try {
                var = APIHelper::unpack<pair<long, bool>>(request);
            } catch (msgpack::v1::type_error &e) {
                var.first = APIHelper::unpack<long>(request);
            }

            std::scoped_lock lock{display_mutex};

            display.assignVariable(abs(var.first), var.first < 0, var.second);

            send_vertices_render();
        }
    }
}

[[noreturn]] void API::run_increase_variable_activity_socket() {
    while (true) {
        zmq::message_t request;

        if (variable_activity_socket.recv(request)) {
            std::scoped_lock lock{display_mutex};
            display.increaseVariableActivity(abs(APIHelper::unpack<long>(request)));
            send_vertices_render();
        }
    }
}

void API::run() {
    std::thread addThread(&API::run_add_clause_socket, this);
    std::thread removeThread(&API::run_remove_clause_socket, this);
    std::thread varActivityThread(&API::run_increase_variable_activity_socket, this);
    std::thread assignVarThread(&API::run_assign_variable_truth_value, this);

    addThread.join();
    removeThread.join();
    varActivityThread.join();
    assignVarThread.join();
}