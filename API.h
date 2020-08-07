//
// Created by cormac on 30/06/2020.
//
#include <zmq.hpp>
#include "msgpack.hpp"

#ifndef INC_3DVIS_API_H
#define INC_3DVIS_API_H
#endif //INC_3DVIS_API_H

static int ADD_CLAUSE_SOCKET = 29786;
static int REMOVE_CLAUSE_SOCKET = 29787;
static int VARIABLE_ASSIGNMENT_SOCKET = 29788;
static int VARIABLE_ACTIVITY_SOCKET = 29789;

class Display;

class API {
private:
    zmq::context_t context;
    zmq::socket_t add_clause_socket, remove_clause_socket, variable_assignment_socket, variable_activity_socket;
    Display &display;

    template <typename T>
    static T unpack(zmq::message_t &message);

    static std::vector<long> unpack_vector(zmq::message_t &message);

    static std::string port_string(int port) {
        return "tcp://*:" + std::to_string(port);
    }

public:
    explicit API(Display *display) : context(4),
                                     add_clause_socket(context, ZMQ_PULL),
                                     remove_clause_socket(context, ZMQ_PULL),
                                     variable_assignment_socket(context, ZMQ_PULL),
                                     variable_activity_socket(context, ZMQ_PULL),
                                     display(*display) {
        add_clause_socket.bind(port_string(ADD_CLAUSE_SOCKET));
        remove_clause_socket.bind(port_string(REMOVE_CLAUSE_SOCKET));
        variable_assignment_socket.bind(port_string(VARIABLE_ASSIGNMENT_SOCKET));
        variable_activity_socket.bind(port_string(VARIABLE_ACTIVITY_SOCKET));
    }

    ~API() {
        zmq_close(&add_clause_socket);
        zmq_close(&remove_clause_socket);
        zmq_close(&variable_assignment_socket);
        zmq_close(&variable_activity_socket);
        zmq_ctx_destroy(&context);
    }

    [[noreturn]] void run_add_clause_socket();

    [[noreturn]] void run_remove_clause_socket();

    [[noreturn]] void run_assign_variable_truth_value();

    [[noreturn]] void run_increase_variable_activity_socket();
};
