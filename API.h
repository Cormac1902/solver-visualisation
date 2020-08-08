//
// Created by cormac on 30/06/2020.
//
#include <zmq.hpp>
#include "msgpack.hpp"
#include "Display.h"
#include "APIHelper.hpp"

#ifndef INC_3DVIS_API_H
#define INC_3DVIS_API_H
#endif //INC_3DVIS_API_H

class Display;

class API {
private:
    zmq::context_t context;
    zmq::socket_t add_clause_socket, remove_clause_socket, variable_assignment_socket, variable_activity_socket, render_socket_push;
    Display &display;

    static unsigned ADD_CLAUSE_SOCKET;
    static unsigned REMOVE_CLAUSE_SOCKET;
    static unsigned VARIABLE_ASSIGNMENT_SOCKET;
    static unsigned VARIABLE_ACTIVITY_SOCKET;

    static std::vector<long> unpack_vector(zmq::message_t &message);

    void send_render(unsigned opt);

    void send_edges_render() { send_render(Display::EDGES_UPDATE); }

    void send_vertices_render() { send_render(Display::VERTICES_UPDATE); }

public:
    explicit API(Display *display) : context(1),
                                     add_clause_socket(context, ZMQ_PULL),
                                     remove_clause_socket(context, ZMQ_PULL),
                                     variable_assignment_socket(context, ZMQ_PULL),
                                     variable_activity_socket(context, ZMQ_PULL),
                                     render_socket_push(context, ZMQ_PUSH),
                                     display(*display) {
        add_clause_socket.bind(APIHelper::port_string(ADD_CLAUSE_SOCKET));
        remove_clause_socket.bind(APIHelper::port_string(REMOVE_CLAUSE_SOCKET));
        variable_assignment_socket.bind(APIHelper::port_string(VARIABLE_ASSIGNMENT_SOCKET));
        variable_activity_socket.bind(APIHelper::port_string(VARIABLE_ACTIVITY_SOCKET));
        render_socket_push.connect(APIHelper::prepend_string("tcp://localhost:", Display::RENDER_SOCKET));
    }

    ~API() {
        zmq_close(&add_clause_socket);
        zmq_close(&remove_clause_socket);
        zmq_close(&variable_assignment_socket);
        zmq_close(&variable_activity_socket);
        zmq_close(&render_socket_push);
        zmq_ctx_destroy(&context);
    }

    [[noreturn]] void run_add_clause_socket();

    [[noreturn]] void run_remove_clause_socket();

    [[noreturn]] void run_assign_variable_truth_value();

    [[noreturn]] void run_increase_variable_activity_socket();

    void run();
};
